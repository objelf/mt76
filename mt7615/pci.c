// SPDX-License-Identifier: ISC
/* Copyright (C) 2019 MediaTek Inc.
 *
 * Author: Ryder Lee <ryder.lee@mediatek.com>
 *         Felix Fietkau <nbd@nbd.name>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "mt7615.h"

static const struct pci_device_id mt7615_pci_device_table[] = {
	{ PCI_DEVICE(0x14c3, 0x7615) },
	{ PCI_DEVICE(0x14c3, 0x7663) },
	{ },
};

static int mt7615_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	const u32 *map;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	ret = pcim_iomap_regions(pdev, BIT(0), pci_name(pdev));
	if (ret)
		return ret;

	pci_set_master(pdev);

	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return ret;

	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	map = id->device == 0x7663 ? mt7663e_reg_map : mt7615e_reg_map;
	return mt7615_mmio_probe(&pdev->dev, pcim_iomap_table(pdev)[0],
				 pdev->irq, map);
}

static void mt7615_pci_remove(struct pci_dev *pdev)
{
	struct mt76_dev *mdev = pci_get_drvdata(pdev);
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);

	mt7615_unregister_device(dev);
	pci_free_irq_vectors(pdev);
}

static int __maybe_unused mt7615_pci_suspend(struct pci_dev *pdev,
					     pm_message_t state)
{
	struct mt76_dev *mdev = pci_get_drvdata(pdev);
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	int i, err;

	if (!test_bit(MT76_STATE_SUSPEND, &dev->mphy.state) &&
	    mt7615_firmware_offload(dev)) {
		int err;

		err = mt7615_mcu_set_hif_suspend(dev, true);
		if (err < 0)
			return err;
	}

	napi_disable(&mdev->tx_napi);
	for (i = 0; i < ARRAY_SIZE(mdev->q_tx); i++)
		mt76_queue_tx_cleanup(dev, i, true);
	tasklet_kill(&mdev->tx_tasklet);

	for (i = 0; i < ARRAY_SIZE(mdev->q_rx); i++) {
		napi_disable(&mdev->napi[i]);
		mt76_queue_rx_reset(dev, i);
	}
	tasklet_kill(&dev->irq_tasklet);

	err = -EIO;

	if (!mt76_poll_msec(dev, MT_PDMA_BUSY_STATUS, MT_PDMA_TX_IDX_BUSY, 0, 1000)) {
		dev_err(dev->mt76.dev, "PDMA Tx Busy\n");
		goto restore;
	}

	if (!mt76_poll_msec(dev, MT_PLE_PG_INFO, MT_PLE_SRC_CNT, 0, 1000)) {
		dev_err(dev->mt76.dev, "PSE Busy\n");
		goto restore;
	}

	if (!mt76_poll_msec(dev, MT_PDMA_BUSY_STATUS, MT_PDMA_BUSY, 0, 1000)) {
		dev_err(dev->mt76.dev, "PDMA busy\n");
		goto restore;
	}

	mt76_rmw(dev, MT_PDMA_SLP_PROT, MT_PDMA_AXI_SLPPROT_ENABLE, 1);

	if (!mt76_poll_msec(dev, MT_PDMA_SLP_PROT, MT_PDMA_AXI_SLPPROT_RDY,
			    MT_PDMA_AXI_SLPPROT_RDY, 1000)) {
		dev_err(dev->mt76.dev, "PDMA sleep pretection not ready\n");
		goto restore;
	}

	pci_enable_wake(pdev, pci_choose_state(pdev, state), 1);
	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	mt7615_firmware_own(dev);

	return 0;

restore:
	for (i = 0; i < ARRAY_SIZE(mdev->q_rx); i++)
		napi_enable(&mdev->napi[i]);

	napi_enable(&mdev->tx_napi);

	if (!test_bit(MT76_STATE_SUSPEND, &dev->mphy.state) &&
	    mt7615_firmware_offload(dev))
		mt7615_mcu_set_hif_suspend(dev, false);

	return err;
}

static int __maybe_unused mt7615_pci_resume(struct pci_dev *pdev)
{
	struct mt76_dev *mdev = pci_get_drvdata(pdev);
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	bool pdma_reset;
	int i, err = 0;

	mt7615_driver_own(dev);

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	mt76_rmw(dev, MT_PDMA_SLP_PROT, MT_PDMA_AXI_SLPPROT_ENABLE, 0);

	pdma_reset = !mt76_rr(dev, MT_WPDMA_TX_RING0_CTRL0) &&
		      !mt76_rr(dev, MT_WPDMA_TX_RING0_CTRL1);
	if (pdma_reset) {
		/* TODO */
		dev_err(dev->mt76.dev, "PDMA have to reinitialization\n");
	}

        if (is_mt7663(&dev->mt76))
                mt76_wr(dev, MT_PCIE_IRQ_ENABLE, 1);

	for (i = 0; i < ARRAY_SIZE(mdev->q_rx); i++)
		napi_enable(&mdev->napi[i]);

	napi_enable(&mdev->tx_napi);

	if (!test_bit(MT76_STATE_SUSPEND, &dev->mphy.state) &&
	    mt7615_firmware_offload(dev))
		err = mt7615_mcu_set_hif_suspend(dev, false);

	return err;
}


struct pci_driver mt7615_pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= mt7615_pci_device_table,
	.probe		= mt7615_pci_probe,
	.remove		= mt7615_pci_remove,
#ifdef CONFIG_PM
	.suspend 	= mt7615_pci_suspend,
	.resume 	= mt7615_pci_resume,
#endif /* CONFIG_PM */
};

MODULE_DEVICE_TABLE(pci, mt7615_pci_device_table);
MODULE_FIRMWARE(MT7615_FIRMWARE_CR4);
MODULE_FIRMWARE(MT7615_FIRMWARE_N9);
MODULE_FIRMWARE(MT7615_ROM_PATCH);
MODULE_FIRMWARE(MT7663_FIRMWARE_N9);
MODULE_FIRMWARE(MT7663_ROM_PATCH);
