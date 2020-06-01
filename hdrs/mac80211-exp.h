#ifndef __MAC80211_EXP_H
#define __MAC80211_EXP_H
#define ieee80211_tdls_oper_request __mt76_ieee80211_tdls_oper_request
#define ieee80211_find_sta_by_ifaddr __mt76_ieee80211_find_sta_by_ifaddr
#define ieee80211_find_sta __mt76_ieee80211_find_sta
#define ieee80211_sta_block_awake __mt76_ieee80211_sta_block_awake
#define ieee80211_sta_eosp __mt76_ieee80211_sta_eosp
#define ieee80211_send_eosp_nullfunc __mt76_ieee80211_send_eosp_nullfunc
#define ieee80211_sta_set_buffered __mt76_ieee80211_sta_set_buffered
#define ieee80211_sta_register_airtime __mt76_ieee80211_sta_register_airtime
#define ieee80211_update_mu_groups __mt76_ieee80211_update_mu_groups
#define ieee80211_calc_rx_airtime __mt76_ieee80211_calc_rx_airtime
#define ieee80211_calc_tx_airtime __mt76_ieee80211_calc_tx_airtime
#define __ieee80211_get_radio_led_name __mt76___ieee80211_get_radio_led_name
#define __ieee80211_get_assoc_led_name __mt76___ieee80211_get_assoc_led_name
#define __ieee80211_get_tx_led_name __mt76___ieee80211_get_tx_led_name
#define __ieee80211_get_rx_led_name __mt76___ieee80211_get_rx_led_name
#define __ieee80211_create_tpt_led_trigger __mt76___ieee80211_create_tpt_led_trigger
#define lockdep_rht_mutex_is_held __mt76_lockdep_rht_mutex_is_held
#define lockdep_rht_bucket_is_held __mt76_lockdep_rht_bucket_is_held
#define rhashtable_insert_slow __mt76_rhashtable_insert_slow
#define rhashtable_walk_enter __mt76_rhashtable_walk_enter
#define rhashtable_walk_exit __mt76_rhashtable_walk_exit
#define rhashtable_walk_start_check __mt76_rhashtable_walk_start_check
#define rhashtable_walk_next __mt76_rhashtable_walk_next
#define rhashtable_walk_peek __mt76_rhashtable_walk_peek
#define rhashtable_walk_stop __mt76_rhashtable_walk_stop
#define rhashtable_init __mt76_rhashtable_init
#define rhltable_init __mt76_rhltable_init
#define rhashtable_free_and_destroy __mt76_rhashtable_free_and_destroy
#define rhashtable_destroy __mt76_rhashtable_destroy
#define rht_bucket_nested __mt76_rht_bucket_nested
#define rht_bucket_nested_insert __mt76_rht_bucket_nested_insert
#define ieee80211_chswitch_done __mt76_ieee80211_chswitch_done
#define ieee80211_ap_probereq_get __mt76_ieee80211_ap_probereq_get
#define ieee80211_beacon_loss __mt76_ieee80211_beacon_loss
#define ieee80211_connection_loss __mt76_ieee80211_connection_loss
#define ieee80211_cqm_rssi_notify __mt76_ieee80211_cqm_rssi_notify
#define ieee80211_cqm_beacon_loss_notify __mt76_ieee80211_cqm_beacon_loss_notify
#define ieee80211_tkip_add_iv __mt76_ieee80211_tkip_add_iv
#define ieee80211_get_tkip_p1k_iv __mt76_ieee80211_get_tkip_p1k_iv
#define ieee80211_get_tkip_rx_p1k __mt76_ieee80211_get_tkip_rx_p1k
#define ieee80211_get_tkip_p2k __mt76_ieee80211_get_tkip_p2k
#define ieee80211_csa_finish __mt76_ieee80211_csa_finish
#define ieee80211_nan_func_terminated __mt76_ieee80211_nan_func_terminated
#define ieee80211_nan_func_match __mt76_ieee80211_nan_func_match
#define ieee80211_tx_prepare_skb __mt76_ieee80211_tx_prepare_skb
#define ieee80211_tx_dequeue __mt76_ieee80211_tx_dequeue
#define ieee80211_next_txq __mt76_ieee80211_next_txq
#define __ieee80211_schedule_txq __mt76___ieee80211_schedule_txq
#define ieee80211_txq_airtime_check __mt76_ieee80211_txq_airtime_check
#define ieee80211_txq_may_transmit __mt76_ieee80211_txq_may_transmit
#define ieee80211_txq_schedule_start __mt76_ieee80211_txq_schedule_start
#define ieee80211_csa_update_counter __mt76_ieee80211_csa_update_counter
#define ieee80211_csa_set_counter __mt76_ieee80211_csa_set_counter
#define ieee80211_csa_is_complete __mt76_ieee80211_csa_is_complete
#define ieee80211_beacon_get_template __mt76_ieee80211_beacon_get_template
#define ieee80211_beacon_get_tim __mt76_ieee80211_beacon_get_tim
#define ieee80211_proberesp_get __mt76_ieee80211_proberesp_get
#define ieee80211_pspoll_get __mt76_ieee80211_pspoll_get
#define ieee80211_nullfunc_get __mt76_ieee80211_nullfunc_get
#define ieee80211_probereq_get __mt76_ieee80211_probereq_get
#define ieee80211_rts_get __mt76_ieee80211_rts_get
#define ieee80211_ctstoself_get __mt76_ieee80211_ctstoself_get
#define ieee80211_get_buffered_bc __mt76_ieee80211_get_buffered_bc
#define ieee80211_reserve_tid __mt76_ieee80211_reserve_tid
#define ieee80211_unreserve_tid __mt76_ieee80211_unreserve_tid
#define ieee80211_send_bar __mt76_ieee80211_send_bar
#define ieee80211_start_tx_ba_session __mt76_ieee80211_start_tx_ba_session
#define ieee80211_start_tx_ba_cb_irqsafe __mt76_ieee80211_start_tx_ba_cb_irqsafe
#define ieee80211_stop_tx_ba_session __mt76_ieee80211_stop_tx_ba_session
#define ieee80211_stop_tx_ba_cb_irqsafe __mt76_ieee80211_stop_tx_ba_cb_irqsafe
#define ieee80211_scan_completed __mt76_ieee80211_scan_completed
#define ieee80211_sched_scan_results __mt76_ieee80211_sched_scan_results
#define ieee80211_sched_scan_stopped __mt76_ieee80211_sched_scan_stopped
#define ieee80211_iter_chan_contexts_atomic __mt76_ieee80211_iter_chan_contexts_atomic
#define __alloc_bucket_spinlocks __mt76___alloc_bucket_spinlocks
#define free_bucket_spinlocks __mt76_free_bucket_spinlocks
#define ieee80211_iter_keys __mt76_ieee80211_iter_keys
#define ieee80211_iter_keys_rcu __mt76_ieee80211_iter_keys_rcu
#define ieee80211_gtk_rekey_notify __mt76_ieee80211_gtk_rekey_notify
#define ieee80211_get_key_rx_seq __mt76_ieee80211_get_key_rx_seq
#define ieee80211_set_key_rx_seq __mt76_ieee80211_set_key_rx_seq
#define ieee80211_remove_key __mt76_ieee80211_remove_key
#define ieee80211_gtk_rekey_add __mt76_ieee80211_gtk_rekey_add
#define ieee80211_report_wowlan_wakeup __mt76_ieee80211_report_wowlan_wakeup
#define ieee80211_get_vht_max_nss __mt76_ieee80211_get_vht_max_nss
#define ieee80211_rate_control_register __mt76_ieee80211_rate_control_register
#define ieee80211_rate_control_unregister __mt76_ieee80211_rate_control_unregister
#define ieee80211_get_tx_rates __mt76_ieee80211_get_tx_rates
#define rate_control_set_rates __mt76_rate_control_set_rates
#define ieee80211_sta_ps_transition __mt76_ieee80211_sta_ps_transition
#define ieee80211_sta_pspoll __mt76_ieee80211_sta_pspoll
#define ieee80211_sta_uapsd_trigger __mt76_ieee80211_sta_uapsd_trigger
#define ieee80211_mark_rx_ba_filtered_frames __mt76_ieee80211_mark_rx_ba_filtered_frames
#define ieee80211_rx_napi __mt76_ieee80211_rx_napi
#define ieee80211_rx_irqsafe __mt76_ieee80211_rx_irqsafe
#define ieee80211_request_smps __mt76_ieee80211_request_smps
#define ieee80211_ready_on_channel __mt76_ieee80211_ready_on_channel
#define ieee80211_remain_on_channel_expired __mt76_ieee80211_remain_on_channel_expired
#define regulatory_set_wiphy_regd __mt76_regulatory_set_wiphy_regd
#define regulatory_set_wiphy_regd_sync_rtnl __mt76_regulatory_set_wiphy_regd_sync_rtnl
#define ieee80211_restart_hw __mt76_ieee80211_restart_hw
#define ieee80211_alloc_hw_nm __mt76_ieee80211_alloc_hw_nm
#define ieee80211_register_hw __mt76_ieee80211_register_hw
#define ieee80211_unregister_hw __mt76_ieee80211_unregister_hw
#define ieee80211_free_hw __mt76_ieee80211_free_hw
#define wiphy_to_ieee80211_hw __mt76_wiphy_to_ieee80211_hw
#define ieee80211_generic_frame_duration __mt76_ieee80211_generic_frame_duration
#define ieee80211_rts_duration __mt76_ieee80211_rts_duration
#define ieee80211_ctstoself_duration __mt76_ieee80211_ctstoself_duration
#define ieee80211_wake_queue __mt76_ieee80211_wake_queue
#define ieee80211_stop_queue __mt76_ieee80211_stop_queue
#define ieee80211_stop_queues __mt76_ieee80211_stop_queues
#define ieee80211_queue_stopped __mt76_ieee80211_queue_stopped
#define ieee80211_wake_queues __mt76_ieee80211_wake_queues
#define ieee80211_iterate_interfaces __mt76_ieee80211_iterate_interfaces
#define ieee80211_iterate_active_interfaces_atomic __mt76_ieee80211_iterate_active_interfaces_atomic
#define ieee80211_iterate_active_interfaces_rtnl __mt76_ieee80211_iterate_active_interfaces_rtnl
#define ieee80211_iterate_stations_atomic __mt76_ieee80211_iterate_stations_atomic
#define wdev_to_ieee80211_vif __mt76_wdev_to_ieee80211_vif
#define ieee80211_vif_to_wdev __mt76_ieee80211_vif_to_wdev
#define ieee80211_queue_work __mt76_ieee80211_queue_work
#define ieee80211_queue_delayed_work __mt76_ieee80211_queue_delayed_work
#define ieee80211_resume_disconnect __mt76_ieee80211_resume_disconnect
#define ieee80211_enable_rssi_reports __mt76_ieee80211_enable_rssi_reports
#define ieee80211_disable_rssi_reports __mt76_ieee80211_disable_rssi_reports
#define ieee80211_ave_rssi __mt76_ieee80211_ave_rssi
#define ieee80211_radar_detected __mt76_ieee80211_radar_detected
#define ieee80211_update_p2p_noa __mt76_ieee80211_update_p2p_noa
#define ieee80211_parse_p2p_noa __mt76_ieee80211_parse_p2p_noa
#define ieee80211_txq_get_depth __mt76_ieee80211_txq_get_depth
#define ieee80211_tx_status_irqsafe __mt76_ieee80211_tx_status_irqsafe
#define ieee80211_tx_status __mt76_ieee80211_tx_status
#define ieee80211_tx_status_ext __mt76_ieee80211_tx_status_ext
#define ieee80211_tx_rate_update __mt76_ieee80211_tx_rate_update
#define ieee80211_report_low_ack __mt76_ieee80211_report_low_ack
#define ieee80211_free_txskb __mt76_ieee80211_free_txskb
#define ieee80211_stop_rx_ba_session __mt76_ieee80211_stop_rx_ba_session
#define ieee80211_manage_rx_ba_offl __mt76_ieee80211_manage_rx_ba_offl
#define ieee80211_rx_ba_timer_expired __mt76_ieee80211_rx_ba_timer_expired
#endif
