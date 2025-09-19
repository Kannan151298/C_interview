/*
===========================================================================
 
FILE:         ds90ux98x_init.h
 
$File$
 
===========================================================================
Copyright (C) GM Global Technology Operations LLC 2025.
All Rights Reserved.
GM Confidential Restricted.
 
===========================================================================
*/
#ifndef DS90UX98X_INIT_H
#define DS90UX98X_INIT_H

#define SER_RESET_DIGITAL_ALL 0x02
#define SER_RESET_PLL_ONLY    0x30
#define BRIDGECHIP_EXIT_FROM_DPLINK_UNTRAIN 0x02
BridgeChip_StatusType ser_config_update(int32 i2c_fh);
BridgeChip_StatusType serdes_get_ser_link_status(int32 fd, int32 print);
BridgeChip_StatusType serdes_get_daisy_link_status(int32 fd);
BridgeChip_StatusType dser_config_update(int32 i2c_fh);
BridgeChip_StatusType dser_daisy_config_update(int32 i2c_fh);
BridgeChip_StatusType ser_reset_chip(int32 fd);
BridgeChip_StatusType dser_config_update_recovery(int32 i2c_fh);
BridgeChip_StatusType serdes_check_vp_status(int32 fd);
BridgeChip_StatusType recovery_ser_config_update(int32 i2c_fh);
BridgeChip_StatusType serdes_wait_ser_fpd_linkup(int32 fd, int32 timeout);
void serdes_get_983_dp_rx_status(int32 fd);
BridgeChip_StatusType serdes_monitor_983_dp_link(int32 fd);
BridgeChip_StatusType serdes_check_983_dprx(int32 fd);
BridgeChip_StatusType serializer_video_input_reset(int32 fd);
BridgeChip_StatusType ser_config_vp(int32 fd);
BridgeChip_StatusType serdes_check_vp_status(int32 fd);
BridgeChip_StatusType serdes_monitor_des_video(int32 fd, bool32 reset);
BridgeChip_StatusType serdes_monitor_des_video_daisy(int32 fd, bool32 reset);
BridgeChip_StatusType ser_reset_chip(int32 fd);
void serdes_diagnostic_983dprx(int32 fd);
#endif
