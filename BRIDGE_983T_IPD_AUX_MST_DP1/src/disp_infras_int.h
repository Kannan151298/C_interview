/*
===========================================================================
FILE:         disp_infras_int.h
$File$
===========================================================================
Copyright (C) GM Global Technology Operations LLC 2024.
All Rights Reserved.
GM Confidential Restricted.
===========================================================================
*/

#ifndef DISP_INFRAS__H
#define DISP_INFRAS__H

#include "bc-com-res-mgr.h"

#define DISPLAY_WKUP_CTRL_ON  1
#define DISPLAY_WKUP_CTRL_OFF 0
#define DISPLAY_PDB_CTRL_ON  1
#define DISPLAY_PDB_CTRL_OFF 0
#define DISPLAY_CHAIN_CTRL_ON  1
#define DISPLAY_CHAIN_CTRL_OFF 0

#define BRIDGECHIP_GPIO_PDB                   "/dev/gpio/cpu_dp1_ctrl_en/value"
#define BRIDGECHIP_INIT_SYNC_POINT() \
do \
{ \
    FILE *f; \
    char dp_status_path[128]; \
    memset(dp_status_path, 0, sizeof(dp_status_path)); \
    sprintf (dp_status_path, "/tmp/FF_INIT_SYNC"); \
    f = fopen(dp_status_path, "w+"); \
    fclose(f); \
} while(0);

typedef enum {
    DISP_NOTIFY_TYPE_IPD,
    DISP_NOTIFY_TYPE_FCC
} disp_notify_type_t;

void display_state_notify(char *display_state, disp_notify_type_t notify_type);
int BridgeChip_pdb_control(char *pdb_path, unsigned char val);
BridgeChip_StatusType display_power_ctl(int32 enable);
/*
 * Control for system shutdown, suspend and resume.
 * Will do complete SER and DES reset and initialization
*/
BridgeChip_StatusType disp_infras_display_chain_control(int32 i2c_fh, int32 sate);
/*
 * Used for link lost recovery. Keep DP linkup but partially reset SER.
 * Do complete display switch off and DES initialization
*/
BridgeChip_StatusType disp_infras_display_chain_recovery(int32 i2c_fh, int32 sate,
        disp_notify_type_t notify_type);
#endif
