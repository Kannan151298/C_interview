/*
===========================================================================
FILE:         disp_infras_int.c
$File$
===========================================================================
Copyright (C) GM Global Technology Operations LLC 2024.
All Rights Reserved.
GM Confidential Restricted.
===========================================================================
*/

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdio.h>
#include "bc-com-res-mgr.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/dispatch.h>

#include "bridgechip_plugin.h"
#include "bridgechip_osal.h"
#include "bridgechip_logger.h"
#include "sus_res_comm_lib.h"
#include "libdisplaywakeup.h"
#include "disp_infras_int.h"
#include "ds90ux98x_init.h"
#include "screen_pwrmgr_common.h"

#define MODULE_NAME "DISP-INFRAS DP1MST IPD AUX"

#define CRITICAL_PRINT(name, _str_, ...)               \
    do { \
            LOG_CRITICAL_INFO(name, _str_ , ##__VA_ARGS__); \
    } while(0)

#define DISP_INFRAS_BL_PM              "ipd-brightness-pm"
#define DISP_INFRAS_SUSRES_PATH        "/dev/ipd/suspend-resume-commander/control"

#define DISP_INFRAS_WKUP_CTRL_PATH     "/dev/wakeup-binder/control"
#define DISP_INFRAS_WKUP_CTRL_NAME     "ipd-wakeup"
#define DISP_INFRAS_WKUP_CTRL_ONLINE   "1"
#define DISP_INFRAS_WKUP_CTRL_OFFLINE  "0"
#define CLIENT_OPERATIONS_CNT 150

const char *notify_type_str[2] = {"IPD","FCC"};

void display_state_notify(char *display_state, disp_notify_type_t notify_type)
{
    LOG_CRITICAL_INFO(MODULE_NAME, "%s display_state %s \n", notify_type_str[notify_type], display_state);
    screen_pwr_data_t msg;
    int server_coid = 0;

    if(0 == strcmp(display_state,"ready")) {
        if (DISP_NOTIFY_TYPE_FCC == notify_type) {
            msg.data = FCC_DISP_STATE_READY;
        } else {
            msg.data = IPD_DISP_STATE_READY;
        }
    } else if (0 == strcmp(display_state,"lost")) {
        if (DISP_NOTIFY_TYPE_FCC == notify_type) {
            msg.data = FCC_DISP_STATE_LOST;
        } else {
            msg.data = IPD_DISP_STATE_LOST;
        }
    } else {
        LOG_CRITICAL_INFO(MODULE_NAME, "%s display_state invalid agr\n",notify_type_str[notify_type]);
        return;
    }
    if ((server_coid = name_open(DISP_STATE_ATTACH_POINT, 0)) == -1) {
        LOG_CRITICAL_INFO(MODULE_NAME, "name open %s failed, error %d(%s)",
                            DISP_STATE_ATTACH_POINT, server_coid, strerror(errno));
    }
    // We would have pre-defined data to stuff here
    msg.hdr.type = 0x00;
    msg.hdr.subtype = 0x00;
    if (MsgSend(server_coid, &msg, sizeof(msg), NULL, 0) == -1) {
        LOG_CRITICAL_INFO(MODULE_NAME, "%s MsgSend error\n",notify_type_str[notify_type]);
        goto fail;
    }
    LOG_CRITICAL_INFO(MODULE_NAME, "display_state_notify Done\n");
fail:
    name_close(server_coid);
    return;
}

int BridgeChip_pdb_control(char *pdb_path, unsigned char val)
{

    FILE *pdb_fp = NULL;
    int ret_val = 0;

    pdb_fp = fopen (pdb_path, "r+");
    if (pdb_fp == NULL) {
        LOG_CRITICAL_INFO(MODULE_NAME, "open %s fail", pdb_path);
        return -1;
    }
    else{
        LOG_CRITICAL_INFO(MODULE_NAME, "set %s to 0x%d", pdb_path, val);
        rewind(pdb_fp);
        ret_val = fprintf(pdb_fp, "%x", val);
        if(ret_val != sizeof(unsigned char)) {
            LOG_CRITICAL_INFO(MODULE_NAME, "write %s fail", pdb_path);
            fclose(pdb_fp);
            pdb_fp = NULL;
            return -1;
        }
        fflush(pdb_fp);
        fclose(pdb_fp);
        BridgeChip_OSAL_SleepMs(10);
    }

    return 0;
}

BridgeChip_StatusType display_power_ctl(int32 enable)
{
    char cmd_value[8];
    int fh;
    int ret;

    LOG_CRITICAL_INFO(MODULE_NAME, "Enter %s\n", __func__);
    memset (cmd_value, 0, sizeof(cmd_value));
    if (enable == DISPLAY_WKUP_CTRL_ON)
        strcpy(cmd_value, "on");
    else if (enable == DISPLAY_WKUP_CTRL_OFF)
        strcpy(cmd_value, "off");
    else {
        LOG_CRITICAL_INFO(MODULE_NAME, "unknow power mode setting");
        return BRIDGECHIP_STATUS_FAILED;
    }

    fh = displaywakeup_status_open(DISP_INFRAS_WKUP_CTRL_PATH);
    if (fh < 0) {
      LOG_CRITICAL_INFO(MODULE_NAME, "Could get access to status devnode: %s, error=%d",
                        DISP_INFRAS_WKUP_CTRL_PATH, fh);
      return BRIDGECHIP_STATUS_FAILED;
    }

    ret = displaywakeup_ctrl_set(fh, "online", DISP_INFRAS_WKUP_CTRL_ONLINE);
    if (ret) {
      LOG_CRITICAL_INFO(MODULE_NAME, "Couldn't switch to online");
      goto fail;
    }
    ret = displaywakeup_ctrl_set(fh, DISP_INFRAS_WKUP_CTRL_NAME, cmd_value);
    if (ret) {
      LOG_CRITICAL_INFO(MODULE_NAME,"Couldn't set chain <%s> state to %s", DISP_INFRAS_WKUP_CTRL_NAME, cmd_value);
      goto fail;
    }
    displaywakeup_status_close(fh);
    LOG_CRITICAL_INFO(MODULE_NAME, "Exit %s\n", __func__);
    return BRIDGECHIP_STATUS_SUCCESS;

fail:
    displaywakeup_status_close(fh);
    return BRIDGECHIP_STATUS_FAILED;
}

BridgeChip_StatusType disp_infras_display_chain_control(int32 i2c_fh, int32 state)
{
    //TODO: need check return value of each functions
    if (DISPLAY_CHAIN_CTRL_OFF == state) {
        LOG_CRITICAL_INFO(MODULE_NAME, "call %s OFF", __func__);
        BridgeChip_pdb_control(BRIDGECHIP_GPIO_PDB, DISPLAY_PDB_CTRL_OFF);
        (void) usleep(5*1000); //wait ser enter PDB after Disable PDB
    } else if (DISPLAY_CHAIN_CTRL_ON == state) {
        LOG_CRITICAL_INFO(MODULE_NAME, "call %s ON", __func__);
        BridgeChip_pdb_control(BRIDGECHIP_GPIO_PDB, DISPLAY_PDB_CTRL_ON);
        (void) usleep(10*1000); //wait ser ready after enable PDB
        ser_config_update(i2c_fh);
    }
    LOG_CRITICAL_INFO(MODULE_NAME, "Exit %s", __func__);
    return BRIDGECHIP_STATUS_SUCCESS;
}

BridgeChip_StatusType disp_infras_display_chain_recovery(int32 i2c_fh, int32 state, disp_notify_type_t notify_type)
{
    //TODO: need check return value of each functions
    if (DISPLAY_CHAIN_CTRL_OFF == state) {
        if (notify_type == DISP_NOTIFY_TYPE_FCC)
        {
            LOG_CRITICAL_INFO(MODULE_NAME, "call %s OFF for %s", __func__, notify_type_str[notify_type]);
            display_state_notify("lost",DISP_NOTIFY_TYPE_FCC);
        } 
        else 
        {
            LOG_CRITICAL_INFO(MODULE_NAME, "call %s OFF for %s", __func__, notify_type_str[notify_type]);
            display_state_notify("lost",DISP_NOTIFY_TYPE_IPD);
            display_state_notify("lost",DISP_NOTIFY_TYPE_FCC);
        }
    } else if (DISPLAY_CHAIN_CTRL_ON == state) {
        LOG_CRITICAL_INFO(MODULE_NAME, "call %s ON for %s", __func__, notify_type_str[notify_type]);
    }
    LOG_CRITICAL_INFO(MODULE_NAME, "Exit %s", __func__);

    return BRIDGECHIP_STATUS_SUCCESS;
}

#ifdef __cplusplus
}
#endif