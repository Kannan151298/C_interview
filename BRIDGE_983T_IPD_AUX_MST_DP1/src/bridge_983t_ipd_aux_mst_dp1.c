/*
 ===========================================================================

 FILE:         bridge_983t_ipd_aux_mst_dp1.c

 $File$

 ===========================================================================
 Copyright (C) GM Global Technology Operations LLC 2025.
 All Rights Reserved.
 GM Confidential Restricted.
 
 Copyright (c) 2018 Qualcomm Technologies, Inc.
 All Rights Reserved.
 Qualcomm Technologies Proprietary and Confidential.

 Copyright (c) 2021 Bosch Automotive Products (Suzhou) Co.,Ltd(RBAC).
 All Rights Reserved.
 RBAC Proprietary and Confidential.

 ===========================================================================
 */

#ifdef __cplusplus
extern "C"
{
#endif

#include "bc-com-res-mgr.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/iofunc.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <devctl.h>
#include <errno.h>
#include <semaphore.h>
#include <pthread.h>

#include "bridgechip_plugin.h"
#include "bridgechip_osal.h"
#include "bridgechip_logger.h"
#include "bridgechip_i.h"
#include "i2c_client.h"
#include "DALDeviceId.h"
#include "devcfg_BridgeChip.h"
#include "ds90ux98x_init.h"
#include "supply_handling.h"
#include "disp_infras_int.h"
#include "screen_pwrmgr_common.h"

#define CHIP_ID                                       "BRIDGE DP1MST IPD-AUX"
#define BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM         0xDEADC0DE
#define I2C_BUS_ID_MAX_LENGTH                         0x11

/* DIRTY BITS */
#define BRIDGECHIP_POWER_DIRTYBIT             0x00000001

#define DEFAULT_I2C_SLAVE_ADDR                0x11
#define DEFAULT_I2C_GSBI_ID                   "/dev/i2c6"

#define DEFAULT_SINK_ERR_INTR_SUPPORT         FALSE
#define DEFAULT_ERR_POLLING_TIME_IN_MS        (1000)   // 1 seconds
#define DEFAULT_ERR_POLLING_TIME_IN_MS_MIN     (500)
#define FAST_ERR_POLLING_TIME_IN_MS            (100)   // 0.1 seconds
#define I2C_BUS_ACCESS_INFO_FILE              "/tmp/no-display-i2c-access"

#define CLIENT_OPERATIONS_CNT                 150
#define API_DEV_PATH                          "/dev/i2c6_ipc"
#define MONITOR_THREAD_SLEEP_TIME_MS          300
#define PDPLINK_CHECK_COUNT                   5

static bc_com_i2c_st_t bc_com_i2c = {0};

typedef struct _Plugin_Data_t
{
    int32 hI2CFileDes; /* Handle to I2C RM  */
    /* Board configuration information */
    uint8 uI2CSlaveId;
    uint8 uDeserNewSlaveAddr;
    char aI2CBusId[I2C_BUS_ID_MAX_LENGTH];
    BridgeChip_ThreadStateType ePollingThreadState; /* Tracking state of the polling thread */
    BridgeChip_OSAL_ThreadID hPollingThreadID; /* Thread ID of the error polling thread */
    uint32 uErrorPollingTime; /* Error polling time */
    /* Other statuses */
    bool32 bIsRxDetected;
    bool32 bIsDaisyRxDetected;
    bool32 bIsDesVideoValid;
    bool32 bIsDesDaisyVideoValid;
    bool32 bIsDPRxReady;
    bool32 bSinkErrIntrSupported;
    uint32 power_count;
    bool32 first_poweron;
    uint32 open_counter;
    uint32 commit_cnt;
    pthread_cond_t cond_disp_recovery;
    pthread_mutex_t mutex_disp_recovery;
    pthread_cond_t cond_disp_recovery_done;
    pthread_mutex_t mutex_disp_recovery_done;
    BridgeChip_OSAL_ThreadID disp_recovery_thread_id;
    BridgeChip_ThreadStateType disp_stat_recovery_thread_state;
    bool32 bdisplay_state_ready;
    uint32 linkup_counter;
    uint32 linkdown_counter;
    uint32 dplink_check_count;
    uint32 vpcheck_counter;
    bool32 bIsVpEnabled;
} PluginData;

/*!
 *  \b BridgeChip_Plugin_UserCtxtInfo
 *
 *  Defines the structure containing user context information specific to the plugged in
 *  bridge chip driver.
 *
 */
typedef struct _BridgeChip_Plugin_UserCtxInfo_t
{
  uint32 uMagicNumCheck; /* Magic number for checking handle sanity */
  BridgeChip_Plugin_PhyCtxInfo *pPhysicalContext; /* Pointer to the physical context information */
  struct _BridgeChip_Plugin_UserCtxInfo_t *pNextUser;
  bool32 bPowerEnable; /* Flag indicating power status  */
  BridgeChip_Plugin_CbInfo sCallbackInfo; /* Client callback registration information */
  uint32 uDirtyBits;
  uint32 uFlags;

} BridgeChip_Plugin_UserCtxtInfo;

static BridgeChip_Plugin_PhyCtxInfo gsPhyCtx;
static PluginData gsPluginInfo;
//static int iI2cErrValue = -1;
sem_t mutex_sem;

/*------------------------------------------------------------------------------
 * Prototypes
 *----------------------------------------------------------------------------*/
static void bridge_983t_ipd_aux_mst_dp1_process_cb(BridgeChip_Plugin_PhyCtxInfo *pPhyInfo, uint32 uEventMask);
static void bridge_983t_ipd_aux_mst_dp1_add_usr(BridgeChip_Plugin_PhyCtxInfo *pPhyInfo,
                                                BridgeChip_Plugin_UserCtxtInfo *pUserInfo);
static void bridge_983t_ipd_aux_mst_dp1_remove_usr(BridgeChip_Plugin_PhyCtxInfo *pPhyInfo,
                                                   BridgeChip_Plugin_UserCtxtInfo *pUserInfo);

void disp_chain_recovery_thread(void *pArgs)
{
    BridgeChip_Plugin_PhyCtxInfo *pPhyInfo = NULL;
    //bool32 b_state = FALSE;
    pPhyInfo = (BridgeChip_Plugin_PhyCtxInfo *) pArgs;
    PluginData *pInfo = NULL;

    pInfo = (PluginData *) pPhyInfo->pCustomData;
    LOG_CRITICAL_INFO(CHIP_ID, "Enter %s", __func__);
    while (pInfo->disp_stat_recovery_thread_state == BRIDGECHIP_OSAL_THREAD_STATE_RUN) {
        pthread_mutex_lock( &pInfo->mutex_disp_recovery );
        pthread_cond_wait( &pInfo->cond_disp_recovery, &pInfo->mutex_disp_recovery);
        pthread_mutex_unlock( &pInfo->mutex_disp_recovery );
		
	bc_serve_clients(&bc_com_i2c, CLIENT_OPERATIONS_CNT,MONITOR_THREAD_SLEEP_TIME_MS);
		
        //Check the power state before do recovery
        if (TRUE == pPhyInfo->bPowerEnable) {
            if ((pInfo->bIsDaisyRxDetected == FALSE) && (pInfo->bIsRxDetected != FALSE))
            {
                LOG_CRITICAL_INFO(CHIP_ID, "=========>>> FCC handle for HPD : %s", __func__);
                disp_infras_display_chain_recovery(pInfo->hI2CFileDes, DISPLAY_CHAIN_CTRL_OFF,
                                                DISP_NOTIFY_TYPE_FCC);
                disp_infras_display_chain_recovery(pInfo->hI2CFileDes, DISPLAY_CHAIN_CTRL_ON,
                                                DISP_NOTIFY_TYPE_FCC);
            } else if ((pInfo->bIsDaisyRxDetected == FALSE) && (pInfo->bIsRxDetected == FALSE))
            {
                LOG_CRITICAL_INFO(CHIP_ID, "=========>>> FCC & IPD handle for HPD : %s", __func__);
                disp_infras_display_chain_recovery(pInfo->hI2CFileDes, DISPLAY_CHAIN_CTRL_OFF,
                                                DISP_NOTIFY_TYPE_IPD);
                disp_infras_display_chain_recovery(pInfo->hI2CFileDes, DISPLAY_CHAIN_CTRL_ON,
                                                DISP_NOTIFY_TYPE_IPD);
            }
  	}
	bc_serve_clients(&bc_com_i2c, CLIENT_OPERATIONS_CNT,MONITOR_THREAD_SLEEP_TIME_MS);

        pthread_cond_signal(&pInfo->cond_disp_recovery_done);
    };

    LOG_CRITICAL_INFO(CHIP_ID, "Exit %s", __func__);

}

static BridgeChip_StatusType disp_chain_recovery_init()
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    gsPluginInfo.cond_disp_recovery = (pthread_cond_t)PTHREAD_COND_INITIALIZER;
    gsPluginInfo.mutex_disp_recovery = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
    gsPluginInfo.cond_disp_recovery_done = (pthread_cond_t)PTHREAD_COND_INITIALIZER;
    gsPluginInfo.mutex_disp_recovery_done = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
    gsPluginInfo.bdisplay_state_ready = FALSE;

    if (pthread_cond_init(&gsPluginInfo.cond_disp_recovery, NULL) != EOK)
    {
        LOG_ERROR(CHIP_ID, "Failed to pthread_cond_init cond_disp_recovery");
    }
    if (pthread_cond_init(&gsPluginInfo.cond_disp_recovery_done, NULL) != EOK)
    {
        LOG_ERROR(CHIP_ID, "Failed to pthread_cond_init cond_disp_recovery_done");
    }

    gsPluginInfo.disp_stat_recovery_thread_state = BRIDGECHIP_OSAL_THREAD_STATE_RUN;
    eStatus = BridgeChip_OSAL_CreateThread(&gsPluginInfo.disp_recovery_thread_id,
            disp_chain_recovery_thread, (void *) &gsPhyCtx);
    if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
    {
        LOG_ERROR(CHIP_ID, "Failed to create Error SER INIT thread (%d)", eStatus);
    }
    return BRIDGECHIP_STATUS_SUCCESS;
}

static void disp_chain_recovery_deinit()
{
    BridgeChip_OSAL_DestroyThread(&gsPluginInfo.disp_recovery_thread_id);
    pthread_cond_destroy(&gsPluginInfo.cond_disp_recovery);
    pthread_mutex_destroy(&gsPluginInfo.mutex_disp_recovery);
    pthread_cond_destroy(&gsPluginInfo.cond_disp_recovery_done);
    pthread_mutex_destroy(&gsPluginInfo.mutex_disp_recovery_done);
}

static int sem_timedwait_ms(sem_t *sem, int ms)
{
    struct timespec tm;
    clock_gettime(CLOCK_MONOTONIC, &tm);
    tm.tv_sec += (ms/1000);
    tm.tv_nsec += (ms - (ms/1000)*1000)*1000*1000;
    if (tm.tv_nsec >= 1000000000) {
        tm.tv_sec ++;
        tm.tv_nsec -= 1000000000;
    }
    return sem_timedwait_monotonic( sem, &tm );
}

static void bridge_983t_ipd_aux_mst_dp1_add_usr(BridgeChip_Plugin_PhyCtxInfo *pPhyInfo,
                                                BridgeChip_Plugin_UserCtxtInfo *pUserInfo)
{
    pUserInfo->pNextUser = (BridgeChip_Plugin_UserCtxtInfo *) pPhyInfo->pClientInfo;
    pPhyInfo->pClientInfo = (void *) pUserInfo;
    LOG_INFO(CHIP_ID, "Add new client (%p)", pUserInfo);
}

static void bridge_983t_ipd_aux_mst_dp1_remove_usr(BridgeChip_Plugin_PhyCtxInfo *pPhyInfo,
                                                   BridgeChip_Plugin_UserCtxtInfo *pUserInfo)
{
    BridgeChip_Plugin_UserCtxtInfo *pTmp;
    BridgeChip_Plugin_UserCtxtInfo *pPrev;

    pTmp = pPrev = (BridgeChip_Plugin_UserCtxtInfo *) pPhyInfo->pClientInfo;

    if (pTmp == pUserInfo)
    {
        pPhyInfo->pClientInfo = pTmp->pNextUser;
        LOG_INFO(CHIP_ID, "Remove client (%p)", pUserInfo);
    }
    else
    {
        while (NULL != pTmp)
        {
            if (pTmp == pUserInfo)
            {
                pPrev->pNextUser = pUserInfo->pNextUser;
                LOG_INFO(CHIP_ID, "Remove client (%p)", pUserInfo);
                break;
            }
            else
            {
                pPrev = pTmp;
                pTmp = pTmp->pNextUser;
            }
        }
    }
}

static void bridge_983t_ipd_aux_mst_dp1_process_cb(BridgeChip_Plugin_PhyCtxInfo *pPhyInfo, uint32 uEventMask)
{
    PluginData *pInfo = (PluginData *) pPhyInfo->pCustomData;
    uint32 uI = 0;
    uint32 uEventId = 0;
    bool32 bUnlockMutex = FALSE;
    BridgeChip_Plugin_UserCtxtInfo *pTmp = NULL;
    BridgeChip_CallbackInfoType sCbInfo;

    pTmp = (BridgeChip_Plugin_UserCtxtInfo *) pPhyInfo->pClientInfo;

    while (NULL != pTmp)
    {
        if (BridgeChip_OSAL_LockMutex(pPhyInfo->sHwMutex) == BRIDGECHIP_STATUS_SUCCESS)
        {
            bUnlockMutex = TRUE;
        }
        LOG_INFO(CHIP_ID, "client = %p; uEventMask=0x%x", pTmp, pTmp->sCallbackInfo.sCbRegInfo.uEventMask);

        /* Proceed if a callback function and a valid event mask exist,
        * and a new event notification matches the user's event mask
        */
        if ((NULL != pTmp->sCallbackInfo.sCbRegInfo.pCallback)
                && (0x00 != pTmp->sCallbackInfo.sCbRegInfo.uEventMask)
                && (0x00 != (pTmp->sCallbackInfo.sCbRegInfo.uEventMask & uEventMask)))
        {
            BridgeChip_OSAL_Memset(&sCbInfo, 0x00, sizeof(BridgeChip_CallbackInfoType));

            sCbInfo.hHandle = pTmp->sCallbackInfo.hUserHandle;
            sCbInfo.pUserData = pTmp->sCallbackInfo.sCbRegInfo.pUserData;
            sCbInfo.uFlags = 0x00;

            for (uI = 1, uEventId = 0; uI < (BRIDGECHIP_EVENT_ALL + 1); uI = uI << 1, uEventId++)
            {
                /* Process events for which this client has registered */
                if ((0x00 != (pTmp->sCallbackInfo.sCbRegInfo.uEventMask & uI)) && (0x00 != (uEventMask & uI)))
                {
                    sCbInfo.sEventInfo.uEventMask |= uI;
                    switch (uI)
                    {
                        case BRIDGECHIP_EVENT_HPD:
                            if (pInfo->bIsRxDetected && pInfo->bIsDaisyRxDetected)
                                sCbInfo.sEventInfo.sHPDInfo.bPlugged = TRUE;
                            else {
                                LOG_CRITICAL_INFO (CHIP_ID, "sHPDInfo.bPlugged = FALSE");
                                sCbInfo.sEventInfo.sHPDInfo.bPlugged = FALSE;
                                pthread_cond_signal( &pInfo->cond_disp_recovery);
                                /* Wait on recovery completion */
                                pthread_mutex_lock(&pInfo->mutex_disp_recovery_done);
                                pthread_cond_wait(&pInfo->cond_disp_recovery_done,
                                  &pInfo->mutex_disp_recovery_done);
                                pthread_mutex_unlock( &pInfo->mutex_disp_recovery_done);
                            }
                            break;
                        case BRIDGECHIP_EVENT_PLL:
                        case BRIDGECHIP_EVENT_REMOTE_DEVICE_INTERRUPT:
                        case BRIDGECHIP_EVENT_HDCP:
                        case BRIDGECHIP_EVENT_DEVICE_RECOVERY:
                            break;
                        default:
                        {
                            // Should never reach here
                            LOG_WARNING(CHIP_ID, "Unsupported interrupt (%x) - should not reach here", uI);
                            continue;
                        }
                    }
                }
            }
            if (bUnlockMutex)
            {
                BridgeChip_OSAL_UnLockMutex(pPhyInfo->sHwMutex);
                bUnlockMutex = FALSE; /* reset flag */
            }

            if (0x00 != sCbInfo.sEventInfo.uEventMask)
            {
                LOG_INFO(CHIP_ID, "Invoking Callback: client = %p, uEventMask = 0x%x", pTmp, uEventMask);
                pTmp->sCallbackInfo.sCbRegInfo.pCallback(&sCbInfo);
                (void) usleep(50*1000);

                LOG_INFO(CHIP_ID, "Callback Done");
            }
        }
        if (bUnlockMutex)
        {
            BridgeChip_OSAL_UnLockMutex(pPhyInfo->sHwMutex);
            bUnlockMutex = FALSE; /* reset flag */
        }

        LOG_INFO(CHIP_ID, "client->next = %p", pTmp->pNextUser);
        pTmp = pTmp->pNextUser;
    }
}

void bridge_983t_ipd_aux_mst_dp1_err_poll(void *pArgs)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    BridgeChip_Plugin_PhyCtxInfo *pPhyInfo = NULL;
    PluginData *pInfo = NULL;
    uint32 uEventMask = 0;
    bool32 bUnlockMutex = FALSE;
    bool32 bExist;
    struct timespec tm;
    uint32 loop_flood_check = 0;
    struct timespec tm_old, tm_now;
    uint32 timediff = 0;
	uint8 dplink_stat = 0;

    pPhyInfo = (BridgeChip_Plugin_PhyCtxInfo *) pArgs;
    pInfo = (PluginData *) pPhyInfo->pCustomData;

    LOG_CRITICAL_INFO(CHIP_ID, "ENTER ErrorPolling polling");
    pInfo->bIsRxDetected = FALSE;
    pInfo->bIsDaisyRxDetected = FALSE;
    pInfo->bIsDesVideoValid = FALSE;
    pInfo->bIsDesDaisyVideoValid = FALSE;
    pInfo->bIsDPRxReady = FALSE;
    pInfo->bIsVpEnabled = FALSE;
    pInfo->linkup_counter = 0;
    pInfo->linkdown_counter = 0;
    pInfo->dplink_check_count = 0;
    pInfo->vpcheck_counter = 0;

    while (BRIDGECHIP_OSAL_THREAD_STATE_RUN == pInfo->ePollingThreadState)
    {
        eStatus = BridgeChip_OSAL_LockMutex(pPhyInfo->sHwMutex);
        if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
        {
            LOG_ERROR(CHIP_ID, "Failed to lock mutex (%d)", eStatus);
            goto fail;
        }

        bUnlockMutex = TRUE;

        if (TRUE == pPhyInfo->bPowerEnable)
        {
            /* Reset re-usable variables */
            uEventMask = 0x00;
            /*** Check status registers for errors ***/
            /* Only error scenarios are reported to client.
            * Otherwise, register statuses are used to update software statuses
            */
	    eStatus = serdes_get_ser_link_status(pInfo->hI2CFileDes, 1);
            if (eStatus == BRIDGECHIP_STATUS_SUCCESS) /* serdes_get_ser_link_status: linked */
            {
                    if (pInfo->bIsRxDetected == FALSE)
                    {
                        dplink_stat = serdes_check_983_dprx(pInfo->hI2CFileDes);
                        if (dplink_stat == BRIDGECHIP_STATUS_FAILED)
                        {
                            LOG_CRITICAL_INFO(CHIP_ID, "dp link is not ready. need wait video ready");
                            if (pInfo->dplink_check_count++ > PDPLINK_CHECK_COUNT)
                            {
                                pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
                                uEventMask |= BRIDGECHIP_EVENT_DEVICE_RECOVERY;
                                LOG_CRITICAL_INFO("[DpReTr]", "Wait 983 DP ready timeout, Try to trigger SoC DP Link Training !");
                                serdes_diagnostic_983dprx(pInfo->hI2CFileDes);
                            }
                            else
                            {
                                pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                            }
                            goto go_next;
                        }
                        else if(dplink_stat == BRIDGECHIP_EXIT_FROM_DPLINK_UNTRAIN)
                        {
                            LOG_CRITICAL_INFO(CHIP_ID, "dp link untrained with 0x30044");
                            uEventMask |= BRIDGECHIP_EVENT_DEVICE_RECOVERY;
                            LOG_CRITICAL_INFO("[DpReTr]", "Trying to trigger Recovery and SoC DP Link retraining!");
                            serdes_diagnostic_983dprx(pInfo->hI2CFileDes);
                        }
                        else
                        {
                            pInfo->bIsDPRxReady = TRUE;
                            if (pInfo->bIsVpEnabled == FALSE)
                            {
                                ser_config_vp(pInfo->hI2CFileDes);
                                pInfo->bIsVpEnabled = TRUE;
                                (void) usleep(100 * 1000);    // wait about 6 frames
                            }
                        }
                    	/*Check Video Processors status*/
                    	eStatus = serdes_check_vp_status(pInfo->hI2CFileDes);
                    	if (eStatus == BRIDGECHIP_STATUS_FAILED)
                    	{
                        	/*run the loop for 5 times and exit even if the VPs are out of sync*/
                       	 	if(pInfo->vpcheck_counter++ < 5)
                       		{
	                            	LOG_ERROR(CHIP_ID, "VP Status failed counter value (%d)", pInfo->vpcheck_counter);
	                            	serializer_video_input_reset(pInfo->hI2CFileDes);
	                            	pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
	                            	goto go_next;
                        	}
                        	else
                        	{
	                            	pInfo->vpcheck_counter = 0;
	                            	pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
	                            	LOG_ERROR(CHIP_ID, "VP Status failed counter value reached max (%d) continue DES init", pInfo->vpcheck_counter);
                        	}
                    	}
						else if (eStatus == BRIDGECHIP_STATUS_SUCCESS)
						{
							LOG_CRITICAL_INFO(CHIP_ID, "VPs synchronized....");
							pInfo->vpcheck_counter = 0;
							pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
						}
						else if (eStatus == BRIDGECHIP_STATUS_BAD_PARAMS)
						{
							LOG_CRITICAL_INFO(CHIP_ID, "Read VPs Status failed");
							pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
							goto go_next;
			}
                        (void) usleep(50 * 1000);
                        if(serdes_get_ser_link_status(gsPluginInfo.hI2CFileDes, 1) == BRIDGECHIP_STATUS_SUCCESS)
                            LOG_CRITICAL_INFO(CHIP_ID, "FPD-LINK VI linkup is okay");
                        else
                        {
                            LOG_CRITICAL_INFO(CHIP_ID, "FPD-LINK VI link is down !!!");
                            goto go_next;
                        }

                        if (pInfo->linkdown_counter == 0)
                            eStatus = dser_config_update(pInfo->hI2CFileDes); //first time boot or resume
                        else
                            eStatus = dser_config_update_recovery(pInfo->hI2CFileDes); //link lost des reinit
                        if (eStatus != BRIDGECHIP_STATUS_SUCCESS)
                        {
                            LOG_CRITICAL_INFO(CHIP_ID, "Configure de-serializer failed, go to try again!!!");
                            pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                            goto go_next;
                        }

                        LOG_CRITICAL_INFO(CHIP_ID, "Configure de-serializer successfully");
                        serdes_get_983_dp_rx_status(pInfo->hI2CFileDes);
                        pInfo->bIsRxDetected = TRUE;
                        pInfo->linkup_counter ++;
                        uEventMask |= BRIDGECHIP_EVENT_HPD;
                        pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;

                        //TODO: sync for valid video streaming which is important for enabling display
                        if (pInfo->bIsDesVideoValid == FALSE)
                        {
                            (void) usleep(100 * 1000); /*give 100msec delay before reading DTG*/
                            if (serdes_monitor_des_video(pInfo->hI2CFileDes, TRUE) == BRIDGECHIP_STATUS_SUCCESS)
                            {
                                pInfo->bIsDesVideoValid = TRUE;
                            }
                            else
                            {
                                pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                            }
                        }
                        BRIDGECHIP_INIT_SYNC_POINT();
                        display_state_notify("ready",DISP_NOTIFY_TYPE_IPD);

                        LOG_CRITICAL_INFO(CHIP_ID, "DISP INIT DONE");
                    }
                    else
                    {
                        if (pInfo->bIsDesVideoValid == FALSE)
                        {
                            if (serdes_monitor_des_video(pInfo->hI2CFileDes, TRUE) == BRIDGECHIP_STATUS_SUCCESS)
                            {
                                pInfo->bIsDesVideoValid = TRUE;
                                pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
                            }
                            else
                            {
                                pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                            }
                        }
                        else if((TRUE == pInfo->bIsDesVideoValid) && (TRUE == pInfo->bIsDaisyRxDetected))
                    {
                        if (serdes_monitor_des_video(pInfo->hI2CFileDes, FALSE) == BRIDGECHIP_STATUS_FAILED)
                        {
                            pInfo->bIsDesVideoValid = FALSE;
                        }
		    }
		}
                eStatus = serdes_get_daisy_link_status(pInfo->hI2CFileDes);
                if (eStatus == BRIDGECHIP_STATUS_SUCCESS) /* serdes_get_daisy_link_status: linked */
		{
                    if (pInfo->bIsDaisyRxDetected == FALSE)
                    {
                        (void) usleep(100 * 1000);
                        dser_daisy_config_update(pInfo->hI2CFileDes);
                        LOG_CRITICAL_INFO(CHIP_ID, "dser_daisy_config_update called");
                        pInfo->bIsDaisyRxDetected = TRUE;
                        LOG_CRITICAL_INFO(CHIP_ID, "DAISY CHAIN DISP INIT DONE");
                        display_state_notify("ready",DISP_NOTIFY_TYPE_FCC);
                        uEventMask |= BRIDGECHIP_EVENT_HPD;
                        pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
                        //TODO: sync for valid video streaming which is important for enabling display
                        if (pInfo->bIsDesDaisyVideoValid == FALSE)
                        {
                            (void) usleep(100 * 1000); /*give 100msec delay before reading DTG*/
                            if (serdes_monitor_des_video_daisy(pInfo->hI2CFileDes, TRUE) == BRIDGECHIP_STATUS_SUCCESS)
                            {
                                pInfo->bIsDesDaisyVideoValid = TRUE;
                            }
                            else
                            {
                                pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                            }
                        }

                    }
                    else
                    {
                        if (pInfo->bIsDesDaisyVideoValid == FALSE)
                        {
                            if (serdes_monitor_des_video_daisy(pInfo->hI2CFileDes, TRUE) == BRIDGECHIP_STATUS_SUCCESS)
                            {
                                pInfo->bIsDesDaisyVideoValid = TRUE;
                                pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
                            }
                            else
                            {
                                pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                            }
                        }
                        else
                        {
                            if (serdes_monitor_des_video_daisy(pInfo->hI2CFileDes, FALSE) == BRIDGECHIP_STATUS_FAILED)
                            {
                                pInfo->bIsDesDaisyVideoValid = FALSE;
                            }
                        }
                    }
		}
		else if (eStatus == BRIDGECHIP_STATUS_FAILED) /* serdes_get_daisy_link_status: link lost */
		{
                    if (pInfo->bIsDaisyRxDetected == TRUE)
                    {
                        LOG_CRITICAL_INFO(CHIP_ID, "DAISY CHAIN Link not detected or lost!");
                        uEventMask |= BRIDGECHIP_EVENT_HPD;
                        pInfo->linkdown_counter ++;
                    }
                    pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS_MIN;
                    pInfo->bIsDaisyRxDetected = FALSE;
                    pInfo->bIsDesDaisyVideoValid = FALSE;
                    pInfo->dplink_check_count = 0;
                    goto go_next;
                }
                else /* serdes_get_daisy_link_status: BRIDGECHIP_STATUS_BAD_PARAMS: i2c read error */
                {
                    LOG_CRITICAL_INFO(CHIP_ID, "serdes_get_daisy_link_status: i2c read error");
                }
            }
            else if (eStatus == BRIDGECHIP_STATUS_FAILED) /* serdes_get_ser_link_status: link lost */
            {
                /* Link lost */
                if (TRUE == pInfo->bIsRxDetected)
                {
                    uEventMask |= BRIDGECHIP_EVENT_HPD;
                    pInfo->linkdown_counter ++;
                    LOG_CRITICAL_INFO(CHIP_ID, "RX NOT detected or lost");
                }
                pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS_MIN;
                /* Link lost recovery in next round */
                pInfo->bIsRxDetected = FALSE;
                pInfo->bIsDesVideoValid = FALSE;
                pInfo->bIsDesDaisyVideoValid = FALSE;             
                pInfo->bIsDaisyRxDetected = FALSE;
                pInfo->dplink_check_count = 0;
            }
	    else /* serdes_get_ser_link_status: BRIDGECHIP_STATUS_BAD_PARAMS: i2c read error */
	    {
		    LOG_CRITICAL_INFO(CHIP_ID, "serdes_get_ser_link_status: i2c read error");
            }

            if (uEventMask != 0){
                LOG_CRITICAL_INFO(CHIP_ID, "uEventMask = 0x%x", uEventMask);
            }
          
            // Monitor the DP Link status
            if (BRIDGECHIP_STATUS_FAILED == serdes_monitor_983_dp_link(pInfo->hI2CFileDes))
            {
                if (pInfo->bIsDPRxReady == FALSE)
                {
                    if (pInfo->dplink_check_count++ > PDPLINK_CHECK_COUNT)
                    {
                        pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
                        uEventMask |= BRIDGECHIP_EVENT_DEVICE_RECOVERY;
                        LOG_CRITICAL_INFO("[DpReTr]", "Wait 983 DP ready timeout, Try to trigger SoC DP Link Training !");
                        serdes_diagnostic_983dprx(pInfo->hI2CFileDes);
                    }
                    else
                    {
                        pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                    }
                }
                else
                {
                    uEventMask |= BRIDGECHIP_EVENT_DEVICE_RECOVERY;
                    LOG_CRITICAL_INFO("[DpReTr]", "983 DP Link Lost, Try to recovery! uEventMask |= BRIDGECHIP_EVENT_DEVICE_RECOVERY");
                    serdes_diagnostic_983dprx(pInfo->hI2CFileDes);
                    pInfo->bIsDPRxReady = FALSE;
                    pInfo->dplink_check_count = 0;
                }
            }
            else
            {
                pInfo->dplink_check_count = 0;
                pInfo->bIsDPRxReady = TRUE;
            }
            if ((pInfo->bIsDPRxReady == TRUE) && (pInfo->bIsVpEnabled == TRUE))
            {
                // Monitor the video processors status
                eStatus = serdes_check_vp_status(pInfo->hI2CFileDes);
                if (eStatus == BRIDGECHIP_STATUS_FAILED)
                {
                    LOG_CRITICAL_INFO(CHIP_ID, " VPs lost sync with DPRX....");
                    serdes_diagnostic_983dprx(pInfo->hI2CFileDes);
                    /* FIXME: Reset 983 can be taken as a workarround to trigger the DP retraining.
                    To reproduce VP issue and hold the issue, this should be committed */
		    ser_reset_chip(pInfo->hI2CFileDes);
                    pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                    uEventMask |= BRIDGECHIP_EVENT_DEVICE_RECOVERY;
                }
		else if (eStatus == BRIDGECHIP_STATUS_BAD_PARAMS)
                {
                    LOG_CRITICAL_INFO(CHIP_ID, "Read VP Status failed");
                    pInfo->uErrorPollingTime = FAST_ERR_POLLING_TIME_IN_MS;
                }
		else
                {
                }
			}
	}
        else
        {
            pInfo->uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
            pInfo->bIsRxDetected = FALSE;
            pInfo->bIsDesVideoValid = FALSE;
            pInfo->bIsDesDaisyVideoValid = FALSE;
            pInfo->bIsDaisyRxDetected = FALSE;
            pInfo->bIsVpEnabled = FALSE;
            pInfo->bIsDPRxReady = FALSE;
        }

fail:
go_next:
        /* No need to allow I2C requests, while powered off or going to
        * SUSPEND
        */
        if (TRUE == pPhyInfo->bPowerEnable)
        {
            bc_serve_clients(&bc_com_i2c, CLIENT_OPERATIONS_CNT,
                                           MONITOR_THREAD_SLEEP_TIME_MS);
            if (uEventMask != 0)
                LOG_CRITICAL_INFO(CHIP_ID, "uEventMask = 0x%x", uEventMask);
            if (bUnlockMutex)
            {
                BridgeChip_OSAL_UnLockMutex(pPhyInfo->sHwMutex);
                bUnlockMutex = FALSE; /* reset flag */
            }
            // Provide callbacks
            bridge_983t_ipd_aux_mst_dp1_process_cb(pPhyInfo, uEventMask);
            uEventMask = 0x00;
            if (BridgeChip_OSAL_LockMutex(pPhyInfo->sHwMutex) == BRIDGECHIP_STATUS_SUCCESS)
            {
                bUnlockMutex = TRUE;
            }
            /* No need to allow I2C requests, while powered off or going to
            * SUSPEND
                        */
            if (TRUE == pPhyInfo->bPowerEnable)
            {
                bc_serve_clients(&bc_com_i2c, CLIENT_OPERATIONS_CNT,
                MONITOR_THREAD_SLEEP_TIME_MS);
            }
        }

        if (bUnlockMutex)
        {
            BridgeChip_OSAL_UnLockMutex(pPhyInfo->sHwMutex);
            bUnlockMutex = FALSE; /* reset flag */
        }

        if (BRIDGECHIP_OSAL_THREAD_STATE_RUN == pInfo->ePollingThreadState)
        {
            sem_timedwait_ms(&mutex_sem, pInfo->uErrorPollingTime);
            clock_gettime(CLOCK_MONOTONIC, &tm);
            LOG_TRACE(CHIP_ID, ">>> new loop, %d, %d s , %d ns", pInfo->uErrorPollingTime, tm.tv_sec, tm.tv_nsec);
        }
        eStatus = BridgeChip_OSAL_IsPathExist(I2C_BUS_ACCESS_INFO_FILE, &bExist, NULL);
        if (eStatus == BRIDGECHIP_STATUS_SUCCESS)
        {
            if (bExist && pInfo->bIsRxDetected)
                break;
        }

        /* Check if looping is too fast */
        clock_gettime(CLOCK_REALTIME, &tm_now);
        if (loop_flood_check == 0)
            tm_old = tm_now;

        if (loop_flood_check == 20) {
            /* if 20 loop time diff < 200ms which means there are no blocking and timeout for looping */
            timediff = (tm_now.tv_sec - tm_old.tv_sec)*1000;
            timediff += (tm_now.tv_nsec - tm_old.tv_nsec)/1000/1000;
            if (timediff < 200)
                LOG_CRITICAL_INFO(CHIP_ID, "!!!!!! looping flood !!!!!!");
            loop_flood_check = 0;
                continue;
        }
        loop_flood_check++;
        /* Check if looping is too fast */
    }
    /* Lock IPC to prevent clients from i2c usage */
    bc_serve_clients(&bc_com_i2c, 0, 0);
    LOG_CRITICAL_INFO(CHIP_ID, "%s will EXIT", __func__);
    pInfo->ePollingThreadState = BRIDGECHIP_OSAL_THREAD_STATE_EXITED;

    LOG_TRACE(CHIP_ID, "EXIT");
}

/****************************************************************************
 *
 ** FUNCTION: bridge_983t_ipd_aux_mst_dp1_get_capabilities()
 */
/*!
 * \brief
 *   The \b bridge_983t_ipd_aux_mst_dp1_get_capabilities function returns the capabilities
 *   of the bridge chip.
 *
 * \param [out] pCaps        - Pointer to the structure where caps will be stored.
 *
 * \retval BridgeChip_StatusType
 *
 ****************************************************************************/
BridgeChip_StatusType bridge_983t_ipd_aux_mst_dp1_get_capabilities(BridgeChip_CapabilitiesType *pCaps)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    LOG_TRACE(CHIP_ID, "ENTER");

    if ( NULL == pCaps)
    {
        LOG_ERROR(CHIP_ID, "Bad Params");
        eStatus = BRIDGECHIP_STATUS_BAD_PARAMS;
        goto fail;
    }

    BridgeChip_OSAL_Memset(pCaps, 0x00, sizeof(BridgeChip_CapabilitiesType));

    pCaps->eVideoInput[0] = BRIDGECHIP_VIDEO_INTERFACE_HDMI_TX;
    pCaps->eAudioInput[0] = BRIDGECHIP_AUDIO_INTERFACE_NONE;

    pCaps->eVideoOutput[0] = BRIDGECHIP_VIDEO_INTERFACE_FPDLINK_III;
    pCaps->eAudioOutput[0] = BRIDGECHIP_AUDIO_INTERFACE_NONE;

    pCaps->bRemoteDeviceComm[BRIDGECHIP_REMOTE_DEVICE_PROTOCOL_I2C] = FALSE;

    pCaps->uMaxPixelClock = 210000; // 210Mhz
    pCaps->uMinPixelClock = 50000;  //  50Mhz

    pCaps->bHDCPSupported = FALSE;
    pCaps->bHPDSupported = FALSE;
    pCaps->bTestPatternSupport = FALSE;
    pCaps->bVideoBytePackingModeSupport = FALSE;
    pCaps->bBackwardCompatibiltySupport = FALSE;

fail:
    LOG_TRACE(CHIP_ID, "EXIT");

    return eStatus;
}

/****************************************************************************
 *
 ** FUNCTION: bridge_983t_ipd_aux_mst_dp1_dev_open()
 */
/*!
 * \brief
 *   The \b bridge_983t_ipd_aux_mst_dp1_dev_open function initializes the plugged in bridge chip
 *   driver. Each new client should call this function again to request a unique handle.
 *   On successful execution, a unique handle will be returned that needs to be used with
 *   subsequent calls.
 *
 * \param [in]  hHandle      - Pointer to the handle which will be returned.
 * \param [in]  uFlags       - Reserved for future use
 *
 * \retval BridgeChip_StatusType
 *
 ****************************************************************************/
BridgeChip_StatusType bridge_983t_ipd_aux_mst_dp1_dev_open(BridgeChip_HandleType *phHandle, uint32 uFlags)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    BridgeChip_Plugin_UserCtxtInfo *pUserInfo = NULL;
    bc_com_i2c.i2c_dev_name = DEFAULT_I2C_GSBI_ID;

    LOG_TRACE(CHIP_ID, "ENTER");

    if (NULL == phHandle)
    {
        LOG_ERROR(CHIP_ID, "Bad Params");
        eStatus = BRIDGECHIP_STATUS_BAD_PARAMS;
        goto fail;
    }

    if (BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM != gsPhyCtx.uMagicNumCheck)
    {
        BridgeChip_OSAL_Memset(&gsPhyCtx, 0x00, sizeof(gsPhyCtx));

        eStatus = BridgeChip_OSAL_CreateMutex(&gsPhyCtx.sHwMutex, TRUE);
        if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
        {
            LOG_ERROR(CHIP_ID, "Failed to create mutex (%d)", eStatus);
            goto phyfail;
        }

        if (BRIDGECHIP_STATUS_SUCCESS == (eStatus = BridgeChip_OSAL_LockMutex(gsPhyCtx.sHwMutex)))
        {
            if (gsPluginInfo.open_counter == 0)
            {
                eStatus = bc_com_init(&bc_com_i2c, CHIP_ID, API_DEV_PATH);

                if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
                {
                    log_err("Can't init resource manager\n");
                    /* No failure here, as it's not primaty pluggin's purpose */
                }
                /* Default configuration */
                gsPluginInfo.uI2CSlaveId = DEFAULT_I2C_SLAVE_ADDR;
                gsPluginInfo.uDeserNewSlaveAddr = 0x00;
                BridgeChip_OSAL_StrCpy(gsPluginInfo.aI2CBusId, DEFAULT_I2C_GSBI_ID,
                                             I2C_BUS_ID_MAX_LENGTH);
                gsPluginInfo.uErrorPollingTime = DEFAULT_ERR_POLLING_TIME_IN_MS;
                gsPluginInfo.bSinkErrIntrSupported = DEFAULT_SINK_ERR_INTR_SUPPORT;

                /* Open a fd for i2c client and set slave address */
                gsPluginInfo.hI2CFileDes = i2c_open(gsPluginInfo.aI2CBusId);
                if (-1 == gsPluginInfo.hI2CFileDes)
                {
                    LOG_ERROR(CHIP_ID, "Failed to open an i2c client handle");
                    eStatus = BRIDGECHIP_STATUS_NO_RESOURCES;
                    goto phyfail;
                }
                i2c_set_slave_addr(gsPluginInfo.hI2CFileDes, gsPluginInfo.uI2CSlaveId, I2C_ADDRFMT_7BIT);
                uint32_t speed = 0;
                int i2c_rc = 0;

                i2c_rc = i2c_set_bus_speed(gsPluginInfo.hI2CFileDes, I2C_SPEED_FAST, &speed);
                if (-1 == i2c_rc)
                {
                    LOG_ERROR(CHIP_ID, "%s ::Failed with Err=[%d], Speed=[%d]! <i2c_set_bus_speed!>", __func__, i2c_rc,
                                                speed);
                }

                gsPhyCtx.pCustomData = (void *) &gsPluginInfo;
                gsPhyCtx.uMagicNumCheck = BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM;

                disp_chain_recovery_init();
                //eStatus = ser_config_update(gsPluginInfo.hI2CFileDes);
                eStatus = disp_infras_display_chain_control(gsPluginInfo.hI2CFileDes, DISPLAY_CHAIN_CTRL_ON);
                if (eStatus != BRIDGECHIP_STATUS_SUCCESS)
                      LOG_ERROR(CHIP_ID, "Serializer reg update failed");

                if (FALSE == gsPluginInfo.bSinkErrIntrSupported)
                {
                    // Create an error status polling thread if de-serializer doesn't support HDCP
                    gsPluginInfo.ePollingThreadState = BRIDGECHIP_OSAL_THREAD_STATE_RUN;
                    gsPluginInfo.first_poweron = TRUE;
                    if (sem_init(&mutex_sem, 0, 0) < 0) {
                        LOG_ERROR(CHIP_ID, "sem_init fail, bridgechip startup fail");
                        eStatus = BRIDGECHIP_STATUS_FAILED;
                        goto phyfail;
                    }
                    eStatus = BridgeChip_OSAL_CreateThread(&gsPluginInfo.hPollingThreadID,
                                      bridge_983t_ipd_aux_mst_dp1_err_poll, (void *) &gsPhyCtx);
                    if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
                    {
                        gsPluginInfo.ePollingThreadState = BRIDGECHIP_OSAL_THREAD_STATE_NONE;
                        LOG_ERROR(CHIP_ID, "Failed to create Error Polling thread (%d)", eStatus);
                    }
                }

                LOG_INFO(CHIP_ID, "i2c_bus = %s, i2c_slave_id = 0x%02x", gsPluginInfo.aI2CBusId,
                                    gsPluginInfo.uI2CSlaveId);
            }
        }
        else
        {
            LOG_CRITICAL_INFO(CHIP_ID, "Only support open once: gsPluginInfo.open_counter :%d", gsPluginInfo.open_counter);
            eStatus = BRIDGECHIP_STATUS_SUCCESS;
        }
        (void) BridgeChip_OSAL_UnLockMutex(gsPhyCtx.sHwMutex);
    }

    if (BRIDGECHIP_STATUS_SUCCESS == (eStatus = BridgeChip_OSAL_LockMutex(gsPhyCtx.sHwMutex)))
    {
        pUserInfo = (BridgeChip_Plugin_UserCtxtInfo *) BridgeChip_OSAL_Malloc(
                        sizeof(BridgeChip_Plugin_UserCtxtInfo), 0x00);

        if (NULL == pUserInfo)
        {
            LOG_ERROR(CHIP_ID, "Malloc failed");
            eStatus = BRIDGECHIP_STATUS_NO_RESOURCES;
            (void) BridgeChip_OSAL_UnLockMutex(gsPhyCtx.sHwMutex);
            goto fail;
        }

        BridgeChip_OSAL_Memset(pUserInfo, 0x00, sizeof(BridgeChip_Plugin_UserCtxtInfo));
        pUserInfo->uMagicNumCheck = BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM;
        pUserInfo->pPhysicalContext = &gsPhyCtx;

        bridge_983t_ipd_aux_mst_dp1_add_usr(&gsPhyCtx, pUserInfo);

        *phHandle = (BridgeChip_HandleType) pUserInfo;
        gsPluginInfo.open_counter ++;
        (void) BridgeChip_OSAL_UnLockMutex(gsPhyCtx.sHwMutex);
    }
    else
    {
        LOG_ERROR(CHIP_ID, "Failed to lock mutex (%d)", eStatus);
        goto fail;
    }

    LOG_INFO(CHIP_ID, "Successful");

phyfail:
    if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
    {
        if (-1 != gsPluginInfo.hI2CFileDes)
        {
            i2c_close(gsPluginInfo.hI2CFileDes);
        }
        gsPluginInfo.hI2CFileDes = 0;

        if (gsPhyCtx.sHwMutex)
        {
            (void) BridgeChip_OSAL_UnLockMutex(gsPhyCtx.sHwMutex);
            BridgeChip_OSAL_DestroyMutex(gsPhyCtx.sHwMutex);
        }

        BridgeChip_OSAL_Memset(&gsPhyCtx, 0x00, sizeof(gsPhyCtx));
    }

fail:
    LOG_TRACE(CHIP_ID, "EXIT");

    return eStatus;
}

/****************************************************************************
 *
 ** FUNCTION: bridge_983t_ipd_aux_mst_dp1_dev_close()
 */
/*!
 * \brief
 *   The \b bridge_983t_ipd_aux_mst_dp1_dev_close function cloese the context for in bridge chip
 *   driver. Once all the open clients call this function, all resources acquired
 *   as part of Open will be freed.
 *
 * \param [in]  hHandle      - Pointer to the handle.
 *
 * \retval BridgeChip_StatusType
 *
 ****************************************************************************/
BridgeChip_StatusType bridge_983t_ipd_aux_mst_dp1_dev_close(BridgeChip_HandleType hHandle)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    BridgeChip_Plugin_UserCtxtInfo *pUserInfo = NULL;

    LOG_TRACE(CHIP_ID, "ENTER");

    pUserInfo = (BridgeChip_Plugin_UserCtxtInfo *) hHandle;

    if ((NULL == pUserInfo) || (BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM != pUserInfo->uMagicNumCheck))
    {
        LOG_ERROR(CHIP_ID, "Bad Params");
        eStatus = BRIDGECHIP_STATUS_BAD_PARAMS;
        goto fail;
    }

    if (BRIDGECHIP_STATUS_SUCCESS
            == (eStatus = BridgeChip_OSAL_LockMutex(pUserInfo->pPhysicalContext->sHwMutex)))
    {
        bridge_983t_ipd_aux_mst_dp1_remove_usr(pUserInfo->pPhysicalContext, pUserInfo);

        (void) BridgeChip_OSAL_UnLockMutex(pUserInfo->pPhysicalContext->sHwMutex);

        BridgeChip_OSAL_Free(pUserInfo);

        if (NULL == gsPhyCtx.pClientInfo)
        {
            LOG_INFO(CHIP_ID, "Last Client: Destroying physical context");
            if (gsPhyCtx.sHwMutex)
            {
                BridgeChip_OSAL_DestroyMutex(gsPhyCtx.sHwMutex);
            }

            if (gsPluginInfo.hPollingThreadID)
            {
                BridgeChip_OSAL_DestroyThread(&gsPluginInfo.hPollingThreadID);
            }

            if (-1 != gsPluginInfo.hI2CFileDes)
            {
                i2c_close(gsPluginInfo.hI2CFileDes);
            }
            gsPluginInfo.hI2CFileDes = 0;
            disp_chain_recovery_deinit();
            BridgeChip_OSAL_Memset(&gsPhyCtx, 0x00, sizeof(gsPhyCtx));

            /* Stop bridgechip resource manager */
            bc_com_deinit();
        }
    }
    else
    {
        LOG_ERROR(CHIP_ID, "Failed to lock mutex (%d)", eStatus);
        goto fail;
    }

fail:
    LOG_TRACE(CHIP_ID, "EXIT");

    return eStatus;
}

/****************************************************************************
 *
 ** FUNCTION: bridge_983t_ipd_aux_mst_dp1_dev_get_prop()
 */
/*!
 * \brief
 *   The \b bridge_983t_ipd_aux_mst_dp1_dev_get_prop function retrieves the properties of the bridge chip
 *   associated with the input handle. The properties returned by this function reflect
 *   the current device status. It does not return the un-committed properties set by client.
 *
 * \param [in]  hHandle        - Pointer to the handle.
 * \param [in]  eProperty      - Property type to retrieve.
 * \param [out] pPropertyData  - Pointer to the structure to store property data.
 * \param [in]  uFlags         - Reserved for future use.
 *
 * \retval BridgeChip_StatusType
 *
 ****************************************************************************/
BridgeChip_StatusType bridge_983t_ipd_aux_mst_dp1_dev_get_prop(BridgeChip_HandleType hHandle,
               BridgeChip_PropertyType eProperty, BridgeChip_PropertyDataType *pPropertyData, uint32 uFlags)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    BridgeChip_Plugin_UserCtxtInfo *pUserInfo = NULL;

    LOG_TRACE(CHIP_ID, "ENTER");

    pUserInfo = (BridgeChip_Plugin_UserCtxtInfo *) hHandle;

    if ((NULL == pUserInfo) || (BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM != pUserInfo->uMagicNumCheck)
        || (BRIDGECHIP_PROPERTY_NONE == eProperty) || (BRIDGECHIP_PROPERTY_MAX <= eProperty)
        || (NULL == pPropertyData))
    {
        LOG_ERROR(CHIP_ID, "Bad Params hHandle = %p, property (%d), pPropData = %p", hHandle, eProperty,
                pPropertyData);
        eStatus = BRIDGECHIP_STATUS_BAD_PARAMS;
        goto fail;
    }

    eStatus = BridgeChip_OSAL_LockMutex(pUserInfo->pPhysicalContext->sHwMutex);
    if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
    {
        LOG_ERROR(CHIP_ID, "Failed to lock mutex (%d)", eStatus);
        goto fail;
    }

    LOG_INFO(CHIP_ID, "client = %p, eProperty = %d", pUserInfo, eProperty);
    switch (eProperty)
    {
        case BRIDGECHIP_PROPERTY_POWER:
        {
            pPropertyData->bPowerOn = pUserInfo->pPhysicalContext->bPowerEnable;
            break;
        }
        default:
            LOG_ERROR(CHIP_ID, "Property(%d) not supported", eProperty);
            eStatus = BRIDGECHIP_STATUS_NOT_SUPPORTED;
            break;
    }

    BridgeChip_OSAL_UnLockMutex(pUserInfo->pPhysicalContext->sHwMutex);

fail:
    LOG_TRACE(CHIP_ID, "EXIT");

    return eStatus;
}

/****************************************************************************
 *
 ** FUNCTION: bridge_983t_ipd_aux_mst_dp1_dev_set_prop()
 */
/*!
 * \brief
 *   The \b bridge_983t_ipd_aux_mst_dp1_dev_set_prop function modifies the properties of the bridge chip
 *   associated with the input handle.
 *   All bridge chips do not support all the properties. Client should check the
 *   bridge chip capabilities before configuring the properties.
 *   Each client has its own context to set properties, and each client only commits
 *   property updates by itself.
 *   Validation is performed on BridgeChip_Device_Commit(). If commit fails, the latest
 *   successfully committed properties will be in effect.
 *   In case of multiple clients to the same bridge chip, the actual hardware configuration
 *   depends on the requests from other clients as well. In case of conflicting properties,
 *   the latest API call will be in effect.
 *
 * \param [in]  hHandle       - Pointer to the handle.
 * \param [in]  eProperty     - Property type to modify.
 * \param [in]  pPropertyData - Pointer to the structure containing new property data.
 * \param [in]  uFlags        - Reserved for future use.
 *
 * \retval BridgeChip_StatusType
 *
 ****************************************************************************/
BridgeChip_StatusType bridge_983t_ipd_aux_mst_dp1_dev_set_prop(BridgeChip_HandleType hHandle,
            BridgeChip_PropertyType eProperty, BridgeChip_PropertyDataType *pPropertyData, uint32 uFlags)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    BridgeChip_Plugin_UserCtxtInfo *pUserInfo = NULL;

    LOG_TRACE(CHIP_ID, "ENTER");

    pUserInfo = (BridgeChip_Plugin_UserCtxtInfo *) hHandle;

    if ((NULL == pUserInfo) || (BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM != pUserInfo->uMagicNumCheck)
        || (BRIDGECHIP_PROPERTY_NONE == eProperty) || (BRIDGECHIP_PROPERTY_MAX <= eProperty)
        || (NULL == pPropertyData))
    {
        LOG_ERROR(CHIP_ID, "Bad Params hHandle = %p, property (%d), pPropData = %p", hHandle, eProperty,
                pPropertyData);
        eStatus = BRIDGECHIP_STATUS_BAD_PARAMS;
        goto fail;
    }

    eStatus = BridgeChip_OSAL_LockMutex(pUserInfo->pPhysicalContext->sHwMutex);
    if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
    {
        LOG_ERROR(CHIP_ID, "Failed to lock mutex (%d)", eStatus);
        goto fail;
    }

    LOG_INFO(CHIP_ID, "client = %p, eProperty = %d, uFlags=0x%x", pUserInfo, eProperty, uFlags);

    switch (eProperty)
    {
        case BRIDGECHIP_PROPERTY_POWER:
        {
            LOG_CRITICAL_INFO(CHIP_ID, "##### BRIDGECHIP_PROPERTY_POWER : %d", pPropertyData->bPowerOn);
            pUserInfo->bPowerEnable = pPropertyData->bPowerOn;
            pUserInfo->uDirtyBits |= BRIDGECHIP_POWER_DIRTYBIT;
            pUserInfo->uFlags |= uFlags;
            break;
        }
        default:
            LOG_ERROR(CHIP_ID, "Property(%d) not supported", eProperty);
            eStatus = BRIDGECHIP_STATUS_NOT_SUPPORTED;
            break;
    }

    BridgeChip_OSAL_UnLockMutex(pUserInfo->pPhysicalContext->sHwMutex);

fail:
    LOG_TRACE(CHIP_ID, "EXIT");

    return eStatus;
}

/****************************************************************************
 *
 ** FUNCTION: bridge_983t_ipd_aux_mst_dp1_dev_commit()
 */
/*!
 * \brief
 *   The \b bridge_983t_ipd_aux_mst_dp1_dev_commit function commits all the changes done to a device
 *   to hardware. This function is a synchronous call and will return after all the
 *   hardware changes are done.
 *
 * \param [in]  hHandle        - Pointer to the handle.
 * \param [in]  uFlags         - Reserved for future use.
 *
 * \retval BridgeChip_StatusType
 *
 ****************************************************************************/
BridgeChip_StatusType bridge_983t_ipd_aux_mst_dp1_dev_commit(BridgeChip_HandleType hHandle, uint32 uFlags)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    BridgeChip_Plugin_UserCtxtInfo *pUserInfo = NULL;
    bool32 bUnlockMutex = FALSE;

    LOG_TRACE(CHIP_ID, "ENTER");

    pUserInfo = (BridgeChip_Plugin_UserCtxtInfo *) hHandle;

    if ((NULL == pUserInfo) || (BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM!= pUserInfo->uMagicNumCheck))
    {
        LOG_ERROR(CHIP_ID, "Bad Params hHandle = %p", hHandle);
        eStatus = BRIDGECHIP_STATUS_BAD_PARAMS;
        goto fail;
    }

    eStatus = BridgeChip_OSAL_LockMutex(pUserInfo->pPhysicalContext->sHwMutex);
    if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
    {
        LOG_ERROR(CHIP_ID, "Failed to lock mutex (%d)", eStatus);
        goto fail;
    }

    bUnlockMutex = TRUE;

    LOG_CRITICAL_INFO(CHIP_ID, "Client = %p, Dirtybits = 0x%x, uFlags= 0x%x", pUserInfo, pUserInfo->uDirtyBits, uFlags);
    if (BRIDGECHIP_POWER_DIRTYBIT & pUserInfo->uDirtyBits)
    {
        if (pUserInfo->bPowerEnable == TRUE)
        {
            if (uFlags == BRIDGECHIP_RECOVERY_PRE_POWER_ON)
            {
                pUserInfo->pPhysicalContext->bPowerEnable = pUserInfo->bPowerEnable;
                gsPluginInfo.bIsRxDetected = FALSE;
                gsPluginInfo.bIsDesVideoValid = FALSE;
                gsPluginInfo.linkdown_counter = 0;
                gsPluginInfo.dplink_check_count = 0;
                gsPluginInfo.bIsDPRxReady = FALSE;
                gsPluginInfo.bIsVpEnabled = FALSE;
                if (gsPluginInfo.commit_cnt == 0)
                {
                    LOG_CRITICAL_INFO(CHIP_ID, "Client4 = %p, Dirtybits = 0x%x, uFlags= 0x%x", pUserInfo, pUserInfo->uDirtyBits, uFlags);
                    LOG_CRITICAL_INFO(CHIP_ID, "Device powered ON %d", gsPluginInfo.power_count);
                    if (gsPluginInfo.first_poweron != TRUE)
                    {
                        LOG_CRITICAL_INFO(CHIP_ID, "Device powered on after off");
                        disp_infras_display_chain_control(gsPluginInfo.hI2CFileDes, DISPLAY_CHAIN_CTRL_ON);
                    }
                    else
                    {
                        gsPluginInfo.first_poweron = FALSE;
                    }
                    gsPluginInfo.power_count ++;
                }
                gsPluginInfo.commit_cnt ++;
                sem_post(&mutex_sem);
            }
            else
            {
                LOG_CRITICAL_INFO(CHIP_ID, "Unknow Power On Command");
            }
        }
        else
        {
            if (uFlags == BRIDGECHIP_RECOVERY_PRE_POWER_OFF)
            {
                pUserInfo->pPhysicalContext->bPowerEnable = pUserInfo->bPowerEnable;
                LOG_CRITICAL_INFO(CHIP_ID, "Video Pre Power off, Stop Polling Status");
            }
            else if (uFlags == BRIDGECHIP_RECOVERY_POST_POWER_OFF)
            {
                gsPluginInfo.bIsRxDetected = FALSE;
                gsPluginInfo.bIsDaisyRxDetected = FALSE;
                gsPluginInfo.linkup_counter = 0;
                gsPluginInfo.linkdown_counter = 0;
                gsPluginInfo.dplink_check_count = 0;
                gsPluginInfo.bIsDesVideoValid = FALSE;
                gsPluginInfo.bIsDPRxReady = FALSE;
                gsPluginInfo.bIsVpEnabled = FALSE;
                gsPluginInfo.commit_cnt --;
                if (gsPluginInfo.commit_cnt == 0)
                {
                    LOG_CRITICAL_INFO(CHIP_ID, "Video Post Power Off, Power off the Bridge Chip");
                    disp_infras_display_chain_control(gsPluginInfo.hI2CFileDes, DISPLAY_CHAIN_CTRL_OFF);
                }
                pUserInfo->pPhysicalContext->bPowerEnable = pUserInfo->bPowerEnable;
            }
            else
            {
                LOG_CRITICAL_INFO(CHIP_ID, "Unknow Power Off Command");
            }
        }
    }
    else
    {

    }

fail:
    if (bUnlockMutex)
    {
        pUserInfo->uDirtyBits = 0;
        BridgeChip_OSAL_UnLockMutex(pUserInfo->pPhysicalContext->sHwMutex);
    }

    LOG_TRACE(CHIP_ID, "EXIT");

    return eStatus;
}

/****************************************************************************
 *
 ** FUNCTION: bridge_983t_ipd_aux_mst_dp1_dev_reg_cb()
 */
/*!
 * \brief
 *   The \b bridge_983t_ipd_aux_mst_dp1_dev_reg_cb lets clients to register a callback for
 *   events associated with a bridge chip. To de-register the callback, client needs to
 *   use a NULL callback function.
 *   In case of interrupts, the interrupt will be masked until callback is processed.
 *   Interrupt will be unmasked as soon as the callback function is returned.
 *   Each client can register a single callback function for all event notifications.
 *
 * \param [in]  hHandle        - Pointer to the handle.
 * \param [in]  pCbRegInfo     - Pointer to the structure containing callback register information.
 * \retval BridgeChip_StatusType
 *
 ****************************************************************************/
BridgeChip_StatusType bridge_983t_ipd_aux_mst_dp1_dev_reg_cb(BridgeChip_HandleType hHandle,
                           BridgeChip_Plugin_CbInfo *pCbRegInfo)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    BridgeChip_Plugin_UserCtxtInfo *pUserInfo = NULL;
    bool32 bUnlockMutex = FALSE;

    LOG_TRACE(CHIP_ID, "ENTER");

    pUserInfo = (BridgeChip_Plugin_UserCtxtInfo *) hHandle;

    if ((NULL == pUserInfo) || (BRIDGE_983T_IPD_AUX_MST_DP1_MAGIC_NUM != pUserInfo->uMagicNumCheck)
                || (NULL == pCbRegInfo))
    {
        LOG_ERROR(CHIP_ID, "Bad Params hHandle = %p, pCbRegInfo = %p", hHandle, pCbRegInfo);
        eStatus = BRIDGECHIP_STATUS_BAD_PARAMS;
        goto fail;
    }

    eStatus = BridgeChip_OSAL_LockMutex(pUserInfo->pPhysicalContext->sHwMutex);
    if (BRIDGECHIP_STATUS_SUCCESS != eStatus)
    {
        LOG_ERROR(CHIP_ID, "Failed to lock mutex (%d)", eStatus);
        goto fail;
    }

    bUnlockMutex = TRUE;

    LOG_INFO(CHIP_ID, "Client = %p, uEventMask = 0x%x", pUserInfo, pCbRegInfo->sCbRegInfo.uEventMask);

    pUserInfo->sCallbackInfo = *pCbRegInfo;

fail:
    if (bUnlockMutex)
    {
        BridgeChip_OSAL_UnLockMutex(pUserInfo->pPhysicalContext->sHwMutex);
    }

    LOG_TRACE(CHIP_ID, "EXIT");

    return eStatus;
}

BridgeChip_StatusType bridge_983t_ipd_aux_mst_dp1_Install(BridgeChip_PluginFunctionTableType *pFxnTable)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    (void) BridgeChip_Logger_Init();

    LOG_TRACE(CHIP_ID, "ENTER");

    if (pFxnTable)
    {
        BridgeChip_OSAL_Memset(pFxnTable, 0x00, sizeof(BridgeChip_PluginFunctionTableType));
        BridgeChip_OSAL_Memset(&gsPluginInfo, 0x00, sizeof(gsPluginInfo));

        pFxnTable->pBridgeChip_Plugin_GetCapabilities = bridge_983t_ipd_aux_mst_dp1_get_capabilities;
        pFxnTable->pBridgeChip_Plugin_Open = bridge_983t_ipd_aux_mst_dp1_dev_open;
        pFxnTable->pBridgeChip_Plugin_Close = bridge_983t_ipd_aux_mst_dp1_dev_close;
        pFxnTable->pBridgeChip_Plugin_SetProperty = bridge_983t_ipd_aux_mst_dp1_dev_set_prop;
        pFxnTable->pBridgeChip_Plugin_GetProperty = bridge_983t_ipd_aux_mst_dp1_dev_get_prop;
        pFxnTable->pBridgeChip_Plugin_Commit = bridge_983t_ipd_aux_mst_dp1_dev_commit;
        pFxnTable->pBridgeChip_Plugin_RegisterEventCallback = bridge_983t_ipd_aux_mst_dp1_dev_reg_cb;
    }
    else
    {
        LOG_ERROR(CHIP_ID, "Bad Params");
        eStatus = BRIDGECHIP_STATUS_BAD_PARAMS;
    }
    LOG_TRACE(CHIP_ID, "EXIT");

    return eStatus;
}

#ifdef __cplusplus
}
#endif
