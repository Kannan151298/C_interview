/*
===========================================================================

FILE:         ds90ux98x_init.c

$File$

===========================================================================
Copyright (C) GM Global Technology Operations LLC 2025.
All Rights Reserved.
GM Confidential Restricted.

===========================================================================
*/

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <amss/i2c_client.h>
#include "bridgechip_plugin.h"
#include "bridgechip_osal.h"
#include "bridgechip_logger.h"
#include "ds90ux98x_init.h"

#define SER_ID_7BIT         0x11
#define DES_ID_7BIT         0x2C
#define MCU_ID_7BIT         0x12
#define DES_ID_7BIT_ALIAS   0x3D
#define MCU_ID_7BIT_ALIAS   0x15

#define SER_ID_8BIT         (SER_ID_7BIT << 1)
#define DES_ID_8BIT         (DES_ID_7BIT << 1)
#define MCU_ID_8BIT         (MCU_ID_7BIT << 1)
#define DES_ID_8BIT_ALIAS   (DES_ID_7BIT_ALIAS<<1)
#define MCU_ID_8BIT_ALIAS   (MCU_ID_7BIT_ALIAS<<1)

#define DES_DC_ID_7BIT          0x38
#define MCU_DC_ID_7BIT          0x13
#define DES_DC_ID_7BIT_ALIAS    0x31
#define MCU_DC_ID_7BIT_ALIAS    0x13

#define DES_DC_ID_8BIT          (DES_DC_ID_7BIT << 1)
#define MCU_DC_ID_8BIT          (MCU_DC_ID_7BIT << 1)
#define DES_DC_ID_8BIT_ALIAS    (DES_DC_ID_7BIT_ALIAS << 1)
#define MCU_DC_ID_8BIT_ALIAS    (MCU_DC_ID_7BIT_ALIAS << 1)

#define TOUCH_DC_ID_7BIT        0x48
#define TOUCH_DC_ID_7BIT_ALIAS  0x72
#define TOUCH_DC_ID_8BIT        (TOUCH_DC_ID_7BIT << 1)
#define TOUCH_DC_ID_8BIT_ALIAS  (TOUCH_DC_ID_7BIT_ALIAS << 1)

#define WAIT_INTERVAL  10

// Panel Timings
#define THW               4008     // Total horizontal
#define TVW               1132     // Total vertical
#define AHW               3556     // Active horizontal
#define AVW               1072     // Active vertical
#define HBP               160      // Horizontal back porch
#define VBP               4        // Vertical back porch
#define HSW               72       // Horizontal sync width
#define VSW               2        // Vertical sync width
#define HFP               220      // Horizantal front porch
#define VFP               54       // Vertical front porch
#define BPP               30       // Bits per pixel
#define PCLK              272.64

// AUX Panel Timings
#define DC_THW            1536     // Total horizontal
#define DC_TVW            1132     // Total vertical
#define DC_AHW            1376     // Active horizontal
#define DC_AVW            1076     // Active vertical
#define DC_HBP            56       // Horizontal back porch
#define DC_HFP            60       // Horizantal front porch
#define DC_HSW            44       // Horizontal sync width
#define DC_VBP            8        // Vertical back porch
#define DC_VSW            2        // Vertical sync width
#define DC_VFP            46       // Vertical front porch
#define DC_BPP            30       // Bits per pixel
#define DC_PCLK           104.52

#define FPS               60.0     // Frame rate 60fps
#define FC_FPD_FREQ       6.75

#define GENERAL_STS_MASK       0x51
#define GENERAL_STS_MASK_BIT1  0x02
#define GENERAL_STS_MASK_BIT4  0x10
#define RX_LOCK_DETECT_BIT     6
#define LINK_LOST_FLAG_BIT     4
#define BIST_CRC_ERROR_BIT     3
#define BC_CRC_ERROR_BIT       1
#define LINK_DETECT_BIT        0
#define DAISY_RX_LOCK_DET_P0   6
#define DAISY_BC_LINK_LOST_FLAG_P0 5
#define I2C_WRITE_RETRY_COUNT  3
#define SERDES_RESET_DIGITAL_ALL 0x02

#define APB_CTL        0x48
#define APB_ADR0       0x49
#define APB_ADR1       0x4A
#define APB_DATA0      0x4B
#define APB_DATA1      0x4C
#define APB_DATA2      0x4D
#define APB_DATA3      0x4E
#define APB_DP_RX      0x00
#define APB_DP0_TX     APB_DP_RX
#define APB_DP1_TX     0x01
#define APB_CFG_DATA   0x02
#define APB_DIE_ID     0x03

static int32 verbosedebug = 1;
static int32 reset_keep_dprx = 0;

#define MODULE_NAME "SER-DES DP1MST IPD AUX"
#define ENABLE_VP0_PATTERN 0
#define ENABLE_VP1_PATTERN 0

#define DEBUG_PRINT(name, _str_, ...)               \
    do { \
        if (verbosedebug) { \
            LOG_INFO(name, _str_ , ##__VA_ARGS__); \
        } \
    } while(0)

#define CRITICAL_PRINT(name, _str_, ...)               \
    do { \
        if (verbosedebug) { \
            LOG_CRITICAL_INFO(name, _str_ , ##__VA_ARGS__); \
        } \
    } while(0)

struct video_timing {
    uint32 htotal;
    uint32 vtotal;
    uint32 hres;
    uint32 vres;
    uint32 hstart;
    uint32 vstart;
    uint32 hswidth;
    uint32 vswidth;
};

static uint8 lower_byte(uint16 in_data)
{
    return (uint8)(in_data & 0xFF);
}

static uint8 upper_byte(uint16 in_data)
{
    return (uint8)((in_data & 0xFF00) >> 8);
}

static void serdes_daisy_clear_link_crc_err_flag(int32 fd);
static int32 write_reg(int32 fd, uint8 reg, uint8 val)
{
    uint8 writedata[2];
    int32 iRetVal;
    writedata[0] = reg;
    writedata[1] = val;
    uint8 retry;

    iRetVal = i2c_write(fd, writedata, 2);

    /*Time to time we are seeing i2c_write failures with SER and DES, these are happening very
      randomly and reason is unknown,TI recommended to have retry on i2c_write failure,
      newly added section code will be executed only on i2c_write failure
      There will be additinal 3 retry attempts with 5 msec sleep for each attempt*/

    if (iRetVal < 0)
    {
        CRITICAL_PRINT(MODULE_NAME, "First i2c_write failed 0x%x : 0x%x",  reg, val);
        /*Loop for retry attempts*/

        for (retry = 0; retry < I2C_WRITE_RETRY_COUNT; retry++)
        {
            iRetVal = i2c_write(fd, writedata, 2);
            if (iRetVal < 0)
            {
                if (retry == I2C_WRITE_RETRY_COUNT -1)
                {
                    CRITICAL_PRINT(MODULE_NAME, "FATAL: 3 tries i2c_write failed 0x%x : 0x%x",  reg, val);
                    /*Check if it is alreay reached to max attempts,return if it is already
                      reached else give 5 msec sleep and try*/

                    return -1;
                }
                (void) usleep(5*1000);  /*5 msec sleep*/
            }
            else
            {
                 /*Register write is success*/
                 return  0;
            }
        }
    }

    /*Register write is success*/
    return  0;
}

static int32 read_reg(int32 fd, uint8 reg, uint8 *retval)
{
    uint32 write_data[2];

    write_data[0] = reg;
    int32 iRetVal = i2c_combined_writeread(fd, write_data, 1, retval, 1);

    if (iRetVal < 0) {
        CRITICAL_PRINT(MODULE_NAME, "read_reg(0x%x) FAILED, iRetVal=%d", reg, iRetVal);
        return -1;
    }

    return 0;
}

/*------------------------------------------------------------------------------
 * apb_read_reg
 * Ser-Des chip APB register read function.
 * APB register is indirect register which need combine several register opeations.
 *----------------------------------------------------------------------------*/
static int32 apb_read_reg(int32 fd, uint8 channel, uint16 offset, uint32 *apb_data)
{
    uint8  addr16b_lsb;
    uint8  addr16b_msb;
    uint8  wr_val;
    uint8  apb_data0;
    uint8  apb_data1;
    uint8  apb_data2;
    uint8  apb_data3;
    int32 ret = 0;
    addr16b_lsb = offset & 0xff;
    addr16b_msb = (offset & 0xff00) >> 8;
    ret = write_reg(fd, APB_ADR0, addr16b_lsb);
    if (ret != 0)
        return ret;
    ret = write_reg(fd, APB_ADR1, addr16b_msb);
    if (ret != 0)
        return ret;
    switch (channel)
    {
    case APB_DP_RX:
        wr_val = 0x03;
        break;
    case APB_DP1_TX:
        wr_val = 0x0B;
        break;
    case APB_CFG_DATA:
        wr_val = 0x13;
        break;
    case APB_DIE_ID:
        wr_val = 0x1B;
        break;
    default:
        CRITICAL_PRINT(MODULE_NAME, "Unsupport APB write channel, try write APB channel 0");
        wr_val = 0x03;
        break;
    }
    ret = write_reg(fd, APB_CTL, wr_val);
    if (ret != 0)
        return ret;
    ret = read_reg(fd, APB_DATA0, &apb_data0);
    if (ret != 0)
        return ret;
    ret = read_reg(fd, APB_DATA1, &apb_data1);
    if (ret != 0)
        return ret;
    ret = read_reg(fd, APB_DATA2, &apb_data2);
    if (ret != 0)
        return ret;
    ret = read_reg(fd, APB_DATA3, &apb_data3);
    if (ret != 0)
        return ret;
    *apb_data =  ((apb_data3 << 24) | (apb_data2 << 16) | (apb_data1 << 8) | apb_data0);

    return 0;
}
/*------------------------------------------------------------------------------
 * apb_write_reg
 * Ser-Des chip APB register write function.
 * APB register is indirect register which need combine several register opeations.
 *----------------------------------------------------------------------------*/
static int32 apb_write_reg(int32 fd, uint8 channel, uint16 offset, uint32 apb_data)
{
    uint8 addr16b_lsb;
    uint8 addr16b_msb;
    uint8 wr_val;
    uint8 apb_data0;
    uint8 apb_data1;
    uint8 apb_data2;
    uint8 apb_data3;
    int32 ret = 0;
    switch (channel)
    {
        case APB_DP_RX:
            wr_val = 0x01;
            break;
        case APB_DP1_TX:
            wr_val = 0x09;
            break;
        default:
            CRITICAL_PRINT(MODULE_NAME, "Unsupport APB read channel, try read APB channel 0");
            wr_val = 0x01;
            break;
    }
    ret = write_reg(fd, APB_CTL, wr_val);
    if (ret != 0)
        return ret;
    addr16b_lsb = offset & 0xff;
    addr16b_msb = (offset & 0xff00) >> 8;

    ret = write_reg(fd, APB_ADR0, addr16b_lsb);
    if (ret != 0)
        return ret;
    ret = write_reg(fd, APB_ADR1, addr16b_msb);
    if (ret != 0)
        return ret;

    apb_data0 = apb_data & 0xff;
    apb_data1 = (apb_data >> 8) & 0xff;
    apb_data2 = (apb_data >> 16) & 0xff;
    apb_data3 = (apb_data >> 24) & 0xff;

    ret = write_reg(fd, APB_DATA0, apb_data0);
    if (ret != 0)
        return ret;
    ret = write_reg(fd, APB_DATA1, apb_data1);
    if (ret != 0)
        return ret;
    ret = write_reg(fd, APB_DATA2, apb_data2);
    if (ret != 0)
        return ret;
    ret = write_reg(fd, APB_DATA3, apb_data3);
    return ret;
}


static BridgeChip_StatusType check_mesaured_video(struct video_timing *timing)
{
    if ((timing->vtotal > (TVW -10)) && (timing->vtotal < (TVW + 10)) &&
            (timing->htotal > (THW -10)) && (timing->htotal < (THW + 20)) &&
            (timing->hres == AHW) && (timing->vres == AVW))
    {
        return BRIDGECHIP_STATUS_SUCCESS;
    }
    return BRIDGECHIP_STATUS_FAILED;
}

static BridgeChip_StatusType check_mesaured_video_daisy(struct video_timing *timing)
{
    if ((timing->vtotal > (DC_TVW -10)) && (timing->vtotal < (DC_TVW + 10)) &&
            (timing->htotal > (DC_THW -10)) && (timing->htotal < (DC_THW + 10)) &&
            (timing->hres == DC_AHW) && (timing->vres == DC_AVW))
    {
        return BRIDGECHIP_STATUS_SUCCESS;
    }
    return BRIDGECHIP_STATUS_FAILED;
}

static uint32 serdes_des_measure_video_fpd_iv(int32 fd, int32 print_status)
{
    int32 i;
    uint32 hactive = 0;
    uint8 regval;

    if (print_status)
        CRITICAL_PRINT(MODULE_NAME, "FPD IV Link Layer Timing:");
    write_reg(fd, 0x40, 0x4B);
    write_reg(fd, 0x41, 0x39);
    for (i = 0; i < 4; i++)
    {
        read_reg(fd, 0x42, &regval);
        hactive = regval;
        read_reg(fd, 0x42, &regval);
        hactive = hactive + (regval << 8);
        if (print_status)
        {
            CRITICAL_PRINT(MODULE_NAME, "Stream %d Hactive = %d", i, hactive);
        }
    }
    return 0;
}

static BridgeChip_StatusType serdes_des_measure_video_dtg(int32 fd, int32 port, struct video_timing *timing, int32 print_status)
{
    uint8 offset;
    uint32 htotal, vtotal, hres, vres, hstart, vstart, hswidth, vswidth;
    uint8 read_val;

    if (port == 0)
    {
        offset = 0;
    }
    else
    {
        offset = 0x30;
    }

    do
    {
        write_reg(fd, 0x40, 81);
        write_reg(fd, 0x41, 0x40 + offset);
        read_reg(fd, 0x42, &read_val);
        htotal = read_val * 256;
        write_reg(fd, 0x41, 0x41 + offset);
        read_reg(fd, 0x42, &read_val);
        htotal = htotal + read_val;

        write_reg(fd, 0x41, 0x42 + offset);
        read_reg(fd, 0x42, &read_val);
        vtotal = read_val * 256;
        write_reg(fd, 0x41, 0x43 + offset);
        read_reg(fd, 0x42, &read_val);
        vtotal = vtotal + read_val;

        write_reg(fd, 0x41, 0x44 + offset);
        read_reg(fd, 0x42, &read_val);
        hres = read_val * 256;
        write_reg(fd, 0x41, 0x45 + offset);
        read_reg(fd, 0x42, &read_val);
        hres = hres + read_val;

        write_reg(fd, 0x41, 0x46 + offset);
        read_reg(fd, 0x42, &read_val);
        vres = read_val * 256;
        write_reg(fd, 0x41, 0x47 + offset);
        read_reg(fd, 0x42, &read_val);
        vres = vres + read_val;

        write_reg(fd, 0x41, 0x48 + offset);
        read_reg(fd, 0x42, &read_val);
        hstart = read_val * 256;
        write_reg(fd, 0x41, 0x49 + offset);
        read_reg(fd, 0x42, &read_val);
        hstart = hstart + read_val;

        write_reg(fd, 0x41, 0x4a + offset);
        read_reg(fd, 0x42, &read_val);
        vstart = read_val * 256;
        write_reg(fd, 0x41, 0x4b + offset);
        read_reg(fd, 0x42, &read_val);
        vstart = vstart + read_val;

        write_reg(fd, 0x41, 0x4c + offset);
        read_reg(fd, 0x42, &read_val);
        hswidth = read_val * 256;
        write_reg(fd, 0x41, 0x4d + offset);
        read_reg(fd, 0x42, &read_val);
        hswidth = hswidth + read_val;

        write_reg(fd, 0x41, 0x4e + offset);
        read_reg(fd, 0x42, &read_val);
        vswidth = read_val * 256;
        write_reg(fd, 0x41, 0x4f + offset);
        read_reg(fd, 0x42, &read_val);
        vswidth = vswidth + read_val;
    } while (0);

    if (print_status)
    {
        CRITICAL_PRINT(MODULE_NAME, "Port%d: DTG VIDEO RESOLUTION", port);
        CRITICAL_PRINT(MODULE_NAME, "htotal=%d " \
                "vtotal=%d " \
                "hswidth=%d " \
                "vswidth=%d " \
                "hres =%d " \
                "vres =%d " \
                "hstart=%d " \
                "vstar=%d",
                htotal,
                vtotal,
                hswidth,
                vswidth,
                hres,
                vres,
                hstart,
                vstart);
    }

    if (timing)
    {
        timing->htotal = htotal;
        timing->vtotal = vtotal;
        timing->hres = hres;
        timing->vres = vres;
        timing->hstart = hstart;
        timing->vstart = vstart;
        timing->hswidth = hswidth;
        timing->vswidth = vswidth;
    }
    return BRIDGECHIP_STATUS_SUCCESS;
}

/*------------------------------------------------------------------------------
 * serdes_des_read_video_stream_error_status
 * Diagnostic function to read DES chip ECC and CRC error and can select clear
 * only fucntion by clear_err argument
 *----------------------------------------------------------------------------*/
static BridgeChip_StatusType serdes_des_read_video_stream_error_status(int32 fd, int32 stream_id, int32 err_wait, int32 clear_err, int32 print_status)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    uint8 offset = 0;
    uint8 s_video_comp = 0;
    uint8 bpp = 0;
    uint8 regval = 0;
    uint16 line_length = 0;
    uint16 line_cnt = 0;
    uint16 crc = 0;
    uint16 ecc = 0;
    uint32 data_frame_cnt = 0;
    uint32 ctl_frame_cnt = 0;
    uint8 ecc_error_flag = 0;
    uint8 crc_error_flag = 0;
    uint8 line_lgth_chng = 0;
    uint8 line_cnt_chng = 0;
 
    write_reg(fd, 0x40, 0x48);  // Page 18

    switch (stream_id)
    {
    case 0:
        offset = 0;
        write_reg(fd, 0x41, 0x45);
        read_reg(fd, 0x42, &s_video_comp);
        s_video_comp = s_video_comp & 0x03;
        break;
    case 1:
        offset = 0x10;
        write_reg(fd, 0x41, 0x45);
        read_reg(fd, 0x42, &s_video_comp);
        s_video_comp = (s_video_comp >> 2) & 0x03;
        break;
    case 2:
        offset = 0x20;
        write_reg(fd, 0x41, 0x45);
        read_reg(fd, 0x42, &s_video_comp);
        s_video_comp = (s_video_comp >> 4) & 0x03;
        break;
    case 3:
        offset = 0x30;
        write_reg(fd, 0x41, 0x45);
        read_reg(fd, 0x42, &s_video_comp);
        s_video_comp = (s_video_comp >> 6) & 0x03;
        break;
    case 4:
        offset = 0x40;
        write_reg(fd, 0x41, 0x46);
        read_reg(fd, 0x42, &s_video_comp);
        s_video_comp = s_video_comp & 0x03;
        break;
    case 5:
        offset = 0x50;
        write_reg(fd, 0x41, 0x46);
        read_reg(fd, 0x42, &s_video_comp);
        s_video_comp = (s_video_comp >> 2) & 0x03;
        break;
    default:
        CRITICAL_PRINT(MODULE_NAME, "Unknown stream id = %d", stream_id);
        break;
    }

    if (s_video_comp == 0x00)
        bpp = 18;
    else if (s_video_comp == 0x01)
        bpp = 24;
    else if (s_video_comp == 0x02)
        bpp = 30;
    else
        bpp = 0;

    // ENABLE CAPTURE
    // Disable All Captures (Main & Stream Based)
    write_reg(fd, 0x41, 0x29);
    write_reg(fd, 0x42, 0x00);  // Disable All Captures & Update
    write_reg(fd, 0x41, 0x60 + offset);
    write_reg(fd, 0x42, 0x00);  // Disable All Captures & Update

    // Select All Captures (Main & Stream Based)
    write_reg(fd, 0x41, 0x29);
    write_reg(fd, 0x42, 0x3F);  // Select All Captures
    write_reg(fd, 0x41, 0x60 + offset);
    write_reg(fd, 0x42, 0xF0);  // Select All Captures (Stream Based)

    if (clear_err)
    {
        // Select All Clear Controls (Main & Stream Based)
        write_reg(fd, 0x41, 0x2A);
        write_reg(fd, 0x42, 0x3F);  // Select All Clear Controls
        write_reg(fd, 0x41, 0x60 + offset);
        write_reg(fd, 0x42, 0xFF);  // Select All Clear Controls (Stream Based)

        write_reg(fd, 0x40, 18 * 4);
        // Toggle re_update_en to take effect
        write_reg(fd, 0x41, 0x29);
        write_reg(fd, 0x42, 0x3F);  // Toggle re_update_en
        write_reg(fd, 0x41, 0x29);
        write_reg(fd, 0x42, 0x7F);  // Toggle re_update_en

        // De-Select All Clear Controls (Main & Stream Based)
        write_reg(fd, 0x41, 0x2A);
        write_reg(fd, 0x42, 0x00);  // Select All Clear Controls
        write_reg(fd, 0x41, 0x60 + offset);
        write_reg(fd, 0x42, 0xF0);  // Select All Clear Controls (Stream Based)
    }

    // Toggle re_update_en to take effect
    write_reg(fd, 0x41, 0x29);
    write_reg(fd, 0x42, 0x3F);  // Toggle re_update_en
    write_reg(fd, 0x41, 0x29);
    write_reg(fd, 0x42, 0x7F);  // Toggle re_update_en

    // Clear Stream Error Flags
    write_reg(fd, 0x40, 18 * 4 + 1);
    write_reg(fd, 0x41, 0x69 + offset);
    read_reg(fd, 0x42, &regval);
    read_reg(fd, 0x42, &regval);
    write_reg(fd, 0x40, 18 * 4);

    // WAIT TIMER
    if (err_wait)
        (void) usleep(err_wait * 1000);

    // LINE_LENGTH
    write_reg(fd, 0x41, 0x66 + offset);

    read_reg(fd, 0x42, &regval);
    line_length = regval << 8;
    write_reg(fd, 0x41, 0x65 + offset);
    read_reg(fd, 0x42, &regval);
    line_length = line_length + regval;

    // LINE_COUNT
    write_reg(fd, 0x41, 0x68 + offset);
    read_reg(fd, 0x42, &regval);
    line_cnt = regval << 8;
    write_reg(fd, 0x41, 0x67 + offset);
    read_reg(fd, 0x42, &regval);
    line_cnt = line_cnt + regval;

    // CRC
    write_reg(fd, 0x41, 0x64 + offset);
    read_reg(fd, 0x42, &regval);
    crc = regval << 8;
    write_reg(fd, 0x41, 0x63 + offset);
    read_reg(fd, 0x42, &regval);
    crc = crc + regval;

    // ECC
    write_reg(fd, 0x41, 0x62 + offset);
    read_reg(fd, 0x42, &regval);
    ecc = regval << 8;
    write_reg(fd, 0x41, 0x61 + offset);
    read_reg(fd, 0x42, &regval);
    ecc = ecc + regval;

    // Frame Data Counter
    write_reg(fd, 0x41, 0x33);
    read_reg(fd, 0x42, &regval);
    data_frame_cnt = regval << 16;
    write_reg(fd, 0x41, 0x32);
    read_reg(fd, 0x42, &regval);
    data_frame_cnt = data_frame_cnt + (regval << 8);
    write_reg(fd, 0x41, 0x31);
    read_reg(fd, 0x42, &regval);
    data_frame_cnt = data_frame_cnt + regval;

    // Frame Control Counter
    write_reg(fd, 0x41, 0x30);
    read_reg(fd, 0x42, &regval);
    ctl_frame_cnt = regval << 16;
    write_reg(fd, 0x41, 0x2F);
    read_reg(fd, 0x42, &regval);
    ctl_frame_cnt = ctl_frame_cnt + (regval << 8);
    write_reg(fd, 0x41, 0x2E);
    read_reg(fd, 0x42, &regval);
    ctl_frame_cnt = ctl_frame_cnt + regval;

    // READ FLAGS
    write_reg(fd, 0x40, 18 * 4 + 1);
    write_reg(fd, 0x41, 0x69 + offset);
    read_reg(fd, 0x42, &regval);
    ecc_error_flag = (regval >> 3) & 0x01;
    crc_error_flag = (regval >> 2) & 0x01;
    line_lgth_chng = (regval >> 1) & 0x01;
    line_cnt_chng = regval & 0x01;
    write_reg(fd, 0x40, 18 * 4);

    if (print_status)
    {
        CRITICAL_PRINT(MODULE_NAME, "STREAM=%d ERROR_WAIT=%d CLEAR_ERR=%d", stream_id, err_wait, clear_err);
        CRITICAL_PRINT(MODULE_NAME, "ERROR FLAGS: ECC_ERROR_FLAG=%d CRC_ERROR_FLAG=%d LINE_LGTH_CHNG=%d LINE_CNT_CHNG=%d",
                ecc_error_flag, crc_error_flag, line_lgth_chng, line_cnt_chng);
        CRITICAL_PRINT(MODULE_NAME, "READ_VIDEO_STREAM_ERROR_STATUS: ALL_ERROR_FLAG=0x%02X CRC ERR=%d ECC ERR=%d LINE_LENGTH=%d LINE_CNT=%d BPP=%d",
                regval, crc, ecc, line_length, line_cnt, bpp);
        CRITICAL_PRINT(MODULE_NAME, "DATA_FRAME_CNT=0x%X CONTROL_FRAME_CNT=0x%X STREAM=%d ERROR_WAIT=%d",
                data_frame_cnt, ctl_frame_cnt, stream_id, err_wait);
    }

    return eStatus;
}

static void dser_getclear_0x54(int32 fd, int32 print)
{
    uint8 reg_val;
    i2c_set_slave_addr(fd, DES_ID_7BIT, I2C_ADDRFMT_7BIT);
    read_reg(fd, 0x54, &reg_val);
    if (print)
        CRITICAL_PRINT(MODULE_NAME, "DES 0x54:0x%x(bit2:0x%x)", reg_val, ((reg_val >> 2) & 0x01));
}

static void serdes_clear_link_crc_err_flag(int32 fd)
{
    uint8 reg_cfg2 = 0;

    //CRITICAL_PRINT(MODULE_NAME, "Clear BC CRC Flags");
    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    write_reg(fd, 0x2D, 0x01);
    if (read_reg(fd, 0x02, &reg_cfg2) == 0)
    {
        reg_cfg2 |= 0x20;
        write_reg(fd, 0x02, reg_cfg2);
    }
    if (read_reg(fd, 0x02, &reg_cfg2) == 0)
    {
        reg_cfg2 &= 0xDF;
        write_reg(fd, 0x02, reg_cfg2);
    }
}

static BridgeChip_StatusType serdes_wait_des_link_stable(int32 fd, int32 timeout_ms)
{
    uint8 regval = 0;
    int32 times = timeout_ms / 10;

    do
    {
        if (serdes_get_ser_link_status(fd, 0) == BRIDGECHIP_STATUS_SUCCESS)
        {
            i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
            if (read_reg(fd, 0x00, &regval) == 0)
            {
                if (regval == DES_ID_8BIT)
                {
                    break;
                }
            }
        }
        (void)usleep(10 * 1000);
    }
    while(times--);

    if (times < 0)
    {
        CRITICAL_PRINT(MODULE_NAME, "Wait des link stable timeout, timeout: %d ms", timeout_ms);
        return BRIDGECHIP_STATUS_FAILED;
    }
    else
    {
        return BRIDGECHIP_STATUS_SUCCESS;
    }
}

static BridgeChip_StatusType serdes_wait_des_daisy_link_stable(int32 fd, int32 timeout_ms)
{
    uint8 regval = 0;
    int32 times = timeout_ms / WAIT_INTERVAL;

    do
    {
        if (serdes_get_ser_link_status(fd, 0) == BRIDGECHIP_STATUS_SUCCESS)
        {
            i2c_set_slave_addr(fd, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
            if (read_reg(fd, 0x00, &regval) == 0)
            {
                if (regval == DES_DC_ID_8BIT)
                {
                    break;
                }
            }
        }
        (void)usleep(WAIT_INTERVAL * 1000);
    }
    while(times--);

    if (times < 0)
    {
        CRITICAL_PRINT(MODULE_NAME, "Wait des link stable timeout, timeout: %d ms", timeout_ms);
        return BRIDGECHIP_STATUS_FAILED;
    }
    else
    {
        return BRIDGECHIP_STATUS_SUCCESS;
    }
}

/*------------------------------------------------------------------------------
 * serializer_i2c_config
 *
 *----------------------------------------------------------------------------*/
static BridgeChip_StatusType serializer_i2c_config(int32 fd)
{
    CRITICAL_PRINT(MODULE_NAME, "I2C Targets and controller configurations");
    CRITICAL_PRINT(MODULE_NAME, "================================================");
    CRITICAL_PRINT(MODULE_NAME, " I2C connections:");
    CRITICAL_PRINT(MODULE_NAME, " I2C Target 0 <----> SDA0/SCL0");
    CRITICAL_PRINT(MODULE_NAME, " I2C Target 1 X");
    CRITICAL_PRINT(MODULE_NAME, " I2C Target 2 X");
    CRITICAL_PRINT(MODULE_NAME, " Remote I2C controller: Disabled");
    CRITICAL_PRINT(MODULE_NAME, "================================================");
    // Disable I2C remote control
    write_reg(fd, 0x3A, 0x88);
    //No connection to I2C Target 0, I2C Target 1 connect to second I2C Port (SDA1/SCL1)
    write_reg(fd, 0x38, 0x20);
    //No connection to I2C Target 2
    write_reg(fd, 0x39, 0x00);

    return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType link_set_serializer_i2c_pass_through(int32 fd)
{
    write_reg(fd, 0x2D, 0x01);                     //Write enable for TX port 0 registers.
    write_reg(fd, 0x70, DES_ID_8BIT);              //Remote Slave Device ID 0
    write_reg(fd, 0x78, DES_ID_8BIT_ALIAS);        //Remote Slave Device Alias ID 0
    write_reg(fd, 0x88, 0x00);
    write_reg(fd, 0x71, DES_DC_ID_8BIT);           //Remote Slave Device ID 1
    write_reg(fd, 0x79, DES_DC_ID_8BIT_ALIAS);     //Remote Slave Device Alias ID 1
    write_reg(fd, 0x89, 0x05);
    write_reg(fd, 0x72, TOUCH_DC_ID_8BIT);        //Remote Slave Device ID 2
    write_reg(fd, 0x7A, TOUCH_DC_ID_8BIT_ALIAS);  //Remote Slave Device Alias ID 2
    write_reg(fd, 0x8A, 0x05);
    write_reg(fd, 0x73, MCU_ID_8BIT);             //Remote Slave Device ID 3
    write_reg(fd, 0x7B, MCU_ID_8BIT_ALIAS);       //Remote Slave Device Alias ID 3
    write_reg(fd, 0x8B, 0x00);
    write_reg(fd, 0x74, MCU_DC_ID_8BIT);          //Remote Slave Device ID 4
    write_reg(fd, 0x7C, MCU_DC_ID_8BIT_ALIAS);    //Remote Slave Device Alias ID 4
    write_reg(fd, 0x8C, 0x05);

    //Set SER I2C speed to 1MHz
    write_reg(fd, 0x2B, 0x08);
    write_reg(fd, 0x2C, 0x08);

    CRITICAL_PRINT(MODULE_NAME, "Enable I2C Pass Through");
    write_reg(fd, 0x07, 0x88);          //CRC Checker Enable, Pass-Through Enabled
    CRITICAL_PRINT(MODULE_NAME, "Set i2c pass through done");

    return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType link_set_serializer_gpio(int32 fd)
{
    uint8 RevId = 0;

    CRITICAL_PRINT(MODULE_NAME, "Set Serializer 983 GPIOs...");
    read_reg(fd, 0x30, &RevId);
    RevId &= 0xF0;
    if (RevId >= 0x40)
    {
        CRITICAL_PRINT(MODULE_NAME, "Touch GPIO interrupt serializer on GPIO 0..");
        //write_reg(fd, 0x1C, 0x80);  //GPIO5 Output Enable, Received PORT0 BC_GPIO0.
        //write_reg(fd, 0x1E, 0x81);  //GPIO7 Output Enable, Received PORT1 BC_GPIO1.
    }
    else
    {
        //write_reg(fd, 0x1C, 0x40);  //GPIO5 Output Disabled,Received PORT0 BC_GPIO0.
        //write_reg(fd, 0x1E, 0x41);  //GPIO7 Output Disabled,Received PORT1 BC_GPIO1.
    }
     
	write_reg(fd, 0x1A, 0x81);  //Map GPIO3 to DES_BC_GPIO1 (MCU IRQ)
	write_reg(fd, 0x1E, 0x90);  //Map GPIO7 to DES_BC_GPIO0 on Port1 (Aux Touch)
    
	return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType deserializer_disable_i2c_remote_slaves(int32 fd)
{
    CRITICAL_PRINT(MODULE_NAME, "Disable I2C remote slaves on DES");
    write_reg(fd, 0xe, 0x3);   //Select Port 0&1
    write_reg(fd, 0x8, 0x20);  //Apply I2C Workaround

    return BRIDGECHIP_STATUS_SUCCESS;
}

//only can be called after softreset
static BridgeChip_StatusType link_set_serializer_dprx(int32 fd)
{
    //uint8 cal_sensitivity_off = 0;
    uint8 cal_sensitivity_on = 0;
    uint8 sm_sts = 0;
    uint8 retry = 0;

    while (sm_sts != 7 && retry<4)
    {
        retry = retry + 1;
        //# *********************************************
        //# Set DP Config
        CRITICAL_PRINT(MODULE_NAME, "Set DP Config with 8.1 plus FIFO Flush with FIFO margin workaround retrycount : %d", retry);
        //# *********************************************
        write_reg(fd, 0x40, 0x24);  // Indirect Page: 9
        write_reg(fd, 0x41, 0x04);  // Indirect Reg: 0x04
        write_reg(fd, 0x42, 0x80);  // DPRX Trap State Mode enabled
        CRITICAL_PRINT(MODULE_NAME, "DPRX Trap State Mode enabled");

        write_reg(fd, 0x41, 0x03);  // Indirect Reg: 0x03
        write_reg(fd, 0x42, 0x07);  // Trapped to State 7
        CRITICAL_PRINT(MODULE_NAME, "Trapped to State 7");

        // overrides link rate to 8.1
        write_reg(fd, 0x41, 0x2C);  // Indirect Reg: 0x2C
        write_reg(fd, 0x42, 0x37);  // Overriding link rate to 8.1
        CRITICAL_PRINT(MODULE_NAME, "Overrides link rate to 8.1");

        // overrides number of lanes to 4
        write_reg(fd, 0x41, 0x2D);  // Indirect Reg: 0x2D
        write_reg(fd, 0x42, 0x1F);  // Overriding number of lanes to 4
        CRITICAL_PRINT(MODULE_NAME, "Overrides number of lanes to 4");

        // DPRX reset
        write_reg(fd, 0x01, 0x40);  // DPRX_RESET
        (void) usleep(10*1000);      // 10 msec Delay for DPRX to settle, recommended by TI
        write_reg(fd, 0x40, 0x24);  // Indirect Page: 9
        write_reg(fd, 0x41, 0x02);  // Indirect Reg: 0x07
        write_reg(fd, 0x42, 0x67);  // # Force state 7
        CRITICAL_PRINT(MODULE_NAME, "Force State 7");

        write_reg(fd, 0x41, 0x04);  // Indirect Reg: 0x04
        write_reg(fd, 0x42, 0xC0);  // # Force state ovr
        CRITICAL_PRINT(MODULE_NAME, "Force State ovr");

        write_reg(fd, 0x41, 0x04);  // Indirect Reg: 0x04
        write_reg(fd, 0x42, 0x40);  // DPRX Trap State Mode disabled
        CRITICAL_PRINT(MODULE_NAME, "DPRX Trap State Mode disabled");

        write_reg(fd, 0x41, 0x01);  // Indirect Reg: 0x01
        write_reg(fd, 0x42, 0x02);  // enables HPD override, manually pulls HPD low
        CRITICAL_PRINT(MODULE_NAME, "enables HPD override, manually pulls HPD low");

        apb_write_reg(fd, APB_DP_RX, 0x74, 0x1e);  //Set max advertised link rate = 8.1Gbps
        CRITICAL_PRINT(MODULE_NAME, "Set max advertised link rate = 8.1Gbps");

        apb_write_reg(fd, APB_DP_RX, 0x70, 0x04);  //Set max advertised lane count = 4
        CRITICAL_PRINT(MODULE_NAME, "Set max advertised lane count = 4");

        apb_write_reg(fd, APB_DP_RX, 0x214, 0x02);  //Request min VOD swing of 0x02
        CRITICAL_PRINT(MODULE_NAME, "Request min VOD swing of 0x02");

        CRITICAL_PRINT(MODULE_NAME, "Lane Rate Optimizations for CS2.0 with 8155 at 8.1Gbps (No SSC)");
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x30);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0x4C);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0x50);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0x56);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0xB0);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0xCC);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0xD0);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0xD6);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x30);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0x4C);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0x50);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0x56);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0xB0);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0xCC);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0xD0);
        write_reg(fd, 0x42, 0x00);
        write_reg(fd, 0x41, 0xD6);
        write_reg(fd, 0x42, 0x00);

        apb_write_reg(fd, APB_DP_RX, 0x18, 0x114);     //Set SST/MST mode and DP/eDP Mode - MST support
        CRITICAL_PRINT(MODULE_NAME, "SET MST");

        apb_write_reg(fd, APB_DP_RX, 0x904, 0x1);      //Set MST Playload ID for VS 0
        apb_write_reg(fd, APB_DP_RX, 0x908, 0x2);      //Set MST Playload ID for VS 1
        apb_write_reg(fd, APB_DP_RX, 0xA00, 0x1);      //Enable MST VS0
        //apb_write_reg(fd, APB_DP_RX, 0xA04, 0x4);      //Set MST VS0 Pixel Width
        //apb_write_reg(fd, APB_DP_RX, 0xA0C, 0x1);      //Disable line reset for VS0
        apb_write_reg(fd, APB_DP_RX, 0xA18, 0x1);      //Set DTG mode for MST VS0
        apb_write_reg(fd, APB_DP_RX, 0xA14, 0x1);      //Enable MST VS0 DTG
        apb_write_reg(fd, APB_DP_RX, 0xB00, 0x1);      //Enable MST VS1
        //apb_write_reg(fd, APB_DP_RX, 0xB04, 0x4);      //Set MST VS1 Pixel Width
        //apb_write_reg(fd, APB_DP_RX, 0xB0C, 0x1);      //Disable line reset for VS1
        apb_write_reg(fd, APB_DP_RX, 0xB18, 0x1);      //Set DTG mode for MST VS1
        apb_write_reg(fd, APB_DP_RX, 0xB14, 0x1);      //Enable MST VS1 DTG
        //apb_write_reg(fd, APB_DP_RX, 0x00, 0x01);     //Force HPD high to trigger link training - this shouldn't be here. Logan 1/9/2025

        // Master clk for i_phase sequence is Lane0
        // Disabling i_phase_en for Lane clk1
        write_reg(fd, 0x40, 0x10);  // Indirect Page: 4
        write_reg(fd, 0x41, 0xB8);  // Indirect Reg: 0xB8
        write_reg(fd, 0x42, 0x01);
        // Disabling i_phase_en for Lane clk2
        write_reg(fd, 0x40, 0x14);  // Indirect Page: 5
        write_reg(fd, 0x41, 0x38);  // Indirect Reg: 0x38
        write_reg(fd, 0x42, 0x01);
        // Disabling i_phase_en for Lane clk3
        //write_reg(fd, 0x40, 0x14);  // Indirect Page: 5
        write_reg(fd, 0x41, 0xB8);  // Indirect Reg: 0xB8
        write_reg(fd, 0x42, 0x01);
        // Master clk for i_phase sequence is Lane0
        // Disabling i_phase_en for Lane clk0
        write_reg(fd, 0x40, 0x10); // Indirect Page: 4
        write_reg(fd, 0x41, 0x38);  // Indirect Reg: 0x38
        write_reg(fd, 0x42, 0x01);

        // Resetting DPRX DESER for main read fifo clk0
        write_reg(fd, 0x40, 0x10);  // Indirect Page: 4
        write_reg(fd, 0x41, 0xa3);  // Indirect Reg: 0xa3
        write_reg(fd, 0x42, 0x0C);  // deser reset ln1
        write_reg(fd, 0x40, 0x14);  // Indirect Page: 4
        write_reg(fd, 0x41, 0x23);  // Indirect Reg: 0x23
        write_reg(fd, 0x42, 0x0C);  // deser reset ln0
        write_reg(fd, 0x40, 0x14);  // Indirect Page: 4
        write_reg(fd, 0x41, 0xa3);  // Indirect Reg: 0xa3
        write_reg(fd, 0x42, 0x0C);  // deser reset ln1

        write_reg(fd, 0x40, 0x10);  // Indirect Page: 4
        write_reg(fd, 0x41, 0xa3);  // Indirect Reg: 0xa3
        write_reg(fd, 0x42, 0x00);  // deser reset ln1
        write_reg(fd, 0x40, 0x14);  // Indirect Page: 4
        write_reg(fd, 0x41, 0x23);  // Indirect Reg: 0x23
        write_reg(fd, 0x42, 0x00);  // deser reset ln0
        //write_reg(fd, 0x40, 0x14);  // Indirect Page: 4
        write_reg(fd, 0x41, 0xa3);  // Indirect Reg: 0xa3
        write_reg(fd, 0x42, 0x00);  // deser reset ln1
        //time.sleep(0.001);
        (void) usleep(1000);  /*1 msec sleep*/
        write_reg(fd, 0x40, 0x10);  // Indirect Page: 4
        write_reg(fd, 0x41, 0x23);  // Indirect Reg: 0x23
        write_reg(fd, 0x42, 0x00);  // clr deser reset ln0
        write_reg(fd, 0x40, 0x24);  // Indirect Page: 9
        write_reg(fd, 0x41, 0x07);  // Indirect Reg: 0x07
        write_reg(fd, 0x42, 0x10);  // fifo sts clr
        //write_reg(fd, 0x40, 0x24);  // Indirect Page: 9
        //write_reg(fd, 0x41, 0x07);  // Indirect Reg: 0x07
        write_reg(fd, 0x42, 0x00);  // fifo sts clr
        write_reg(fd, 0x40, 0x10);  // Indirect Page: 4
        write_reg(fd, 0x41, 0x38);  // Indirect Reg: 0x38

        uint8 shift_counter = 0;
        uint8 sub_shift = 0;
        for (shift_counter = 0; shift_counter < 3; shift_counter++){
            for (sub_shift = 1; sub_shift < 66; sub_shift += 4){
                cal_sensitivity_on = (sub_shift) * 2 + 1;
                if (sub_shift == 65){
                    write_reg(fd, 0x42, 0x1);
                }
                else{
                    write_reg(fd, 0x42, cal_sensitivity_on);
				}
			}
        }
        // Master clk for i_phase sequence is Lane0
        // Disabling i_phase_en for Lane clk1
        write_reg(fd, 0x40, 0x10);  // Indirect Page: 4
        write_reg(fd, 0x41, 0xB8);  // Indirect Reg: 0xB8
        write_reg(fd, 0x42, 0x00);
        // Disabling i_phase_en for Lane clk2
        write_reg(fd, 0x40, 0x14);  // Indirect Page: 5
        write_reg(fd, 0x41, 0x38);  // Indirect Reg: 0x38
        write_reg(fd, 0x42, 0x00);
        // Disabling i_phase_en for Lane clk3
        write_reg(fd, 0x40, 0x14);  // Indirect Page: 5
        write_reg(fd, 0x41, 0xB8);  // Indirect Reg: 0xB8
        write_reg(fd, 0x42, 0x00);
        // Master clk for i_phase sequence is Lane0
        // Disabling i_phase_en for Lane clk0
        write_reg(fd, 0x40, 0x10); // Indirect Page: 4
        write_reg(fd, 0x41, 0x38);  // Indirect Reg: 0x38
        write_reg(fd, 0x42, 0x00);

        write_reg(fd, 0x40, 0x24);  // Indirect Page: 9
        write_reg(fd, 0x41, 0x2C);  // Indirect Reg: 0x2C
        write_reg(fd, 0x42, 0x07);  // Release TPS4 Force, keep at 8.1Gbps

        CRITICAL_PRINT(MODULE_NAME, "Release TPS4 Force, keep at 8.1Gbps");
        (void) usleep(1*1000);
        write_reg(fd, 0x41, 0x04);  // Indirect Reg: 0x04
        write_reg(fd, 0x42, 0x00);  // Force State Mode disabled
        CRITICAL_PRINT(MODULE_NAME, "Force State Mode disabled");
        (void) usleep(1*1000);
        // Read sm status
        write_reg(fd, 0x40, 0x25);  // Indirect Page: 9
        write_reg(fd, 0x41, 0x2b);  // Indirect Reg: 0xC3
        read_reg(fd, 0x42, &sm_sts);
        //sm_sts = board.ReadI2C(SER_ID_8BIT, 0x42);  // dbg_mux_data_obs_7_0Readback_Val[7:0] - 0xC0
        CRITICAL_PRINT(MODULE_NAME, "SM status: 0x%X", sm_sts);
	}
    // disable HPD override
    write_reg(fd, 0x41, 0x01);  // Indirect Reg: 0x01
    write_reg(fd, 0x42, 0x00);  // disable HPD override
    CRITICAL_PRINT(MODULE_NAME, "disable HPD override");
    CRITICAL_PRINT(MODULE_NAME, "Allow time after HPD is pulled high for the source to train and provide video (may need to adjust based on source properties)");

    return BRIDGECHIP_STATUS_SUCCESS;
}
/*------------------------------------------------------------------------------
 * serdes_diagnostic_983dprx
 * called if DP link training and video is not ready.
 *----------------------------------------------------------------------------*/
void serdes_diagnostic_983dprx(int32 fd)
{
    uint8 intr_sts_dp_rx, regval;
    uint32 apb_val;
    uint32 i = 0;
    CRITICAL_PRINT(MODULE_NAME, "----------983 DPRX DIAGNOSTIC------------");
    for (i = 0x490; i <= 0x49C; i += 4)
    {
        (void) apb_read_reg(fd, APB_DP_RX, i, &apb_val);
        CRITICAL_PRINT(MODULE_NAME, "APB: 0x%x - 0x%08x", i, apb_val);
        (void) usleep(10 * 1000);
    }
    (void) apb_read_reg(fd, APB_DP_RX, 0x188, &apb_val);
    CRITICAL_PRINT(MODULE_NAME, "APB: 0x188 - 0x%08x", apb_val);
    (void) usleep(10 * 1000);
    (void) apb_read_reg(fd, APB_DP_RX, 0x194, &apb_val);
    CRITICAL_PRINT(MODULE_NAME, "APB: 0x194 - 0x%08x", apb_val);
    write_reg(fd, 0x40, 0x25);
    write_reg(fd, 0x41, 0x3F);
    read_reg(fd, 0x42, &intr_sts_dp_rx);
    CRITICAL_PRINT(MODULE_NAME, "Page 9: 0x3F - 0x%02x", intr_sts_dp_rx);
    read_reg(fd, 0x45, &regval);
    CRITICAL_PRINT(MODULE_NAME, "Main Page: 0x45 - 0x%02x", regval);
    write_reg(fd, 0x40, 0x31);
    write_reg(fd, 0x41, 0x30);
    read_reg(fd, 0x42, &regval);
    CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x30 - 0x%02x", regval);
    write_reg(fd, 0x41, 0x31);
    read_reg(fd, 0x42, &regval);
    CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x31 - 0x%02x", regval);
    write_reg(fd, 0x41, 0x70);
    read_reg(fd, 0x42, &regval);
    CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x70 - 0x%02x", regval);
    write_reg(fd, 0x41, 0x71);
    read_reg(fd, 0x42, &regval);
    CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x71 - 0x%02x", regval);
    CRITICAL_PRINT(MODULE_NAME, "----------983 DPRX DIAGNOSTIC------------");
}

static BridgeChip_StatusType link_set_serializer_fpd4(int32 fd)
{
    uint8 i2c_pass_through, i2c_pass_through_mask, i2c_pass_through_reg;

    CRITICAL_PRINT(MODULE_NAME, "Program SER to FPD-Link IV mode");
    write_reg(fd, 0x5b, 0x23);  //Disable FPD3 FIFO pass through
    write_reg(fd, 0x5, 0x28);   //Force FPD4_TX dual mode

    CRITICAL_PRINT(MODULE_NAME, "Set up FPD IV PLL Settings");
    write_reg(fd, 0x40, 0x8);   //Select PLL reg page
    write_reg(fd, 0x41, 0x1b);
    write_reg(fd, 0x42, 0x8);   //Disable PLL0
    write_reg(fd, 0x41, 0x5b);
    write_reg(fd, 0x42, 0x8);   //Disable PLL1
    write_reg(fd, 0x2, 0xd1);   //Enable mode overwrite
    write_reg(fd, 0x2d, 0x1);
    write_reg(fd, 0x40, 0x8);   //Select PLL page
    write_reg(fd, 0x41, 0x5);   //Select Ncount Reg
    write_reg(fd, 0x42, 0x7d);  //Set Ncount
    write_reg(fd, 0x41, 0x13);  //Select post div reg
    write_reg(fd, 0x42, 0x90);  //Set post div for 6.75 Gbps
    write_reg(fd, 0x2d, 0x1);   //Select write reg to port 0
    write_reg(fd, 0x6a, 0xa);   //set BC sampling rate
    write_reg(fd, 0x6e, 0x80);  //set BC fractional sampling
    write_reg(fd, 0x40, 0x4);   //Select FPD page and set BC settings for FPD IV port 0
    write_reg(fd, 0x41, 0x6);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0xd);
    write_reg(fd, 0x42, 0x34);
    write_reg(fd, 0x41, 0xe);
    write_reg(fd, 0x42, 0x53);
    write_reg(fd, 0x40, 0x8);   //Select PLL page
    write_reg(fd, 0x41, 0x45);  //Select Ncount Reg
    write_reg(fd, 0x42, 0x7d);  //Set Ncount
    write_reg(fd, 0x41, 0x53);  //Select post div reg
    write_reg(fd, 0x42, 0x90);  //Set post div for 6.75 Gbps
    write_reg(fd, 0x2d, 0x12);  //Select write reg to port 1
    write_reg(fd, 0x6a, 0xa);   //set BC sampling rate
    write_reg(fd, 0x6e, 0x80);  //set BC fractional sampling
    write_reg(fd, 0x40, 0x4);   //Select FPD page and set BC settings for FPD IV port 1
    write_reg(fd, 0x41, 0x26);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0x2d);
    write_reg(fd, 0x42, 0x34);
    write_reg(fd, 0x41, 0x2e);
    write_reg(fd, 0x42, 0x53);
    write_reg(fd, 0x2, 0xd1);   //Set HALFRATE_MODE

    CRITICAL_PRINT(MODULE_NAME, "Zero out PLL fractional");
    write_reg(fd, 0x40, 0x8);   //Select PLL page
    write_reg(fd, 0x41, 0x4);
    write_reg(fd, 0x42, 0x1);
    write_reg(fd, 0x41, 0x1e);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0x1f);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0x20);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0x44);
    write_reg(fd, 0x42, 0x1);
    write_reg(fd, 0x41, 0x5e);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0x5f);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0x60);
    write_reg(fd, 0x42, 0x0);

    CRITICAL_PRINT(MODULE_NAME, "Configure and Enable PLLs"); 
    write_reg(fd, 0x41, 0xe);   //Select VCO reg
    write_reg(fd, 0x42, 0xc7);  //Set VCO
    write_reg(fd, 0x41, 0x4e);  //Select VCO reg
    write_reg(fd, 0x42, 0xc7);  //Set VCO
    write_reg(fd, 0x1, 0x30);   //soft reset PLL
    write_reg(fd, 0x40, 0x8);   //Select PLL page
    write_reg(fd, 0x41, 0x1b);
    write_reg(fd, 0x42, 0x0);   //Enable PLL0
    write_reg(fd, 0x41, 0x5b);
    write_reg(fd, 0x42, 0x0);   //Enable PLL1
    CRITICAL_PRINT(MODULE_NAME, "Software reset 983");
    if (reset_keep_dprx)
      write_reg(fd, 0x01, 0x30);
    else
      write_reg(fd, 0x01, 0x01);  //soft reset Ser
    (void)usleep(50 * 1000);

    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    CRITICAL_PRINT(MODULE_NAME, "Enable I2C Passthrough");
    read_reg(fd, 0x7, &i2c_pass_through);
    i2c_pass_through_mask = 0x08;
    i2c_pass_through_reg = i2c_pass_through | i2c_pass_through_mask;
    write_reg(fd, 0x07, i2c_pass_through_reg);
    serdes_wait_des_link_stable(fd, 50);

    CRITICAL_PRINT(MODULE_NAME," Software reset deserializer");
    i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    write_reg(fd, 0x01, 0x01);  //Soft reset Des
    (void)usleep(50 * 1000);
    serdes_wait_des_link_stable(fd, 50);

    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    write_reg(fd, 0x2d, 0x1);   //Select write to port0 reg
    CRITICAL_PRINT(MODULE_NAME,"END OF FPD4 CONVERSION");
    return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType link_set_serializer_vp(int32 fd)
{
    uint8 n_value = 15;
    float fc_freq = 0;
    float src_pclk_freq = 0;
    float m_value_f;
    uint16 m_value;
	uint8 apbData0, apbData1, apbData2, apbData3;
    uint32 apbData;

    CRITICAL_PRINT(MODULE_NAME, "Configure VP0");
    write_reg(fd, 0x2D, 0x01);  //select port 0 TX
    write_reg(fd, 0x40, 0x30);  //select VP03 indirect registers page
    // Set Stream Source to VPs
    //write_reg(fd, 0x41, 0x01);
    //write_reg(fd, 0x42, 0xA8);  //Set VP_SRC_SELECT to Stream 0 for SST Mode
    // set timing on video processor 0
    // set dp_h_active
    write_reg(fd, 0x41, 0x02);
    write_reg(fd, 0x42, lower_byte(AHW));  //VID H Active
    write_reg(fd, 0x41, 0x03);
    write_reg(fd, 0x42, upper_byte(AHW));  //VID H Active
    // Set crop for VP0
    write_reg(fd, 0x41, 0x10);  //h_active
    write_reg(fd, 0x42, lower_byte(AHW));
    write_reg(fd, 0x41, 0x11);
    write_reg(fd, 0x42, upper_byte(AHW));
    write_reg(fd, 0x41, 0x12);
    write_reg(fd, 0x42, lower_byte(HBP));
    write_reg(fd, 0x41, 0x13);
    write_reg(fd, 0x42, upper_byte(HBP));  //program h back porch
    write_reg(fd, 0x41, 0x14);
    write_reg(fd, 0x42, lower_byte(HSW));
    write_reg(fd, 0x41, 0x15);
    write_reg(fd, 0x42, upper_byte(HSW));  //program h sync
    write_reg(fd, 0x41, 0x16);
    write_reg(fd, 0x42, lower_byte(THW));
    write_reg(fd, 0x41, 0x17);
    write_reg(fd, 0x42, upper_byte(THW));  //program h_tot
    write_reg(fd, 0x41, 0x18);
    write_reg(fd, 0x42, lower_byte(AVW));
    write_reg(fd, 0x41, 0x19);
    write_reg(fd, 0x42, upper_byte(AVW));  //program v_act
    write_reg(fd, 0x41, 0x1A);
    write_reg(fd, 0x42, lower_byte(VBP));
    write_reg(fd, 0x41, 0x1B);
    write_reg(fd, 0x42, upper_byte(VBP));  //program v back porch
    write_reg(fd, 0x41, 0x1C);
    write_reg(fd, 0x42, lower_byte(VSW));
    write_reg(fd, 0x41, 0x1D);
    write_reg(fd, 0x42, upper_byte(VSW));  //program v sync
    write_reg(fd, 0x41, 0x1E);
    write_reg(fd, 0x42, lower_byte(VFP));
    write_reg(fd, 0x41, 0x1F);
    write_reg(fd, 0x42, upper_byte(VFP));  //program v front porch
    write_reg(fd, 0x41, 0x27);
    write_reg(fd, 0x42, 0x00);   //HSYNC Polarity = +,VSYNC Polarity = +
    //write_reg(fd, 0x42, 0x04); //Enable Cropping

    // set m/n value for video processor 0
    // only for FPD4 mode
    // n_value fixed as 15
    n_value = 0xF;
    // forward channel rate
    fc_freq = FC_FPD_FREQ *1000;  //6750  bps
    src_pclk_freq = THW * TVW * FPS / 1000000;
    CRITICAL_PRINT(MODULE_NAME, "VP0 PCLK = %f MHz",  src_pclk_freq);
    m_value_f = src_pclk_freq / 4.0 * (1 <<  n_value) / (fc_freq / 40.0);
    m_value = (uint16)round(m_value_f);
    CRITICAL_PRINT(MODULE_NAME, "The source pixel clock is %f",  src_pclk_freq);
    CRITICAL_PRINT(MODULE_NAME, "The M value for VP0 is m_value = %d",  m_value);
    write_reg(fd, 0x41, 0x23);
    write_reg(fd, 0x42, lower_byte(m_value));
    write_reg(fd, 0x41, 0x24);
    write_reg(fd, 0x42, upper_byte(m_value)); //set M divider
    write_reg(fd, 0x41, 0x25);
    write_reg(fd, 0x42, n_value);             //set N divider

#if ENABLE_VP0_PATTERN
    CRITICAL_PRINT(MODULE_NAME, "Enable VP0 Patgen", __func__);
    write_reg(fd, 0x40, 0x30);
    write_reg(fd, 0x41, 0x29);
    write_reg(fd, 0x42, 0x10);  //Set PATGEN Color Depth to 30bpp for VP0
    write_reg(fd, 0x41, 0x28);
    write_reg(fd, 0x42, 0x95);  //Enable PATGEN on VP0
#endif
    CRITICAL_PRINT(MODULE_NAME, "Configure VP1");
    write_reg(fd, 0x40, 0x32);  //select VP03 indirect registers page
    //write_reg(fd, 0x41, 0x41);
    //write_reg(fd, 0x42, 0xA9);
    write_reg(fd, 0x41, 0x42);
    write_reg(fd, 0x42, lower_byte(DC_AHW));  //VID H Active
    write_reg(fd, 0x42, upper_byte(DC_AHW));  //VID H Active
/*
    //Set filter for VP1
    write_reg(fd, 0x41, 0x46);
    write_reg(fd, 0x42, VFILTER_A_VP1);
    write_reg(fd, 0x42, VFILTER_N_VP1);
    //Set crop for VP1
    write_reg(fd, 0x41, 0x48);
    write_reg(fd, 0x42, 0xe4);  //crop start x
    write_reg(fd, 0x42, 0x0d);  //Crop Start x
    write_reg(fd, 0x42, 0x00);  //Crop Start Y
    write_reg(fd, 0x42, 0x00);  //Crop Start Y
    write_reg(fd, 0x42, 0x43);  //Crop Stop X
    write_reg(fd, 0x42, 0x13);  //Crop Stop X
    write_reg(fd, 0x42, 0x33);  //Crop Stop Y
    write_reg(fd, 0x42, 0x4);   //Crop Stop Y
*/
    write_reg(fd, 0x41, 0x50);
    write_reg(fd, 0x42, lower_byte(DC_AHW));
    write_reg(fd, 0x41, 0x51);
    write_reg(fd, 0x42, upper_byte(DC_AHW));  //VID H Active
    write_reg(fd, 0x41, 0x52);
    write_reg(fd, 0x42, lower_byte(DC_HBP));
    write_reg(fd, 0x41, 0x53);
    write_reg(fd, 0x42, upper_byte(DC_HBP));  //program h back porch
    write_reg(fd, 0x41, 0x54);
    write_reg(fd, 0x42, lower_byte(DC_HSW));
    write_reg(fd, 0x41, 0x55);
    write_reg(fd, 0x42, upper_byte(DC_HSW));  //program h sync
    write_reg(fd, 0x41, 0x56);
    write_reg(fd, 0x42, lower_byte(DC_THW));
    write_reg(fd, 0x41, 0x57);
    write_reg(fd, 0x42, upper_byte(DC_THW));  //program h_tot
    write_reg(fd, 0x41, 0x58);
    write_reg(fd, 0x42, lower_byte(DC_AVW));
    write_reg(fd, 0x41, 0x59);
    write_reg(fd, 0x42, upper_byte(DC_AVW));  //program v_act
    write_reg(fd, 0x41, 0x5A);
    write_reg(fd, 0x42, lower_byte(DC_VBP));
    write_reg(fd, 0x41, 0x5B);
    write_reg(fd, 0x42, upper_byte(DC_VBP));  //program v back porch
    write_reg(fd, 0x41, 0x5C);
    write_reg(fd, 0x42, lower_byte(DC_VSW));
    write_reg(fd, 0x41, 0x5D);
    write_reg(fd, 0x42, upper_byte(DC_VSW));  //program v sync
    write_reg(fd, 0x41, 0x5E);
    write_reg(fd, 0x42, lower_byte(DC_VFP));
    write_reg(fd, 0x41, 0x5F);
    write_reg(fd, 0x42, upper_byte(DC_VFP));  //program v front porch
    write_reg(fd, 0x41, 0x67);
    write_reg(fd, 0x42, 0x00);
    //write_reg(fd, 0x41, 0x40);
    //write_reg(fd, 0x42, 0x04);  //Enable Cropping and Enable Vertical Filter processing

    //set m/n value for video processor 1
    //only for FPD4 mode
    //n_value fixed as 15
    n_value = 0xF;
    //forward channel rate
    fc_freq = FC_FPD_FREQ *1000;  //6750  bps
    src_pclk_freq = DC_THW * DC_TVW * FPS / 1000000;
    CRITICAL_PRINT(MODULE_NAME, "VP1 PCLK = %f MHz",  src_pclk_freq);
    m_value_f = src_pclk_freq / 4.0 * (1 <<  n_value) / (fc_freq / 40.0);
    m_value = (uint16)round(m_value_f);
    CRITICAL_PRINT(MODULE_NAME, "The source pixel clock is %f",  src_pclk_freq);
    CRITICAL_PRINT(MODULE_NAME, "The M value for VP1 is m_value = %d",  m_value);
    write_reg(fd, 0x41, 0x63);
    write_reg(fd, 0x42, lower_byte(m_value));
    write_reg(fd, 0x41, 0x64);
    write_reg(fd, 0x42, upper_byte(m_value));  //set M divider
    write_reg(fd, 0x41, 0x65);
    write_reg(fd, 0x42, n_value);              //set N divider
	CRITICAL_PRINT(MODULE_NAME, "Disable Frame Reset");
	write_reg(fd,0x48,0x1);//Enable APB Interface
	write_reg(fd,0x49,0x18);//Check for Video
	write_reg(fd,0x4a,0xa);
	write_reg(fd,0x48,0x3);
	read_reg(fd,0x4b,&apbData0);
	read_reg(fd,0x4c,&apbData1);
	read_reg(fd,0x4d,&apbData2);
	read_reg(fd,0x4e,&apbData3);
	apbData = (apbData3<<24) | (apbData2<<16) | (apbData1<<8) | (apbData0<<0);
	if((apbData & 0x01) != 0x01)
	CRITICAL_PRINT(MODULE_NAME, "Warning! Video is not available from the DP source yet. Disable frame reset command will not be applied!");
	else
	CRITICAL_PRINT(MODULE_NAME,"Disable Frame Reset");
	write_reg(fd,0x49,0x18);//Disable Frame Reset for VS0
	write_reg(fd,0x4a,0xa);
	write_reg(fd,0x4b,0x5);
	write_reg(fd,0x4c,0x0);
	write_reg(fd,0x4d,0x0);
	write_reg(fd,0x4e,0x0);
	
	write_reg(fd,0x49,0x18);//Disable Frame Reset for VS1
	write_reg(fd,0x4a,0xb);
	write_reg(fd,0x4b,0x5);
	write_reg(fd,0x4c,0x0);
	write_reg(fd,0x4d,0x0);
	write_reg(fd,0x4e,0x0);
	
	write_reg(fd,0x49,0x14);//Disable DTG for VS0
	write_reg(fd,0x4a,0xa);
	write_reg(fd,0x4b,0x0);
	write_reg(fd,0x4c,0x0);
	write_reg(fd,0x4d,0x0);
	write_reg(fd,0x4e,0x0);
	(void)usleep(40 * 1000);
	
	write_reg(fd, 0x49, 0x00);  // 0xA00 = 0 DIS_VS0
    write_reg(fd, 0x4a, 0xa);
    write_reg(fd, 0x4b, 0x0);
    write_reg(fd, 0x4c, 0x0);
    write_reg(fd, 0x4d, 0x0);
    write_reg(fd, 0x4e, 0x0);
	
	write_reg(fd,0x49,0x14);//Enable DTG for VS0
	write_reg(fd,0x4a,0xa);
	write_reg(fd,0x4b,0x1);
	write_reg(fd,0x4c,0x0);
	write_reg(fd,0x4d,0x0);
	write_reg(fd,0x4e,0x0);
	
	write_reg(fd, 0x49, 0x00);  // 0xA00 = 1 EN_VS0
    write_reg(fd, 0x4a, 0xa);
    write_reg(fd, 0x4b, 0x1);
    write_reg(fd, 0x4c, 0x0);
    write_reg(fd, 0x4d, 0x0);
    write_reg(fd, 0x4e, 0x0);
	
	write_reg(fd,0x49,0x14);//Disable DTG for VS1
	write_reg(fd,0x4a,0xb);
	write_reg(fd,0x4b,0x0);
	write_reg(fd,0x4c,0x0);
	write_reg(fd,0x4d,0x0);
	write_reg(fd,0x4e,0x0);
	(void)usleep(40 * 1000);
	
	write_reg(fd, 0x49, 0x00); // 0xB00 = 0 DIS_VS1
    write_reg(fd, 0x4a, 0xb);
    write_reg(fd, 0x4b, 0x0);
    write_reg(fd, 0x4c, 0x0);
    write_reg(fd, 0x4d, 0x0);
    write_reg(fd, 0x4e, 0x0);
	
	write_reg(fd,0x49,0x14);//Enable DTG for VS1
	write_reg(fd,0x4a,0xb);
	write_reg(fd,0x4b,0x1);
	write_reg(fd,0x4c,0x0);
	write_reg(fd,0x4d,0x0);
	write_reg(fd,0x4e,0x0);
	
	write_reg(fd, 0x49, 0x00); // 0xB00 = 1 EN_VS1
    write_reg(fd, 0x4a, 0xb);
    write_reg(fd, 0x4b, 0x1);
    write_reg(fd, 0x4c, 0x0);
    write_reg(fd, 0x4d, 0x0);
    write_reg(fd, 0x4e, 0x0);

#if ENABLE_VP1_PATTERN
    CRITICAL_PRINT(MODULE_NAME, "Enable VP1 Patgen", __func__);
    write_reg(fd, 0x40, 0x30);
    write_reg(fd, 0x41, 0x69);
    write_reg(fd, 0x42, 0x10);
    write_reg(fd, 0x41, 0x68);
    write_reg(fd, 0x42, 0x95);
#endif
    //==================================================
    // TODO: ENABLE VP need be called after video afailable and after video input reset
    CRITICAL_PRINT(MODULE_NAME, "Enable VPs");
    // main page: set number of video streams
    write_reg(fd, 0x43, 0x01);  //Set number of VPs used = 2
    write_reg(fd, 0x44, 0x03);  //Enable video processors

    CRITICAL_PRINT(MODULE_NAME, "Configure Serializer TX Link Layer");
    write_reg(fd, 0x40, 0x2C);  //Link layer Reg page
    write_reg(fd, 0x41, 0x01);  //Link layer stream enable
    write_reg(fd, 0x42, 0x03);  //Link layer stream enable
    write_reg(fd, 0x41, 0x06);
    write_reg(fd, 0x42, 0x29);  //set 35 time slots
    write_reg(fd, 0x41, 0x07);
    write_reg(fd, 0x42, 0x10);  //set 30 time slots
    write_reg(fd, 0x41, 0x20);  //Set Link layer vp bpp
    write_reg(fd, 0x42, 0x5a);  //Set Link layer vp bpp according to VP Bit per pixel
    write_reg(fd, 0x41, 0x00);  //Link layer enable
    write_reg(fd, 0x42, 0x03);  //Link layer enable

    return BRIDGECHIP_STATUS_SUCCESS;
}

/*------------------------------------------------------------------------------
 * serializer_video_input_reset
 * called after DP link training and video ready.
 *----------------------------------------------------------------------------*/
BridgeChip_StatusType serializer_video_input_reset(int32 fd)
{
    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    apb_write_reg(fd, APB_DP_RX, 0x54, 0x01);   //Video Input Reset (should be executed after DP video is available from the source);

    return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType link_set_deserializer_gpio(int32 fd)
{
   write_reg(fd, 0x40, 0x04);   // Page 1
   write_reg(fd, 0x41, 0x31);   // BC_GPIO1_SEL
   write_reg(fd, 0x42, 0x11);   // Forwarding BC daisy chain GPIO on DES0
  return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType link_set_deserializer_temp_optimization(int32 fd)
{
   BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    uint8 temp_final, rb;
    int32 temp_final_c, ramp_up_range_codes_needed, ramp_dn_range_codes_needed;
    int32 ramp_up_cap_delta, ramp_dn_cap_delta, ts_code_up, ts_code_dn, Efuse_TS_CODE;
    uint8 UNIQUEID_Reg0xC = 0;

    i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    write_reg(fd, 0x49, 0xc);
    write_reg(fd, 0x4a, 0x0);
    write_reg(fd, 0x48, 0x1b);
    read_reg(fd, 0x4b, &UNIQUEID_Reg0xC);
    if(UNIQUEID_Reg0xC != 0x12)
    {
        CRITICAL_PRINT(MODULE_NAME, "Override DES0 eFuse");
        write_reg(fd, 0xe, 0x3);  //Enable Write to P0 and P1
        write_reg(fd, 0x61, 0x0);
        write_reg(fd, 0x5a, 0x74);
        write_reg(fd, 0x5f, 0x4);
        write_reg(fd, 0x40, 0x3c);
        write_reg(fd, 0x41, 0xf5);
        write_reg(fd, 0x42, 0x21);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x43);
        write_reg(fd, 0x42, 0x3);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x43);
        write_reg(fd, 0x42, 0x3);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x5);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x5);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x6);
        write_reg(fd, 0x42, 0x1);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x6);
        write_reg(fd, 0x42, 0x1);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x37);
        write_reg(fd, 0x42, 0x32);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x37);
        write_reg(fd, 0x42, 0x32);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x8d);
        write_reg(fd, 0x42, 0xff);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x8d);
        write_reg(fd, 0x42, 0xff);
        write_reg(fd, 0x40, 0x5c);
        write_reg(fd, 0x41, 0x20);
        write_reg(fd, 0x42, 0x3c);
        write_reg(fd, 0x40, 0x5c);
        write_reg(fd, 0x41, 0xa0);
        write_reg(fd, 0x42, 0x3c);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x24);
        write_reg(fd, 0x42, 0x61);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x54);
        write_reg(fd, 0x42, 0x61);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x2c);
        write_reg(fd, 0x42, 0x19);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x5c);
        write_reg(fd, 0x42, 0x19);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x2e);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x5e);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x18);
        write_reg(fd, 0x42, 0x4b);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x38);
        write_reg(fd, 0x42, 0x4b);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x15);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x15);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x4a);
        write_reg(fd, 0x42, 0x1);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x4a);
        write_reg(fd, 0x42, 0x1);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xaa);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xaa);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xab);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xab);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xac);
        write_reg(fd, 0x42, 0x4c);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xac);
        write_reg(fd, 0x42, 0x4c);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xad);
        write_reg(fd, 0x42, 0x4c);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xad);
        write_reg(fd, 0x42, 0x4c);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xae);
        write_reg(fd, 0x42, 0xac);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xae);
        write_reg(fd, 0x42, 0xac);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xaf);
        write_reg(fd, 0x42, 0xac);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xaf);
        write_reg(fd, 0x42, 0xac);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x5);
        write_reg(fd, 0x42, 0xa);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x25);
        write_reg(fd, 0x42, 0xa);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x89);
        write_reg(fd, 0x42, 0x38);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x89);
        write_reg(fd, 0x42, 0x38);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x3c);
        write_reg(fd, 0x42, 0x41);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x1a);
        write_reg(fd, 0x42, 0x8);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x3a);
        write_reg(fd, 0x42, 0x8);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x6f);
        write_reg(fd, 0x42, 0x54);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x70);
        write_reg(fd, 0x42, 0x5);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x80);
        write_reg(fd, 0x42, 0x55);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x81);
        write_reg(fd, 0x42, 0x44);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x82);
        write_reg(fd, 0x42, 0x3);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x86);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x87);
        write_reg(fd, 0x42, 0x6);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x18);
        write_reg(fd, 0x42, 0x32);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x48);
        write_reg(fd, 0x42, 0x32);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x19);
        write_reg(fd, 0x42, 0xe);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x49);
        write_reg(fd, 0x42, 0xe);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x17);
        write_reg(fd, 0x42, 0x72);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x47);
        write_reg(fd, 0x42, 0x72);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x26);
        write_reg(fd, 0x42, 0x87);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x56);
        write_reg(fd, 0x42, 0x87);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x42);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x24);
        write_reg(fd, 0x41, 0x21);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x24);
        write_reg(fd, 0x41, 0x23);
        write_reg(fd, 0x42, 0x30);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x14);
        write_reg(fd, 0x42, 0x78);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x35);
        write_reg(fd, 0x42, 0x7e);
        write_reg(fd, 0x40, 0x6c);
        write_reg(fd, 0x41, 0xd);
        write_reg(fd, 0x42, 0x80);
        write_reg(fd, 0x40, 0x1c);
        write_reg(fd, 0x41, 0x8);
        write_reg(fd, 0x42, 0x13);
        write_reg(fd, 0x40, 0x1c);
        write_reg(fd, 0x41, 0x28);
        write_reg(fd, 0x42, 0x13);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x62);
        write_reg(fd, 0x42, 0x31);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x72);
        write_reg(fd, 0x42, 0x31);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x61);
        write_reg(fd, 0x42, 0x26);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x71);
        write_reg(fd, 0x42, 0x26);
        write_reg(fd, 0x40, 0x2c);
        write_reg(fd, 0x41, 0x56);
        write_reg(fd, 0x42, 0x3f);
        write_reg(fd, 0x40, 0x2c);
        write_reg(fd, 0x41, 0x58);
        write_reg(fd, 0x42, 0x3f);
        write_reg(fd, 0x1, 0x1);   //Soft Reset DES
        (void) usleep(40*100);
    }

    i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    CRITICAL_PRINT(MODULE_NAME, "Set up deserializer Temp Ramp Optimizations");
    // Read Deserializer 0 Temp
    serdes_wait_des_link_stable(fd, 40);
    write_reg(fd, 0x40, 0x6c);
    write_reg(fd, 0x41, 0xd);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0x13);
    read_reg(fd, 0x42,&temp_final);
    temp_final_c = 2*temp_final - 273;

    // Set up Deserializer 0 Temp Ramp Optimizations
    Efuse_TS_CODE = 2;
    ramp_up_range_codes_needed = (150-temp_final_c)/(190/11) + 1;
    ramp_dn_range_codes_needed = (temp_final_c-30)/(190/11) + 1;
    ramp_up_cap_delta = ramp_up_range_codes_needed - 4;
    ramp_dn_cap_delta = ramp_dn_range_codes_needed - 7;

    write_reg(fd, 0x40, 0x3c);
    write_reg(fd, 0x41, 0xf5);
    write_reg(fd, 0x42, (Efuse_TS_CODE<<4)+1); // Override TS_CODE Efuse Code
    if (ramp_up_cap_delta > 0)
    {
        ts_code_up = Efuse_TS_CODE - ramp_up_cap_delta;
        if (ts_code_up < 0)
            ts_code_up = 0;
        write_reg(fd, 0x40, 0x3c);
        write_reg(fd, 0x41, 0xf5);
        read_reg(fd, 0x42,&rb);
        rb &= 0x8F;
        rb |= (ts_code_up << 4);
        write_reg(fd, 0x42, (rb&0xff));

        read_reg(fd, 0x42,&rb);
        rb &= 0xFE;
        rb |= 0x01;
        write_reg(fd, 0x42, (rb&0xff));
        CRITICAL_PRINT(MODULE_NAME, "Reset DES0 988 -1------->");
        write_reg(fd, 0x1, 0x1);
        serdes_wait_des_link_stable(fd, 50);
        i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    }

    if (ramp_dn_cap_delta > 0)
    {
        ts_code_dn = Efuse_TS_CODE + ramp_dn_cap_delta;
        if (ts_code_dn >= 7)
            ts_code_dn = 7;
        write_reg(fd,0x40,0x3c);
        write_reg(fd, 0x41, 0xf5);
        read_reg(fd, 0x42,&rb);
        rb &= 0x8F;
        rb |= (ts_code_dn << 4);
        write_reg(fd, 0x42, (rb&0xff));

        read_reg(fd, 0x42,&rb);
        rb &= 0xFE;
        rb |= 0x01;
        write_reg(fd, 0x42, (rb&0xff));
        CRITICAL_PRINT(MODULE_NAME, "Reset DES0 988 -2------->");
        write_reg(fd, 0x1, 0x1);
        serdes_wait_des_link_stable(fd, 50);
        i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    }
    serdes_clear_link_crc_err_flag(fd);
    // Due to Ti CS3.0 chip relock issue this kind of behavior where the lock
    // comes up initially and then drops and toggles a few times before becoming
    // fully stable. The process normally takes < 30ms but maybe in a rare case
    // it is taking longer? On the final trim silicon a revised lock algorithm
    // is used that does not have this behavior of toggling many times and the
    // lock time is reduced to <10ms.
    //
    (void)usleep(50 * 1000);   //give 50 msec delay for stable lock
    if (serdes_wait_des_link_stable(fd, 50) == BRIDGECHIP_STATUS_FAILED)
    {
        /* We should not be entering here as per TI, safer side giving another 50msec in rare cases*/
        CRITICAL_PRINT(MODULE_NAME,"Wait des link stable failed");
        (void)usleep(50 * 1000);   //give another 50 msec delay for stable lock
    }
    i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    return eStatus;
}

/*------------------------------------------------------------------------------
 * link_set_deserializer_enable_oldi
 * DES I2C initialization.
 *----------------------------------------------------------------------------*/
static BridgeChip_StatusType link_set_deserializer_enable_oldi(int32 fd)
{
    i2c_set_slave_addr(fd, DES_ID_7BIT, I2C_ADDRFMT_7BIT);
    if (serdes_des_read_video_stream_error_status(fd, 0, 0, 1, 0) == BRIDGECHIP_STATUS_FAILED)
        CRITICAL_PRINT(MODULE_NAME, "Read Video Stream 0 error status failed");
    CRITICAL_PRINT(MODULE_NAME, "Hold Des DTG in reset");
    write_reg(fd, 0x40, 0x50);   //Select DTG Page
    write_reg(fd, 0x41, 0x32);
    write_reg(fd, 0x42, 0x06);   //Hold Port 0 DTG in reset
    write_reg(fd, 0x41, 0x62);
    write_reg(fd, 0x42, 0x06);   //Hold Port 1 DTG in reset
    //write_reg(fd, 0xDA, 0x00); //Override the OLDI Sync Mode

    CRITICAL_PRINT(MODULE_NAME, "Disable Stream Mapping");
    write_reg(fd, 0x0E, 0x03);   //Select both Output Ports
    write_reg(fd, 0xD0, 0x00);   //Disable FPD4 video forward to Output Port
    write_reg(fd, 0xD7, 0x00);   //Disable FPD3 video forward to Output Port

    CRITICAL_PRINT(MODULE_NAME, "Setup DTG for port 0");
    write_reg(fd, 0x40, 0x50);   //Select DTG Page
    write_reg(fd, 0x41, 0x20);
    write_reg(fd, 0x42, 0x63);   //DTG detect HS active high and VS active high
    write_reg(fd, 0x41, 0x29);   //Set Hstart
    write_reg(fd, 0x42, 0x80);   //Hstart upper byte
    write_reg(fd, 0x41, 0x2A);
    write_reg(fd, 0x42, 0xE8);   //Hstart lower byte
    write_reg(fd, 0x41, 0x2F);   //Set HSW
    write_reg(fd, 0x42, 0x40);   //HSW upper byte
    write_reg(fd, 0x41, 0x30);
    write_reg(fd, 0x42, 0x48);   //HSW lower byte

    CRITICAL_PRINT(MODULE_NAME, "Map video to display output");
    write_reg(fd, 0x0E, 0x03);   //Select both Output Ports
    write_reg(fd, 0xD0, 0x0c);   //Enable FPD_RX video forward to Output Port
    write_reg(fd, 0xD1, 0x0f);   //Stream 1 forwarded on DC
    write_reg(fd, 0xD6, 0x00);   //Send Stream 0 to Output Port 0 and Send Stream 0 to Output Port 1
    write_reg(fd, 0xD7, 0x00);   //FPD3 mapping disabled
    write_reg(fd, 0x0E, 0x01);   //Select Port 0

    CRITICAL_PRINT(MODULE_NAME, "Configure 988 Display");
    write_reg(fd, 0x40, 0x2C);     //Configure OLDI/RGB Port Settings
    write_reg(fd, 0x41, 0x0);
    write_reg(fd, 0x42, 0x37);
    write_reg(fd, 0x41, 0x1);
    write_reg(fd, 0x42, 0x37);
    write_reg(fd, 0x40, 0x2C);     //Configure OLDI/RGB PLL
    write_reg(fd, 0x41, 0x8);
    write_reg(fd, 0x42, 0x57);     //PLL_NUM23_16
    write_reg(fd, 0x41, 0x9);
    write_reg(fd, 0x42, 0x9b);     //PLL_NUM15_8
    write_reg(fd, 0x41, 0xA);
    write_reg(fd, 0x42, 0xc1);     //PLL_NUM7_0
    write_reg(fd, 0x41, 0xB);
    write_reg(fd, 0x42, 0xff);     //PLL_DEN23_16
    write_reg(fd, 0x41, 0xC);
    write_reg(fd, 0x42, 0xff);     //PLL_DEN15_8
    write_reg(fd, 0x41, 0xD);
    write_reg(fd, 0x42, 0xa5);     //PLL_DEN7_0
    write_reg(fd, 0x41, 0x18);
    write_reg(fd, 0x42, 0x23);     //PLL_NDIV
    write_reg(fd, 0x41, 0x2d);
    write_reg(fd, 0x42, 0x10);     //TX_SEL_CLKDIV

    CRITICAL_PRINT(MODULE_NAME, "Release Des DTG reset");
    write_reg(fd, 0x40, 0x50);   //Select DTG Page
    write_reg(fd, 0x41, 0x32);
    write_reg(fd, 0x42, 0x05);   //Release Des DTG reset
    write_reg(fd, 0x41, 0x62);
    write_reg(fd, 0x42, 0x05);   //Release Des DTG reset

    CRITICAL_PRINT(MODULE_NAME, "Enable oldi Output");
    write_reg(fd, 0x1, 0x40);      //OLDI Reset
    write_reg(fd, 0x40, 0x2c);     //Enable OLDI/RGB
    write_reg(fd, 0x41, 0x2);
    write_reg(fd, 0x42, 0x14);
    write_reg(fd, 0x41, 0x2);      //Toggle OLDI_SER_EN for Dual OLDI Mode
    write_reg(fd, 0x42, 0x4);
    write_reg(fd, 0x42, 0x14);
    write_reg(fd, 0x41, 0x20);     //P0 TX_EN
    write_reg(fd, 0x42, 0x80);
    write_reg(fd, 0x41, 0x22);     //P1 TX_EN
    write_reg(fd, 0x42, 0x80);
    
    CRITICAL_PRINT(MODULE_NAME, "Measure Des video timing-------");
    if (serdes_des_measure_video_dtg(fd, 0, NULL, 1) == BRIDGECHIP_STATUS_FAILED)
        CRITICAL_PRINT(MODULE_NAME, "Measure Des DTG0 video timing failed");
    if (serdes_des_measure_video_fpd_iv(fd, 1) == BRIDGECHIP_STATUS_FAILED)
        CRITICAL_PRINT(MODULE_NAME, "Measure Des Stream video failed");

    return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType link_set_deserializer_tx_fpd4(int32 fd)
{
   BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

   i2c_set_slave_addr(fd, DES_ID_7BIT, I2C_ADDRFMT_7BIT);
   write_reg(fd, 0x08, 0x20);    //Disable TX_BCC I2C master

   write_reg(fd, 0x40, 0x44);    //Select TX Link Layer Page
   write_reg(fd, 0x41, 0x1);     //Select LINK0_STREAM_EN
   write_reg(fd, 0x42, 0x2);     //Enable forwarded daisy chain streams
   write_reg(fd, 0x41, 0x7);     //Select LINK0_SLOT_REQ1
   write_reg(fd, 0x42, 0x41);    //Set number of time slots
   write_reg(fd, 0x41, 0x32);
   write_reg(fd, 0x42, 0x1);     //Set FPD_TX_POL_SEL_EN
   write_reg(fd, 0x41, 0x30);
   write_reg(fd, 0x42, 0x0);     //Configure FPD_TX_HPOL_CTL
   write_reg(fd, 0x41, 0x31);
   write_reg(fd, 0x42, 0x0);     //Configure FPD_TX_VPOL_CTL
   write_reg(fd, 0x41, 0x20);    //Set Link layer vp bpp
   write_reg(fd, 0x42, 0x5a);    //Set Link layer vp bpp according to VP Bit per pixel
   write_reg(fd, 0x41, 0x0);     //Link layer enable
   write_reg(fd, 0x42, 0x3);     //Link layer enable

   CRITICAL_PRINT(MODULE_NAME,"Program Des Daisy TX to FPD-Link IV mode");
   write_reg(fd, 0xa8, 0x28);    //Enable daisy chain TX_FPD_PORTS Single
   write_reg(fd, 0x5, 0xc0);     //Set half rate mode for 6.75Gbps daisy chain
   //write_reg(fd, 0x40, 0xc);   //Select PLL Reg page
   //write_reg(fd, 0x41, 0x5b);  //Select Channel 1 PLL Register
   //write_reg(fd, 0x42, 0x8);   //Disable Unused Channel 1 PLL
   write_reg(fd, 0x1, 0x30);     //soft reset Des
   (void)usleep(50 * 1000);
   serdes_wait_des_daisy_link_stable(fd, 50);
   return eStatus;
}

/*
static BridgeChip_StatusType link_set_deserializer_daisy_gpio(int32 fd)
{
    write_reg(fd, 0x40, 0x04);  //Page 1
    write_reg(fd, 0x41, 0x11);  //BC_GPIO1_SEL
    write_reg(fd, 0x42, 0x00);  //BC GPIO1 send local GPIO0

    return BRIDGECHIP_STATUS_SUCCESS;
}
*/

static BridgeChip_StatusType link_set_deserializer_daisy_temp_optimization(int32 fd)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    uint8 temp_final, rb;
    int32 temp_final_c, ramp_up_range_codes_needed, ramp_dn_range_codes_needed;
    int32 ramp_up_cap_delta, ramp_dn_cap_delta, ts_code_up, ts_code_dn, Efuse_TS_CODE;
    uint8 UNIQUEID_Reg0xC = 0;

    i2c_set_slave_addr(fd, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    write_reg(fd, 0x49, 0xc);
    write_reg(fd, 0x4a, 0x0);
    write_reg(fd, 0x48, 0x1b);
    read_reg(fd, 0x4b, &UNIQUEID_Reg0xC);
    if(UNIQUEID_Reg0xC != 0x12)
    {
        CRITICAL_PRINT(MODULE_NAME, "Override DES1 eFuse");
        write_reg(fd, 0xe, 0x3);  //Enable Write to P0 and P1
        write_reg(fd, 0x61, 0x0);
        write_reg(fd, 0x5a, 0x74);
        write_reg(fd, 0x5f, 0x4);
        write_reg(fd, 0x40, 0x3c);
        write_reg(fd, 0x41, 0xf5);
        write_reg(fd, 0x42, 0x21);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x43);
        write_reg(fd, 0x42, 0x3);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x43);
        write_reg(fd, 0x42, 0x3);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x5);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x5);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x6);
        write_reg(fd, 0x42, 0x1);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x6);
        write_reg(fd, 0x42, 0x1);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x37);
        write_reg(fd, 0x42, 0x32);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x37);
        write_reg(fd, 0x42, 0x32);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x8d);
        write_reg(fd, 0x42, 0xff);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x8d);
        write_reg(fd, 0x42, 0xff);
        write_reg(fd, 0x40, 0x5c);
        write_reg(fd, 0x41, 0x20);
        write_reg(fd, 0x42, 0x3c);
        write_reg(fd, 0x40, 0x5c);
        write_reg(fd, 0x41, 0xa0);
        write_reg(fd, 0x42, 0x3c);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x24);
        write_reg(fd, 0x42, 0x61);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x54);
        write_reg(fd, 0x42, 0x61);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x2c);
        write_reg(fd, 0x42, 0x19);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x5c);
        write_reg(fd, 0x42, 0x19);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x2e);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x5e);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x18);
        write_reg(fd, 0x42, 0x4b);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x38);
        write_reg(fd, 0x42, 0x4b);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x15);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x15);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x4a);
        write_reg(fd, 0x42, 0x1);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x4a);
        write_reg(fd, 0x42, 0x1);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xaa);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xaa);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xab);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xab);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xac);
        write_reg(fd, 0x42, 0x4c);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xac);
        write_reg(fd, 0x42, 0x4c);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xad);
        write_reg(fd, 0x42, 0x4c);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xad);
        write_reg(fd, 0x42, 0x4c);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xae);
        write_reg(fd, 0x42, 0xac);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xae);
        write_reg(fd, 0x42, 0xac);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0xaf);
        write_reg(fd, 0x42, 0xac);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0xaf);
        write_reg(fd, 0x42, 0xac);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x5);
        write_reg(fd, 0x42, 0xa);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x25);
        write_reg(fd, 0x42, 0xa);
        write_reg(fd, 0x40, 0x54);
        write_reg(fd, 0x41, 0x89);
        write_reg(fd, 0x42, 0x38);
        write_reg(fd, 0x40, 0x58);
        write_reg(fd, 0x41, 0x89);
        write_reg(fd, 0x42, 0x38);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x3c);
        write_reg(fd, 0x42, 0x41);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x1a);
        write_reg(fd, 0x42, 0x8);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x3a);
        write_reg(fd, 0x42, 0x8);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x6f);
        write_reg(fd, 0x42, 0x54);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x70);
        write_reg(fd, 0x42, 0x5);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x80);
        write_reg(fd, 0x42, 0x55);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x81);
        write_reg(fd, 0x42, 0x44);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x82);
        write_reg(fd, 0x42, 0x3);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x86);
        write_reg(fd, 0x42, 0x2c);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x87);
        write_reg(fd, 0x42, 0x6);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x18);
        write_reg(fd, 0x42, 0x32);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x48);
        write_reg(fd, 0x42, 0x32);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x19);
        write_reg(fd, 0x42, 0xe);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x49);
        write_reg(fd, 0x42, 0xe);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x17);
        write_reg(fd, 0x42, 0x72);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x47);
        write_reg(fd, 0x42, 0x72);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x26);
        write_reg(fd, 0x42, 0x87);
        write_reg(fd, 0x40, 0x38);
        write_reg(fd, 0x41, 0x56);
        write_reg(fd, 0x42, 0x87);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x42);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x24);
        write_reg(fd, 0x41, 0x21);
        write_reg(fd, 0x42, 0x0);
        write_reg(fd, 0x40, 0x24);
        write_reg(fd, 0x41, 0x23);
        write_reg(fd, 0x42, 0x30);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x14);
        write_reg(fd, 0x42, 0x78);
        write_reg(fd, 0x40, 0x10);
        write_reg(fd, 0x41, 0x35);
        write_reg(fd, 0x42, 0x7e);
        write_reg(fd, 0x40, 0x6c);
        write_reg(fd, 0x41, 0xd);
        write_reg(fd, 0x42, 0x80);
        write_reg(fd, 0x40, 0x1c);
        write_reg(fd, 0x41, 0x8);
        write_reg(fd, 0x42, 0x13);
        write_reg(fd, 0x40, 0x1c);
        write_reg(fd, 0x41, 0x28);
        write_reg(fd, 0x42, 0x13);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x62);
        write_reg(fd, 0x42, 0x31);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x72);
        write_reg(fd, 0x42, 0x31);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x61);
        write_reg(fd, 0x42, 0x26);
        write_reg(fd, 0x40, 0x14);
        write_reg(fd, 0x41, 0x71);
        write_reg(fd, 0x42, 0x26);
        write_reg(fd, 0x40, 0x2c);
        write_reg(fd, 0x41, 0x56);
        write_reg(fd, 0x42, 0x3f);
        write_reg(fd, 0x40, 0x2c);
        write_reg(fd, 0x41, 0x58);
        write_reg(fd, 0x42, 0x3f);
        write_reg(fd, 0x1, 0x1);  //Soft Reset DES
        (void) usleep(40*100);
    }

    i2c_set_slave_addr(fd, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    CRITICAL_PRINT(MODULE_NAME, "Set up deserializer Temp Ramp Optimizations");
    // Read Deserializer 0 Temp
    serdes_wait_des_daisy_link_stable(fd, 50);
    write_reg(fd, 0x40, 0x6c);
    write_reg(fd, 0x41, 0xd);
    write_reg(fd, 0x42, 0x0);
    write_reg(fd, 0x41, 0x13);
    read_reg(fd, 0x42,&temp_final);
    temp_final_c = 2*temp_final - 273;

    // Set up Deserializer 0 Temp Ramp Optimizations
    Efuse_TS_CODE = 2;
    ramp_up_range_codes_needed = (150-temp_final_c)/(190/11) + 1;
    ramp_dn_range_codes_needed = (temp_final_c-30)/(190/11) + 1;
    ramp_up_cap_delta = ramp_up_range_codes_needed - 4;
    ramp_dn_cap_delta = ramp_dn_range_codes_needed - 7;

    write_reg(fd, 0x40, 0x3c);
    write_reg(fd, 0x41, 0xf5);
    write_reg(fd, 0x42, (Efuse_TS_CODE<<4)+1);  //Override TS_CODE Efuse Code
    if (ramp_up_cap_delta > 0)
    {
        ts_code_up = Efuse_TS_CODE - ramp_up_cap_delta;
        if (ts_code_up < 0)
            ts_code_up = 0;
        write_reg(fd, 0x40, 0x3c);
        write_reg(fd, 0x41, 0xf5);
        read_reg(fd, 0x42,&rb);
        rb &= 0x8F;
        rb |= (ts_code_up << 4);
        write_reg(fd, 0x42, (rb&0xff));

        read_reg(fd, 0x42,&rb);
        rb &= 0xFE;
        rb |= 0x01;
        write_reg(fd, 0x42, (rb&0xff));
        CRITICAL_PRINT(MODULE_NAME, "Reset DES1 988 -1------->");
        write_reg(fd, 0x1, 0x1);
        serdes_wait_des_daisy_link_stable(fd, 50);
        i2c_set_slave_addr(fd, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    }

    if (ramp_dn_cap_delta > 0)
    {
        ts_code_dn = Efuse_TS_CODE + ramp_dn_cap_delta;
        if (ts_code_dn >= 7)
            ts_code_dn = 7;
        write_reg(fd,0x40,0x3c);
        write_reg(fd, 0x41, 0xf5);
        read_reg(fd, 0x42,&rb);
        rb &= 0x8F;
        rb |= (ts_code_dn << 4);
        write_reg(fd, 0x42, (rb&0xff));

        read_reg(fd, 0x42,&rb);
        rb &= 0xFE;
        rb |= 0x01;
        write_reg(fd, 0x42, (rb&0xff));
        CRITICAL_PRINT(MODULE_NAME, "Reset DES1 988 -2------->");
        write_reg(fd, 0x1, 0x1);
        serdes_wait_des_daisy_link_stable(fd, 50);
        i2c_set_slave_addr(fd, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    }
    serdes_clear_link_crc_err_flag(fd);
    // Due to Ti CS3.0 chip relock issue this kind of behavior where the lock
    // comes up initially and then drops and toggles a few times before becoming
    // fully stable. The process normally takes < 30ms but maybe in a rare case
    // it is taking longer? On the final trim silicon a revised lock algorithm
    // is used that does not have this behavior of toggling many times and the
    // lock time is reduced to <10ms.
    (void)usleep(50 * 1000);       //give 50 msec delay for stable lock
    if (serdes_wait_des_daisy_link_stable(fd, 50) == BRIDGECHIP_STATUS_FAILED)
    {
        /* We should not be entering here as per TI, safer side giving another 50msec in rare cases*/
        CRITICAL_PRINT(MODULE_NAME,"Wait des link stable failed");
        (void)usleep(50 * 1000);   //give another 50 msec delay for stable lock
    }
    return eStatus;
}

static BridgeChip_StatusType link_set_deserializer_daisy_enable_oldi(int32 fd)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    i2c_set_slave_addr(fd, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    if (serdes_des_read_video_stream_error_status(fd, 0, 0, 1, 0) == BRIDGECHIP_STATUS_FAILED)
        CRITICAL_PRINT(MODULE_NAME, "Read Video Stream 0 error status failed");
    CRITICAL_PRINT(MODULE_NAME, "Hold Des DTG in reset");
    write_reg(fd, 0x40, 0x50);  //Select DTG Page
    write_reg(fd, 0x41, 0x32);
    write_reg(fd, 0x42, 0x06);  //Hold Port 0 DTG in reset
    write_reg(fd, 0x41, 0x62);
    write_reg(fd, 0x42, 0x06);  //Hold Port 1 DTG in reset

    CRITICAL_PRINT(MODULE_NAME, "Disable Stream Mapping");
    write_reg(fd, 0x0E, 0x03);  //Select both Output Ports
    write_reg(fd, 0xD0, 0x00);  //Disable FPD4 video forward to Output Port
    write_reg(fd, 0xD7, 0x00);  //Disable FPD3 video forward to Output Port

    CRITICAL_PRINT(MODULE_NAME, "Setup DTG for port 0");
    write_reg(fd, 0x40, 0x50);  //Select DTG Page
    write_reg(fd, 0x41, 0x20);  //DTG_CTL_P0
    write_reg(fd, 0x42, 0x63);  //DTG detect HS active high and VS active high
    write_reg(fd, 0x41, 0x29);  //Set Hstart
    write_reg(fd, 0x42, 0x80);  //Hstart upper byte
    write_reg(fd, 0x41, 0x2A);
    write_reg(fd, 0x42, 0x64);  //Hstart lower byte
    write_reg(fd, 0x41, 0x2F);  //Set HSW
    write_reg(fd, 0x42, 0x40);  //HSW upper byte
    write_reg(fd, 0x41, 0x30);
    write_reg(fd, 0x42, 0x2c);  //HSW lower byte

    CRITICAL_PRINT(MODULE_NAME, "Map video to display output");
    write_reg(fd, 0x0E, 0x03);  //Select both Output Ports
    write_reg(fd, 0xD0, 0x0C);  //Enable FPD_RX video forward to Output Port
    write_reg(fd, 0xD1, 0x0F);  //Enable FPD_RX video forward to Output Port
    write_reg(fd, 0xD6, 0x09);  //Send Stream 1 to Output Port 1 and Send Stream 1 to Output Port 0
    write_reg(fd, 0xD7, 0x00);  //FPD3 mapping disabled
    write_reg(fd, 0x0E, 0x01);  //Select Port 0

    CRITICAL_PRINT(MODULE_NAME, "Configure 988 Display");
    write_reg(fd, 0x40, 0x2C);  //Configure OLDI/RGB Port Settings
    write_reg(fd, 0x41, 0x0);
    write_reg(fd, 0x42, 0x37);
    write_reg(fd, 0x41, 0x1);
    write_reg(fd, 0x42, 0x37);
    write_reg(fd, 0x40, 0x2C);  //Configure OLDI/RGB PLL
    write_reg(fd, 0x41, 0x8);
    write_reg(fd, 0x42, 0x32);  //PLL_NUM23_16
    write_reg(fd, 0x41, 0x9);
    write_reg(fd, 0x42, 0xf);   //PLL_NUM15_8
    write_reg(fd, 0x41, 0xA);
    write_reg(fd, 0x42, 0xdc);  //PLL_NUM7_0
    write_reg(fd, 0x41, 0xB);
    write_reg(fd, 0x42, 0xff);  //PLL_DEN23_16
    write_reg(fd, 0x41, 0xC);
    write_reg(fd, 0x42, 0xff);  //PLL_DEN15_8
    write_reg(fd, 0x41, 0xD);
    write_reg(fd, 0x42, 0xa5);  //PLL_DEN7_0
    write_reg(fd, 0x41, 0x18);
    write_reg(fd, 0x42, 0x36);  //PLL_NDIV
    write_reg(fd, 0x41, 0x2d);
    write_reg(fd, 0x42, 0x12);  //TX_SEL_CLKDIV

    CRITICAL_PRINT(MODULE_NAME, "Release Des DTG reset");
    write_reg(fd, 0x40, 0x50);  //Select DTG Page
    write_reg(fd, 0x41, 0x32);
    write_reg(fd, 0x42, 0x05);  //Release Des DTG reset
    write_reg(fd, 0x41, 0x62);
    write_reg(fd, 0x42, 0x05);  //Release Des DTG reset

    CRITICAL_PRINT(MODULE_NAME,"Enable OLDI output");
    write_reg(fd, 0x1, 0x40);   //OLDI Reset
    write_reg(fd, 0x40, 0x2c);  //Enable OLDI/RGB
    write_reg(fd, 0x41, 0x2);
    write_reg(fd, 0x42, 0x14);
    write_reg(fd, 0x41, 0x2);   //Toggle OLDI_SER_EN for Dual OLDI Mode
    write_reg(fd, 0x42, 0x4);
    write_reg(fd, 0x42, 0x14);
    write_reg(fd, 0x41, 0x20);  //P0 TX_EN
    write_reg(fd, 0x42, 0x80);
    write_reg(fd, 0x41, 0x22);  //P1 TX_EN
    write_reg(fd, 0x42, 0x80); 
    
    CRITICAL_PRINT(MODULE_NAME, "Measure Des Daisy video timing-------");
    if (serdes_des_measure_video_dtg(fd, 0, NULL, 1) == BRIDGECHIP_STATUS_FAILED)
        CRITICAL_PRINT(MODULE_NAME, "Measure Des DTG0 video timing failed");
    if (serdes_des_measure_video_fpd_iv(fd, 1) == BRIDGECHIP_STATUS_FAILED)
        CRITICAL_PRINT(MODULE_NAME, "Measure Des Stream video failed");
    return eStatus;
}

BridgeChip_StatusType serdes_check_983_dprx(int32 fd)
{
    uint32 lane01_status, lane23_status, h_res, v_res;
    static uint8  vp_status = 0, vp_status_tmp = -1, status_change = 0;
    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    (void) apb_read_reg(fd, APB_DP_RX, 0x43C, &lane01_status);
    (void) apb_read_reg(fd, APB_DP_RX, 0x440, &lane23_status);
    (void) apb_read_reg(fd, APB_DP_RX, 0x500, &h_res);
    (void) apb_read_reg(fd, APB_DP_RX, 0x514, &v_res);
    read_reg(fd, 0x45, &vp_status);

    if (vp_status != vp_status_tmp)
    {
        CRITICAL_PRINT(MODULE_NAME, "VP STATUS: | VP0:%d | VP1:%d | VP2:%d | VP3:%d |",
                vp_status & 0x1, (vp_status >> 1) & 0x01, (vp_status >> 2) & 0x01, (vp_status >> 3) & 0x01);
        serdes_get_983_dp_rx_status(fd);
        vp_status_tmp = vp_status;
    }
    if (((lane01_status&0x3ff) == 0x377) && ((lane23_status&0x3ff) == 0x377) && (h_res == AHW) && v_res == AVW)
    {
        if (status_change == 0)
        {
            CRITICAL_PRINT(MODULE_NAME, "DP video source is ready");
            CRITICAL_PRINT(MODULE_NAME, "983 Status: VP0:%d | VP1:%d | lane_status: 0x%x, 0x%x,  WxH = %d x %d",
                    vp_status & 0x1, (vp_status >> 1) & 0x01, lane01_status, lane23_status, h_res, v_res);
        }
        status_change = 1;
        return BRIDGECHIP_STATUS_SUCCESS;
    }
    else
    {
        if( (lane01_status == 0x30044) || (lane23_status == 0x30044) )
        {
            CRITICAL_PRINT(MODULE_NAME, "DP link is untrained 0x30044 and video is not ready...");
            return BRIDGECHIP_EXIT_FROM_DPLINK_UNTRAIN;
        }
        if (status_change == 1)
        {
            CRITICAL_PRINT(MODULE_NAME, "DP video source is not ready");
            CRITICAL_PRINT(MODULE_NAME, "983 Status: VP0:%d | VP1:%d | lane_status: 0x%x, 0x%x,  WxH = %d x %d",
                    vp_status & 0x1, (vp_status >> 1) & 0x01, lane01_status, lane23_status, h_res, v_res);
        }
        status_change = 0;
        return BRIDGECHIP_STATUS_FAILED;
    }
}

/*------------------------------------------------------------------------------
 * serdes_monitor_983_dp_link
 * check the SoC
 *----------------------------------------------------------------------------*/
BridgeChip_StatusType serdes_monitor_983_dp_link(int32 fd)
{
    uint32 lane01_status, lane23_status, h_res, v_res;
    static uint8  vp_status = 0;
    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    (void) apb_read_reg(fd, APB_DP_RX, 0x43C, &lane01_status);
    (void) apb_read_reg(fd, APB_DP_RX, 0x440, &lane23_status);
    (void) apb_read_reg(fd, APB_DP_RX, 0x500, &h_res);
    (void) apb_read_reg(fd, APB_DP_RX, 0x514, &v_res);
    read_reg(fd, 0x45, &vp_status);

    if (((lane01_status&0x3ff) != 0x377) && ((lane23_status&0x3ff) != 0x377) && (h_res != AHW) && (v_res != AVW))
    {
        CRITICAL_PRINT(MODULE_NAME, "FATAL: | VP0:%d | VP1:%d | lane_status: 0x%x, 0x%x,  WxH = %d x %d, Screen is BLACK",
                vp_status & 0x1, (vp_status >> 1) & 0x01, lane01_status, lane23_status, h_res, v_res);

        return BRIDGECHIP_STATUS_FAILED;
    }
    else
    {
        // Diagnostic DP Link when get any of incorrect value
        if (((lane01_status&0x3ff) != 0x377) || ((lane23_status&0x3ff) != 0x377) || (h_res != AHW) || (v_res != AVW))
        {
            CRITICAL_PRINT(MODULE_NAME, "983 DIAG: VP0:%d | VP1:%d | lane_status: 0x%x, 0x%x,  WxH = %d x %d",
                    vp_status & 0x1, (vp_status >> 1) & 0x01, lane01_status, lane23_status, h_res, v_res);
            serdes_diagnostic_983dprx(fd);
        }
        return BRIDGECHIP_STATUS_SUCCESS;
    }
}
  
void serdes_get_983_dp_rx_status(int32 fd)
{
    uint32 dp_rate, lanes;
    uint32 h_res, h_pol, h_sync, h_back, h_total;
    uint32 v_res, v_pol, v_sync, v_back, v_total;
    uint32 lane01_status, lane23_status;
    uint32 mst_enable;
    uint8  vp_status;

    (void) apb_read_reg(fd, APB_DP_RX, 0x400, &dp_rate);
    dp_rate = dp_rate * 27;
    (void) apb_read_reg(fd, APB_DP_RX, 0x404, &lanes);
    (void) apb_read_reg(fd, APB_DP_RX, 0x43C, &lane01_status);
    (void) apb_read_reg(fd, APB_DP_RX, 0x440, &lane23_status);
    (void) apb_read_reg(fd, APB_DP_RX, 0x48C, &mst_enable);
    (void) apb_read_reg(fd, APB_DP_RX, 0x500, &h_res);
    (void) apb_read_reg(fd, APB_DP_RX, 0x504, &h_pol);
    (void) apb_read_reg(fd, APB_DP_RX, 0x508, &h_sync);
    (void) apb_read_reg(fd, APB_DP_RX, 0x50C, &h_back);
    h_back = h_back - h_sync;
    (void) apb_read_reg(fd, APB_DP_RX, 0x510, &h_total);
    (void) apb_read_reg(fd, APB_DP_RX, 0x514, &v_res);
    (void) apb_read_reg(fd, APB_DP_RX, 0x518, &v_pol);
    (void) apb_read_reg(fd, APB_DP_RX, 0x51C, &v_sync);
    (void) apb_read_reg(fd, APB_DP_RX, 0x520, &v_back);
    v_back = v_back - v_sync;
    (void) apb_read_reg(fd, APB_DP_RX, 0x524, &v_total);
    read_reg(fd, 0x45, &vp_status);
    CRITICAL_PRINT(MODULE_NAME, " ");
    CRITICAL_PRINT(MODULE_NAME, " ");
    CRITICAL_PRINT(MODULE_NAME, "===================================================");
    CRITICAL_PRINT(MODULE_NAME, "DP Lock Status");
    CRITICAL_PRINT(MODULE_NAME, "DP Rate: %d", dp_rate);
    CRITICAL_PRINT(MODULE_NAME, "Lanes: %d", lanes);
    CRITICAL_PRINT(MODULE_NAME, "Lane01 Status: 0x%x", lane01_status);
    CRITICAL_PRINT(MODULE_NAME, "Lane23 Status: 0x%x", lane23_status);
    CRITICAL_PRINT(MODULE_NAME, "MST Enable: 0x%x", mst_enable);
    CRITICAL_PRINT(MODULE_NAME, "H RES: %d", h_res);
    CRITICAL_PRINT(MODULE_NAME, "H POL: %d", h_pol);
    CRITICAL_PRINT(MODULE_NAME, "H SYNC WIDTH: %d", h_sync);
    CRITICAL_PRINT(MODULE_NAME, "H BACK PORCH: %d", h_back);
    CRITICAL_PRINT(MODULE_NAME, "H TOTAL: %d", h_total);
    CRITICAL_PRINT(MODULE_NAME, "v RES: %d", v_res);
    CRITICAL_PRINT(MODULE_NAME, "V POL: %d", v_pol);
    CRITICAL_PRINT(MODULE_NAME, "V SYNC WIDTH: %d", v_sync);
    CRITICAL_PRINT(MODULE_NAME, "V BACK PORCH: %d", v_back);
    CRITICAL_PRINT(MODULE_NAME, "V TOTAL: %d", v_total);
    CRITICAL_PRINT(MODULE_NAME, "===================================================");
    CRITICAL_PRINT(MODULE_NAME, "VP STATUS: | VP0:%d | VP1:%d | VP2:%d | VP3:%d |",
            vp_status & 0x1, (vp_status >> 1) & 0x01, (vp_status >> 2) & 0x01, (vp_status >> 3) & 0x01);
    CRITICAL_PRINT(MODULE_NAME, "===================================================");
    CRITICAL_PRINT(MODULE_NAME, " ");
    CRITICAL_PRINT(MODULE_NAME, " ");
}

static void serdes_daisy_clear_link_crc_err_flag(int32 fd)
{
    uint8 reg_cfg5 = 0;

    //CRITICAL_PRINT(MODULE_NAME, "Clear BC CRC Flags");
    i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);

    if (read_reg(fd, 0x05, &reg_cfg5) == 0)
    {
        reg_cfg5 |= 0x20;
        //CRITICAL_PRINT(MODULE_NAME, "Do Daisy CRC ERROR RESET");
        write_reg(fd, 0x05, reg_cfg5);
    }
    if (read_reg(fd, 0x05, &reg_cfg5) == 0)
    {
        reg_cfg5 &= 0xDF;
        write_reg(fd, 0x05, reg_cfg5);
    }
}

/*------------------------------------------------------------------------------
 * serdes_ser_get_crc_err_count
 * get  crc err count
 *----------------------------------------------------------------------------*/
static void serdes_ser_log_crc_err_count(int32 fd)
{
    uint8 bc_crc_count_lsb = 0;
    uint8 bc_crc_count_msb = 0;
    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    write_reg(fd, 0x2D, 0x01);
    read_reg(fd, 0x0a, &bc_crc_count_lsb);
    read_reg(fd, 0x0b, &bc_crc_count_msb);
    CRITICAL_PRINT(MODULE_NAME, "port0 bc_crc_count:0x%x", ((bc_crc_count_msb<<8) | bc_crc_count_lsb));
    write_reg(fd, 0x2D, 0x12);
    read_reg(fd, 0x0a, &bc_crc_count_lsb);
    read_reg(fd, 0x0b, &bc_crc_count_msb);
    CRITICAL_PRINT(MODULE_NAME, "port1 bc_crc_count:0x%x", ((bc_crc_count_msb<<8) | bc_crc_count_lsb));
    write_reg(fd, 0x2D, 0x01);
}

BridgeChip_StatusType serdes_check_vp_status(int32 fd)
{
    uint8 vp0_sts, vp1_sts;
    uint8 reg_tmp = 0;


    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);

    write_reg(fd, 0x40, 0x31);
    write_reg(fd, 0x41, 0x30);
    // read failed, return bad param
    if (read_reg(fd, 0x42, &vp0_sts) != 0)
        return BRIDGECHIP_STATUS_BAD_PARAMS;

    write_reg(fd, 0x41, 0x70);
    // read failed, return bad param
    if (read_reg(fd, 0x42, &vp1_sts) != 0)
        return BRIDGECHIP_STATUS_BAD_PARAMS;

    if (((vp0_sts & 0x01) == 0) || ((vp1_sts & 0x01) == 0))
    {
        CRITICAL_PRINT(MODULE_NAME, "VPs not synchronized....");
        CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x30 = 0x%02x", vp0_sts);
        write_reg(fd, 0x41, 0x31);
        read_reg(fd, 0x42, &reg_tmp);
        CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x31 = 0x%02x", reg_tmp);
        write_reg(fd, 0x41, 0x35);
        read_reg(fd, 0x42, &reg_tmp);
        CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x35 = 0x%02x", reg_tmp);
        write_reg(fd, 0x41, 0x36);
        read_reg(fd, 0x42, &reg_tmp);
        CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x36 = 0x%02x", reg_tmp);

        CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x70 = 0x%02x", vp1_sts);
        write_reg(fd, 0x41, 0x71);
        read_reg(fd, 0x42, &reg_tmp);
        CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x71 = 0x%02x", reg_tmp);
        write_reg(fd, 0x41, 0x75);
        read_reg(fd, 0x42, &reg_tmp);
        CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x75 = 0x%02x", reg_tmp);
        write_reg(fd, 0x41, 0x76);
        read_reg(fd, 0x42, &reg_tmp);
        CRITICAL_PRINT(MODULE_NAME, "Page 12: 0x76 = 0x%02x", reg_tmp);

        return BRIDGECHIP_STATUS_FAILED;
    }
    else
    {
      
    }

    return BRIDGECHIP_STATUS_SUCCESS;
}

/*------------------------------------------------------------------------------
 * serdes_monitor_des_video
 * monitor function for check the 984 DTG video timing and FIFO status
 * call hanlding if the FIFO overflow happens
 *----------------------------------------------------------------------------*/
BridgeChip_StatusType serdes_monitor_des_video(int32 fd, bool32 print)
{
    struct video_timing dtg0_timing;
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    i2c_set_slave_addr(fd, DES_ID_7BIT, I2C_ADDRFMT_7BIT);
    if (serdes_des_measure_video_dtg(fd, 0, &dtg0_timing, 0) == BRIDGECHIP_STATUS_SUCCESS)
    {
        if (check_mesaured_video(&dtg0_timing) == BRIDGECHIP_STATUS_SUCCESS)
        {
            if (print == TRUE)
            {
                CRITICAL_PRINT(MODULE_NAME," Measured DTG Video Timing Success");
                CRITICAL_PRINT(MODULE_NAME," DTG0: htotal, vtotal, hres, vres, hstart, vstart, hswidth, vswidth");
                CRITICAL_PRINT(MODULE_NAME,"      %d  %d  %d  %d  %d  %d  %d  %d",
                        dtg0_timing.htotal,dtg0_timing.vtotal, dtg0_timing.hres,dtg0_timing.vres,dtg0_timing.hstart,dtg0_timing.vstart, dtg0_timing.hswidth, dtg0_timing.vswidth);
            }
            eStatus = BRIDGECHIP_STATUS_SUCCESS;
        }
        else
        {
            CRITICAL_PRINT(MODULE_NAME," *** Bad Video Timing Port0 (htotal %d, vtotal %d, hres %d, vres %d)",
                    dtg0_timing.htotal, dtg0_timing.vtotal, dtg0_timing.hres, dtg0_timing.vres);
            serdes_des_read_video_stream_error_status(fd, 0, 0, 0, 1);

            eStatus = BRIDGECHIP_STATUS_FAILED;
        }
    }
    else
    {
        CRITICAL_PRINT(MODULE_NAME," Measured DTG Video Timing Failed!!!");
    }

    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);

    return eStatus;
}


BridgeChip_StatusType serdes_monitor_des_video_daisy(int32 fd, bool32 print)
{
    struct video_timing dtg0_timing;
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    i2c_set_slave_addr(fd, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    if (serdes_des_measure_video_dtg(fd, 0, &dtg0_timing, 0) == BRIDGECHIP_STATUS_SUCCESS)
    {
        if (check_mesaured_video_daisy(&dtg0_timing) == BRIDGECHIP_STATUS_SUCCESS)
        {
            if (print == TRUE)
            {
                CRITICAL_PRINT(MODULE_NAME," Measured DTG Video Timing Success");
                CRITICAL_PRINT(MODULE_NAME," DTG0: htotal, vtotal, hres, vres, hstart, vstart, hswidth, vswidth");
                CRITICAL_PRINT(MODULE_NAME,"      %d  %d  %d  %d  %d  %d  %d  %d",
                        dtg0_timing.htotal,dtg0_timing.vtotal, dtg0_timing.hres,dtg0_timing.vres,dtg0_timing.hstart,dtg0_timing.vstart, dtg0_timing.hswidth, dtg0_timing.vswidth);
            }
            eStatus = BRIDGECHIP_STATUS_SUCCESS;
        }
        else
        {
            CRITICAL_PRINT(MODULE_NAME," *** Bad Video Timing Port0 (htotal %d, vtotal %d, hres %d, vres %d)",
                    dtg0_timing.htotal, dtg0_timing.vtotal, dtg0_timing.hres, dtg0_timing.vres);
            serdes_des_read_video_stream_error_status(fd, 0, 0, 0, 1);

            eStatus = BRIDGECHIP_STATUS_FAILED;
        }
    }
    else
    {
        CRITICAL_PRINT(MODULE_NAME," Measured DTG Video Timing Failed!!!");
    }
    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);

    return eStatus;
}
/*------------------------------------------------------------------------------
 * serdes_get_ser_link_status
 * judge the Ser-Des link status: linked or lost
 *----------------------------------------------------------------------------*/
BridgeChip_StatusType serdes_get_ser_link_status(int32 fd, int32 print)
{
    uint8 regval = 0;
    int32 port0_linked = 0;
    uint8 sts_reg_bc_crc_error = 0;
    uint8 sts_reg_lost_flag = 0;

    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);

    write_reg(fd, 0x2D, 0x01);
    if (read_reg(fd, 0x0C, &regval) != 0)
      return BRIDGECHIP_STATUS_BAD_PARAMS;    // return bad param if i2c read error

    //GENERAL_STS_MASK=0x51 : bit 0,4,6
    //0xc = 0x41 which bit 0 and 6 equal 1 means the link is ok.
    //bit 4 equal 1 means the link lost
    //any bit of 0, 6 equal 0 means link lost
    if ((regval & GENERAL_STS_MASK) == ((1 << LINK_DETECT_BIT) | (1 << RX_LOCK_DETECT_BIT)))
    {
        port0_linked = 1;
    }
    else
    {
        if (print == 1)
            CRITICAL_PRINT(MODULE_NAME, "port0_0xc:0x%x bit1:%x, bit4:%x", regval, ((regval>>1)&0x01), ((regval>>4)&0x01));
    }

    sts_reg_bc_crc_error = ((regval & GENERAL_STS_MASK_BIT1) >> 1);  //bit1 : BC_CRC_ERROR
    sts_reg_lost_flag = ((regval & GENERAL_STS_MASK_BIT4) >> 4);     //bit4 : link lost flag

    // clear bc_ecc and logging it
    if ((sts_reg_lost_flag == 1) || (sts_reg_bc_crc_error == 1) || (port0_linked == 0) )
    {
        serdes_ser_log_crc_err_count(fd);    //log
        serdes_clear_link_crc_err_flag(fd);  //clear
        serdes_ser_log_crc_err_count(fd);
    }
    if (port0_linked == 1) //&& (port1_linked == 1))
    {
        return BRIDGECHIP_STATUS_SUCCESS;
    }
    else
    {
        return BRIDGECHIP_STATUS_FAILED;
    }
}

BridgeChip_StatusType serdes_get_daisy_link_status(int32 fd)
{
    uint8 regval = 0;

   i2c_set_slave_addr(fd, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);

    if (read_reg(fd, 0x53, &regval) != 0)
        return BRIDGECHIP_STATUS_BAD_PARAMS;

    /*Either Bit 6 as 0 , Bit 5 as 1 indicates FPD link is Lost*/
    if(!(regval & (1 << DAISY_RX_LOCK_DET_P0)) || (regval & (1 << DAISY_BC_LINK_LOST_FLAG_P0)))
    {
        CRITICAL_PRINT(MODULE_NAME, "Value of reg 0x53 is: 0x%x",regval);
        serdes_daisy_clear_link_crc_err_flag(fd);
        return BRIDGECHIP_STATUS_FAILED;
    }
    else
    {
      return BRIDGECHIP_STATUS_SUCCESS;
    }
}
static BridgeChip_StatusType serdes_get_bridge_name(int32 fd, uint8 *chip_name)
{
    uint8 regval = 0;
    int32 i = 0;
    uint8 offset = 0xF1;
    uint8 reg = 0;
    chip_name[0] = 'D';
    chip_name[1] = 'S';
    chip_name[2] = '9';
    chip_name[3] = '0';
    for (i = 0; i < 5; i++)
    {
        reg = offset + i;
        if (read_reg(fd, reg, &regval))
        {
            return BRIDGECHIP_STATUS_FAILED;
        }
        chip_name[4 + i] = regval;
    }
    chip_name[9] = 0;
    return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType serdes_get_serializer_info(int32 fd)
{
    uint8 chip_name[10] = {0};
    uint8 revision[10] = {0};
    uint8 rev_id = 0;
    serdes_get_bridge_name(fd, chip_name);
    read_reg(fd, 0x30, &rev_id);
    rev_id = (rev_id >> 4) & 0x0F;
    switch(rev_id)
    {
    case 0:
        strncpy((char *)revision, "ES1.0", 5);
        break;
    case 1:
        strncpy((char *)revision, "ES2.0", 5);
        break;
    case 2:
        strncpy((char *)revision, "ProA0", 5);
        break;
    case 5:
        strncpy((char *)revision, "CS2.0", 5);
        break;
    default:
        strncpy((char *)revision, "Unknown", 7);
        break;
    }
    CRITICAL_PRINT(MODULE_NAME,"Serializer name is: %s Rev: %s", chip_name, revision);
    return BRIDGECHIP_STATUS_SUCCESS;
}

static BridgeChip_StatusType serdes_get_deserializer_info(int32 fd)
{
    uint8 chip_name[10] = {0};
    uint8 revision[10] = {0};
    uint8 rev_id = 0;
    serdes_get_bridge_name(fd, chip_name);
    read_reg(fd, 0x30, &rev_id);
    rev_id = (rev_id >> 4) & 0x0F;
    switch(rev_id)
    {
    case 1:
        strncpy((char *)revision, "CS1.0", 5);
        break;
    case 3:
        strncpy((char *)revision, "CS2.0", 5);
        break;
    case 4:
        strncpy((char *)revision, "CS3.0", 5);
        break;
    default:
        strncpy((char *)revision, "Unknown", 7);
        break;
    }
    CRITICAL_PRINT(MODULE_NAME,"De-serializer name is: %s Rev: %s", chip_name, revision);
    return BRIDGECHIP_STATUS_SUCCESS;
}

BridgeChip_StatusType ser_config_vp(int32 fd)
{
    i2c_set_slave_addr(fd, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    link_set_serializer_vp(fd);

    return BRIDGECHIP_STATUS_SUCCESS;
}

BridgeChip_StatusType ser_config_update(int32 i2c_fh)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    CRITICAL_PRINT(MODULE_NAME, "Link setup: Serializer FPD-Link IV mode. Build Time=%s, %s", __DATE__, __TIME__);
    i2c_set_slave_addr(i2c_fh, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    write_reg(i2c_fh, 0x01, SER_RESET_DIGITAL_ALL);
    (void)usleep(20*1000);
    eStatus |= serializer_i2c_config(i2c_fh);
    serdes_get_serializer_info(i2c_fh);

    eStatus |= link_set_serializer_i2c_pass_through(i2c_fh);
    eStatus |= link_set_serializer_gpio(i2c_fh);

    eStatus |= link_set_serializer_fpd4(i2c_fh);
    eStatus |= link_set_serializer_dprx(i2c_fh);

    i2c_set_slave_addr(i2c_fh, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    serdes_clear_link_crc_err_flag(i2c_fh);

    return eStatus;
}

BridgeChip_StatusType dser_config_update(int32 i2c_fh)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    CRITICAL_PRINT(MODULE_NAME, "Link setup: Deserializer FPD-Link IV mode. Build Time=%s, %s", __DATE__, __TIME__);
    i2c_set_slave_addr(i2c_fh, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    CRITICAL_PRINT(MODULE_NAME, "Configure Deserializer 0");
    serdes_get_deserializer_info(i2c_fh);
    eStatus |= deserializer_disable_i2c_remote_slaves(i2c_fh);

    //Set DES0 I2C speed to 1MHz
    write_reg(i2c_fh, 0x2B, 0x08);
    write_reg(i2c_fh, 0x2C, 0x08);

    eStatus |= link_set_deserializer_temp_optimization(i2c_fh);
    i2c_set_slave_addr(i2c_fh, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    eStatus |= link_set_deserializer_gpio(i2c_fh);
    eStatus |= link_set_deserializer_tx_fpd4(i2c_fh); //We need it for AUX

    CRITICAL_PRINT(MODULE_NAME, "Enable Deserializer oLDI");
    eStatus |= link_set_deserializer_enable_oldi(i2c_fh);
    (void)usleep(50 * 1000);

    CRITICAL_PRINT(MODULE_NAME, "----------Measure Timing from Deserializer 0---------");
    serdes_des_measure_video_dtg(i2c_fh, 0, NULL, 1);
    serdes_des_measure_video_fpd_iv(i2c_fh, 1);
    i2c_set_slave_addr(i2c_fh, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    serdes_daisy_clear_link_crc_err_flag(i2c_fh);

    return eStatus;
}
BridgeChip_StatusType ser_reset_chip(int32 fd)
{
    write_reg(fd, 0x01, SERDES_RESET_DIGITAL_ALL);
    return BRIDGECHIP_STATUS_SUCCESS;
}
BridgeChip_StatusType dser_config_update_recovery(int32 i2c_fh)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;
    CRITICAL_PRINT(MODULE_NAME, "DES 988 reinitialization for recovery: Software reset 988");
    i2c_set_slave_addr(i2c_fh, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    dser_getclear_0x54(i2c_fh, 1); //log
    write_reg(i2c_fh, 0x01, 0x01);
    serdes_wait_des_link_stable(i2c_fh, 50);

    CRITICAL_PRINT(MODULE_NAME, "Configure Deserializer 0");
    serdes_get_deserializer_info(i2c_fh);
    eStatus |= deserializer_disable_i2c_remote_slaves(i2c_fh);

    eStatus |= link_set_deserializer_temp_optimization(i2c_fh);
    i2c_set_slave_addr(i2c_fh, DES_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    eStatus |= link_set_deserializer_gpio(i2c_fh);
    eStatus |= link_set_deserializer_tx_fpd4(i2c_fh);

    CRITICAL_PRINT(MODULE_NAME, "Enable Deserializer oLDI");
    eStatus |= link_set_deserializer_enable_oldi(i2c_fh);
    i2c_set_slave_addr(i2c_fh, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    serdes_daisy_clear_link_crc_err_flag(i2c_fh);

    return eStatus;
}

BridgeChip_StatusType dser_daisy_config_update(int32 i2c_fh)
{
    BridgeChip_StatusType eStatus = BRIDGECHIP_STATUS_SUCCESS;

    CRITICAL_PRINT(MODULE_NAME, "Configure Deserializer 1");
    i2c_set_slave_addr(i2c_fh, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    serdes_get_deserializer_info(i2c_fh);
    eStatus |= deserializer_disable_i2c_remote_slaves(i2c_fh);

    //Set DES1 I2C speed to 1MHz
    write_reg(i2c_fh, 0x2B, 0x08);
    write_reg(i2c_fh, 0x2C, 0x08);

    eStatus |= link_set_deserializer_daisy_temp_optimization(i2c_fh);
    CRITICAL_PRINT(MODULE_NAME, "Enable Deserializer oLDI");
    eStatus |= link_set_deserializer_daisy_enable_oldi(i2c_fh);
    i2c_set_slave_addr(i2c_fh, DES_DC_ID_7BIT_ALIAS, I2C_ADDRFMT_7BIT);
    (void) usleep(50 * 1000);

    CRITICAL_PRINT(MODULE_NAME, "----------Measure Timing from Deserializer 1---------");
    serdes_des_measure_video_dtg(i2c_fh, 0, NULL, 1);
    serdes_des_measure_video_fpd_iv(i2c_fh, 1);
    i2c_set_slave_addr(i2c_fh, SER_ID_7BIT, I2C_ADDRFMT_7BIT);
    serdes_daisy_clear_link_crc_err_flag(i2c_fh);

    return eStatus;
}

void set_reset_keep_dprx(int32 val)
{
    reset_keep_dprx = val;
}

#ifdef __cplusplus
}
#endif
