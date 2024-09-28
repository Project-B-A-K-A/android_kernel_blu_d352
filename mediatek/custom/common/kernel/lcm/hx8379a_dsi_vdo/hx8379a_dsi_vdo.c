/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
	#define LCM_DEBUG  printf
	#define LCM_FUNC_TRACE() printf("huyl [uboot] %s\n",__func__)
#else
	#define LCM_DEBUG  printk
	#define LCM_FUNC_TRACE() printk("huyl [kernel] %s\n",__func__)
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)  //lingjinming //(854)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_HX8379A                                      0x79

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY             							0XAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
static void lcm_init_setting()
{
        unsigned int data_array[16];

        data_array[0]= 0x00043902;
        data_array[1]= 0x7983FFB9;
        dsi_set_cmdq(data_array, 2, 1);
        MDELAY(10);

        data_array[0]= 0x51BA1500;
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(10);

        data_array[0]= 0x00143902;
        data_array[1]= 0x445000B1;
        data_array[2]= 0x11088DEA;
        data_array[3]= 0x2F271111;
        data_array[4]= 0x0B421A9A;
        data_array[5]= 0xE600F16E;
        dsi_set_cmdq(data_array, 6, 1);
        MDELAY(10);

        data_array[0]= 0x000E3902;
        data_array[1]= 0xFE0000B2;
        data_array[2]= 0x22190408;
        data_array[3]= 0x0408FF00;
        data_array[4]= 0x00002019;
        dsi_set_cmdq(data_array, 5, 1);
        MDELAY(10);

        data_array[0]= 0x00203902;
        data_array[1]= 0x000882B4;
        data_array[2]= 0x32031032;
        data_array[3]= 0x10327013;
        data_array[4]= 0x28013708;
        data_array[5]= 0x3C083707;
        data_array[6]= 0x08444420;
        data_array[7]= 0x28084000;
        data_array[8]= 0x04303008;
        dsi_set_cmdq(data_array, 9, 1);
        MDELAY(10);

        data_array[0]= 0x00303902;
        data_array[1]= 0x0A0000D5;
        data_array[2]= 0x00050100;
        data_array[3]= 0x88880003;
        data_array[4]= 0x01238888;
        data_array[5]= 0x13024567;
        data_array[6]= 0x88888888;
        data_array[7]= 0x88888888;
        data_array[8]= 0x76548888;
        data_array[9]= 0x20313210;
        data_array[10]= 0x88888888;
        data_array[11]= 0x00008888;
        data_array[12]= 0x00000000;
        dsi_set_cmdq(data_array, 13, 1);
        MDELAY(10);

        data_array[0]= 0x00243902;
        data_array[1]= 0x080579E0;
        data_array[2]= 0x3F25240E;
        data_array[3]= 0x0E07452C;
        data_array[4]= 0x15171410;
        data_array[5]= 0x05191416;
        data_array[6]= 0x25240E08;
        data_array[7]= 0x07452C3F;
        data_array[8]= 0x1714100E;
        data_array[9]= 0x19141615;
       // data_array[10]= 0x00000000;
        dsi_set_cmdq(data_array, 10, 1);
        MDELAY(10);

        data_array[0]= 0x02CC1500;
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(10);

        //{0XB6, 4,   {0X00,0XA0,0X00,0XA0}},
        //{REGFLAG_DELAY, 10, {}},
        data_array[0]= 0x00053902;
        data_array[1]= 0x009E00B6;
        data_array[2]= 0x0000009E;
        dsi_set_cmdq(data_array, 3, 1);
        MDELAY(10);

        data_array[0] = 0x00350500;
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(10);

        data_array[0] = 0x00110500;        //exit sleep mode
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(200);

        data_array[0] = 0x00290500;        //exit sleep mode
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(20);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

static struct LCM_setting_table lcm_initialization_setting[] = {

	{0xB9,3,{0xFF,0x83,0x79}},
	{0xB1,20,{0x46,0x1b,0x1b,0x33,0x53,0x50,0xf0,0xee,0x52,0x80,0x38,0x38,0xf8,0x24,0x42,0x22,0x00,0x80,0x30,0x00}},
	{0xB2,5,{0x80,0xfe,0x0b,0x03,0x80}},
	{0xB4,10,{0x04,0x38,0x08,0x3a,0x08,0x3a,0x05,0x3e,0x0c,0x3e}},
	{0xCC,1,{0x02}}, //{0xCC,1,{0x0a}},
	{0xD3,28,{0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x32,0x10,0x03,0x00,0x03,0x03,0x70,0x03,0x70,0x00,0x08,0x00,0x08,0x37,0x33,0x07,0x07,0x37,0x07,0x07,0x37}},
	{0xD5,32,{0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x02,0x03,0x00,0x01,0x06,0x07,0x04,0x05,0x20,0x22,0x21,0x23,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
	{0xD6,32,{0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x05,0x04,0x07,0x06,0x01,0x00,0x03,0x02,0x23,0x21,0x02,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

	{0xE0,42,{0x01,0x03,0x02,0x10,0x10,0x20,0x22,0x36,0x09,0x11,0x13,0x1b,0x15,0x19,0x1a,0x17,0x17,0x08,0x13,0x14,0x18,0x01,0x03,0x02,0x10,0x10,0x20,0x22,0x36,0x09,0x11,0x13,0x1b,0x15,0x19,0x1a,0x17,0x17,0x08,0x13,0x14,0x18}},
	{0xB6,2,{0x8b,0x8b}},

	{0X11,1,{0}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 1, {0x00}},
	//{0x2c,1,{0x00}},	//????
	{REGFLAG_END_OF_TABLE, 0x00, {}}


};


static struct LCM_setting_table lcm_initialization_setting_ytqlcd_boe397[] = {///add by maxyu 140312 for S819

	{0xB9,3,{0xFF,0x83,0x79}},
	{0xB1,20,{0x64,0x17,0x17,0x33,0x93,0x90,0xf0,0xec,0xd8,0x80,0x38,0x38,0xf8,0x33,0x24,0x44,0x00,0x80,0x30,0x00}},
	{0xB2,9,{0x80,0xfe,0x0b,0x04,0x30,0x50,0x11,0x42,0x1d}},
	{0xB4,13,{0x06,0x80,0x01,0x86,0x04,0x86,0x02,0x86,0x08,0x86,0xb0,0x00,0xff}},
	//{0xCC,1,{0x02}}, //{0xCC,1,{0x0a}},
	{0xD3,29,{0x00,0x00,0x00,0x00,0x00,0x06,0x06,0x32,0x10,0x06,0x00,0x06,0x03,0x70,0x03,0x70,0x00,0x08,0x00,0x08,0x11,0x11,0x06,0x06,0x13,0x06,0x06,0x13,0x09}},
	{0xD5,32,{0x18,0x18,0x18,0x18,0x00,0x01,0x02,0x03,0x20,0x21,0x19,0x19,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
	{0xD6,32,{0x18,0x18,0x19,0x19,0x01,0x00,0x03,0x02,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

	{0xE0,42,{0x00,0x00,0x00,0x1c,0x1d,0x3e,0x2f,0x3e,0x08,0x0a,0x0c,0x17,0x11,0x16,0x18,0x16,0x17,0x07,0x10,0x10,0x14,0x00,0x00,0x00,0x1c,0x1d,0x3e,0x2f,0x3e,0x08,0x0a,0x0c,0x17,0x11,0x16,0x18,0x16,0x17,0x07,0x10,0x10,0x14}},
	{0xB6,2,{0x79,0x79}},

	{0xcc,1,{0x0e}},

	{0x36,1,{0x02}},//for direction


	{0X11,1,{0}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 1, {0x00}},
	//{0x2c,1,{0x00}},	//????
	{REGFLAG_END_OF_TABLE, 0x00, {}}


};

static struct LCM_setting_table lcm_initialization_setting_ytqlcd_boe397_new[] = {///add by maxyu 140323 for s1025/s1021

	{0xB9,3,{0xFF,0x83,0x79}},
	{0xB1,20,{0x44,0x18,0x18,0x31,0x51,0x50,0xD0,0xEE,0x94,0x80,0x38,0x38,0xF8,0x22,0x22,0x22,0x00,0x80,0x30,0x00}},
	{0xB2,9,{0x80,0x3C,0x0B,0x04,0x30,0x50,0x11,0x42,0x1D}},
	{0xB4,10,{0x11,0x7B,0x11,0x7B,0x11,0x7B,0x0A,0x84,0x0A,0x84}},
	{0xCC,1,{0x02}}, 
	{0xD2,1,{0x33}},
	{0xD3,29,{0x00,0x00,0x00,0x00,0x00,0x06,0x06,0x32,0x10,0x06,0x00,0x06,0x03,0x70,0x03,0x70,0x00,0x08,0x00,0x08,0x11,0x11,0x06,0x06,0x13,0x06,0x06,0x13,0x09}},
	{0xD5,32,{0x18,0x18,0x19,0x19,0x01,0x00,0x03,0x02,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
	{0xD6,32,{0x18,0x18,0x18,0x18,0x02,0x03,0x00,0x01,0x20,0x21,0x19,0x19,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

	{0xE0,42,{0x00,0x00,0x05,0x18,0x1C,0x3F,0x31,0x3F,0x09,0x10,0x12,0x1A,0x13,0x16,0x19,0x16,0x15,0x07,0x13,0x13,0x18,0x00,0x00,0x05,0x19,0x1B,0x3F,0x31,0x3F,0x09,0x0F,0x11,0x1A,0x11,0x15,0x17,0x15,0x16,0x07,0x12,0x13,0x17}},
	{0xB6,2,{0x79,0x79}},

	{0xB6,2,{0x76,0x76}},

	{0x36,1,{0x00}},//for direction
	{0x35, 1 ,{0x00}},


	{0X11,1,{0}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 1, {0x00}},
	//{0x2c,1,{0x00}},	//????
	{REGFLAG_END_OF_TABLE, 0x00, {}}


};


static struct LCM_setting_table lcm_initialization_setting_ytqlcd_boe397_new0815[] = {///add by maxyu 140815 for s1025/s1021

	{0xB9,3,{0xFF,0x83,0x79}},
	{0xB1,20,{0x44,0x18,0x18,0x31,0x31,0x90,0xD0,0xEE,0x94,0x80,0x38,0x38,0xF8,0x22,0x22,0x22,0x00,0x80,0x30,0x00}},
	{0x3A, 1 ,{0x77}},
	{0xB2,9,{0x80,0x3C,0x0B,0x04,0x00,0x50,0x11,0x42,0x1D}},
	{0xB4,10,{0x50,0x51,0x50,0x51,0x50,0x51,0x12,0xA0,0x13,0xA0}},
	//{0xCC,1,{0x02}}, 
	//{0xD2,1,{0x33}},
	{0xD3,29,{0x00,0x07,0x00,0x00,0x00,0x06,0x06,0x32,0x10,0x05,0x00,0x05,0x03,0x6F,0x03,0x6F,0x00,0x07,0x00,0x07,0x21,0x22,0x05,0x05,0x23,0x05,0x05,0x23,0x09}},
	{0xD5,32,{0x18,0x18,0x19,0x19,0x01,0x00,0x03,0x02,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
	{0xD6,32,{0x18,0x18,0x18,0x18,0x02,0x03,0x00,0x02,0x20,0x21,0x19,0x19,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

	{0xE0,42,{0x00,0x00,0x05,0x18,0x1C,0x3F,0x31,0x3F,0x09,0x10,0x12,0x1A,0x13,0x16,0x19,0x16,0x15,0x07,0x13,0x13,0x18,0x00,0x00,0x05,0x19,0x1B,0x3F,0x31,0x3F,0x09,0x0F,0x11,0x1A,0x11,0x15,0x17,0x15,0x16,0x07,0x12,0x13,0x17}},
	{0xB6,2,{0x84,0x84}},
	{0xCC,1,{0x02}}, 

	{0x36,1,{0x00}},//for direction
	{0x35, 1 ,{0x00}},


	{0X11,1,{0}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 1, {0x00}},
	//{0x2c,1,{0x00}},	//????
	{REGFLAG_END_OF_TABLE, 0x00, {}}


};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xF0,	5,	{0x55, 0xaa, 0x52,0x08,0x00}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		MDELAY(2);

       	}
    }

}




// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));

		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
    		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
        #endif

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting
	    params->dsi.intermediat_buffer_num = 2;
    
	    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
#if 1//for white line
	    params->dsi.vertical_sync_active				= 6;
	    params->dsi.vertical_backporch					= 5;
	    params->dsi.vertical_frontporch					= 5;
	    params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	    
	    params->dsi.horizontal_sync_active				= 55;
	    params->dsi.horizontal_backporch				= 55;
	    params->dsi.horizontal_frontporch				=55;
	    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#else
		params->dsi.vertical_sync_active				=  3; // 2;  //lingjinming ///// 4;// 3    2
		params->dsi.vertical_backporch					= 7;// 20   1
		params->dsi.vertical_frontporch					= 6;  // 3;  //lingjinming  //  6;//lingjinming ÉÏÏÂ!!!! // 1  12  //vsp
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 78;// 50  2
		params->dsi.horizontal_backporch				= 78;
		params->dsi.horizontal_frontporch				= 78; //lingjinming ×óÓÒ!!!
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#endif


		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4
		//params->dsi.fbk_div =27;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)


		params->dsi.PLL_CLOCK = 148;///160 173; //dsi clock customization: should config clock value directly
		params->dsi.ssc_disable = 1;
		params->dsi.ssc_range = 4;

}

//add by hyde for debug
static void lcm_init(void)
{

       MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);

	SET_RESET_PIN(1);
	MDELAY(120);

	//lcm_init_setting();//lingjinming

	///push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); ///for S819 YTQ_HSD
	///push_table(lcm_initialization_setting_ytqlcd_boe397_new, sizeof(lcm_initialization_setting_ytqlcd_boe397_new) / sizeof(struct LCM_setting_table), 1);///for S1025/S1021 YTQ_BOE
	push_table(lcm_initialization_setting_ytqlcd_boe397_new0815, sizeof(lcm_initialization_setting_ytqlcd_boe397_new0815) / sizeof(struct LCM_setting_table), 1);///for S1025/S1021 YTQ_BOE



}



static void lcm_suspend(void)
{
	//MDELAY(20);
	#if 0
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);
	#else
	push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	#endif
}


static void lcm_resume(void)
{
	#if 0
	lcm_init();
	#else
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	#endif

}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}


static unsigned int lcm_compare_id(void)
{
        unsigned int data_array[16];
	 unsigned char buffer[5];
	 unsigned char id_high=0;
	 unsigned char id_low=0;
	 unsigned int id=0;

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	//lcm_init_setting();

        data_array[0]= 0x00043902;
        data_array[1]= 0x7983FFB9;
        dsi_set_cmdq(&data_array, 2, 1);
	 MDELAY(10);
	 data_array[0] = 0x00023700;
	 dsi_set_cmdq(&data_array, 1, 1);
	 MDELAY(10);
	 read_reg_v2(0xF4, buffer, 2);
	 id = buffer[0];

	#if defined(BUILD_LK)
		printf("%s LK hx8379a id = 0x%08x \n", __func__, id);
	#else
		printk("%s hx8379a id = 0x%08x \n", __func__, id);
	#endif


	return (LCM_ID_HX8379A == id)?1:0;
}

static unsigned int lcm_esd_check(void)
{

  #ifndef BUILD_LK
        if(lcm_esd_test)
        {
            lcm_esd_test = FALSE;
            return TRUE;
        }

    unsigned char buffer[1];
    unsigned int array[16];



    array[0] = 0x00013700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);

        read_reg_v2(0x0A, buffer, 1);

	#if defined(BUILD_LK)
	    printf("lcm_esd_check  0x0A = %x\n",buffer[0]);
	#else
	    printk("lcm_esd_check  0x0A = %x\n",buffer[0]);
	#endif

	if(buffer[0] != 0x1C)
        {
            return TRUE;
        }

	read_reg_v2(0x0B, buffer, 1);
	#if defined(BUILD_LK)
	    printf("lcm_esd_check  0x0B = %x\n",buffer[0]);
	#else
	    printk("lcm_esd_check  0x0B = %x\n",buffer[0]);
	#endif

	if(buffer[0] != 0x00)
        {
            return TRUE;
        }

	array[0]= 0x00043700;// | (max_return_size << 16);

        dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x09, buffer, 4);

	#if defined(BUILD_LK)
	printf("lcm_esd_check,buffer[0]=%x\n", buffer[0]);
	printf("lcm_esd_check,buffer[1]=%x\n", buffer[1]);
	printf("lcm_esd_check,buffer[2]=%x\n", buffer[2]);
	printf("lcm_esd_check,buffer[3]=%x\n", buffer[3]);
	#else
	printk("lcm_esd_check,buffer[0]=%x\n", buffer[0]);
	printk("lcm_esd_check,buffer[1]=%x\n", buffer[1]);
	printk("lcm_esd_check,buffer[2]=%x\n", buffer[2]);
	printk("lcm_esd_check,buffer[3]=%x\n", buffer[3]);
	#endif

	/*if((buffer[0] != 0x80)&&(buffer[1] != 0x73));//&&(buffer[2] != 0x37)&&(buffer[3] != 0x04));
	{
	   //return TRUE;
	}*/

    return FALSE;
#endif


}

static unsigned int lcm_esd_recover(void)
{

	lcm_init();
	lcm_resume();

	return TRUE;
}



LCM_DRIVER hx8379a_dsi_vdo_lcm_drv=
{
    	.name = "zechin_mt6572m_hx8379a_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

