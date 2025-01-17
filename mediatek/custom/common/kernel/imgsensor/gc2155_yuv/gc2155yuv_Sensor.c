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
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *  
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Leo Lee
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * [GC2155YUV V1.0.0]
 * 2.19.2014 Lanking
 * .First Release
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GalaxyCoreinc. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "gc2155yuv_Sensor.h"
#include "gc2155yuv_Camera_Sensor_para.h"
#include "gc2155yuv_CameraCustomized.h"

#define GC2155YUV_DEBUG
#ifdef GC2155YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

  // #define  scaler_preview

#define  GC2155_SET_PAGE0    GC2155_write_cmos_sensor(0xfe,0x00)
#define  GC2155_SET_PAGE1    GC2155_write_cmos_sensor(0xfe,0x01)
#define  GC2155_SET_PAGE2    GC2155_write_cmos_sensor(0xfe,0x02)
#define  GC2155_SET_PAGE3    GC2155_write_cmos_sensor(0xfe,0x03)


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
/*************************************************************************
* FUNCTION
*    GC2155_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void GC2155_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), GC2155_WRITE_ID); 

#if (defined(__GC2155_DEBUG_TRACE__))
  if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}

/*************************************************************************
* FUNCTION
*    GC2155_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 GC2155_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), GC2155_WRITE_ID)) {
        SENSORDB("ERROR: GC2155_read_cmos_sensor \n");
    }

#if (defined(__GC2155_DEBUG_TRACE__))
  if (size != rt) printk("I2C read %x error\n", addr);
#endif

  return in_buff[0];
}


/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 

static kal_bool GC2155_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool GC2155_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 GC2155_exposure_lines=0, GC2155_extra_exposure_lines = 0;

static kal_uint16 GC2155_Capture_Shutter=0;
static kal_uint16 GC2155_Capture_Extra_Lines=0;

kal_uint32 GC2155_capture_pclk_in_M=520,GC2155_preview_pclk_in_M=390,GC2155_PV_dummy_pixels=0,GC2155_PV_dummy_lines=0,GC2155_isp_master_clock=0;

static kal_uint32  GC2155_sensor_pclk=390;

static kal_uint32 Preview_Shutter = 0;
static kal_uint32 Capture_Shutter = 0;

MSDK_SENSOR_CONFIG_STRUCT GC2155SensorConfigData;

kal_uint16 GC2155_read_shutter(void)
{
	return  (GC2155_read_cmos_sensor(0x03) << 8)|GC2155_read_cmos_sensor(0x04) ;
} /* GC2155 read_shutter */



static void GC2155_write_shutter(kal_uint32 shutter)
{

	if(shutter < 1)	
 	return;

	GC2155_write_cmos_sensor(0x03, (shutter >> 8) & 0x1f);
	GC2155_write_cmos_sensor(0x04, shutter & 0xff);
}    /* GC2155_write_shutter */


#define GC2155_TEST_PATTERN_CHECKSUM  0x9e279a16 // 

UINT32 GC2155SetTestPatternMode(kal_bool bEnable)
{
    printk("[GC2155SetTestPatternMode] Test pattern enable:%d\n", bEnable);
	if(bEnable)    // enable test pattern output  0809 ok!!!
	{

	 GC2155_write_cmos_sensor(0xfe, 0x00); 
	 

	 GC2155_write_cmos_sensor(0x80, 0x08); 
        GC2155_write_cmos_sensor(0x81,	0x00); 
	 GC2155_write_cmos_sensor(0x82,	0x00); 	 
        GC2155_write_cmos_sensor(0xb6,	0x00); 
		
	 GC2155_write_cmos_sensor(0x18,	0x06); 
	 
	 GC2155_write_cmos_sensor(0xb3,	0x40); 	 
        GC2155_write_cmos_sensor(0xb4,	0x40); // 0x11
        GC2155_write_cmos_sensor(0xb5,	0x40);

		GC2155_write_cmos_sensor(0xfe,	0x02); 	 
        GC2155_write_cmos_sensor(0xd0,	0x40); // 0x11
        GC2155_write_cmos_sensor(0xdd,	0x00);

		GC2155_write_cmos_sensor(0xfe,	0x00); 
        GC2155_write_cmos_sensor(0xb1,	0x40); 
	 GC2155_write_cmos_sensor(0xb2,	0x40); 	 
        GC2155_write_cmos_sensor(0x03, 0x00); 
		 GC2155_write_cmos_sensor(0x04,	0x00); 
		 
	 GC2155_write_cmos_sensor(0x8c,	0x01); 	 
        GC2155_write_cmos_sensor(0x8d, 0x88); 

	}
    return ERROR_NONE;

}

static void GC2155_set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 GC2155_HV_Mirror;

	switch (image_mirror) 
	{
		case IMAGE_NORMAL:
			GC2155_HV_Mirror = 0x14; 
		    break;
		case IMAGE_H_MIRROR:
			GC2155_HV_Mirror = 0x15;
		    break;
		case IMAGE_V_MIRROR:
			GC2155_HV_Mirror = 0x16; 
		    break;
		case IMAGE_HV_MIRROR:
			GC2155_HV_Mirror = 0x17;
		    break;
		default:
		    break;
	}
	GC2155_write_cmos_sensor(0x17, GC2155_HV_Mirror);
}

static void GC2155_set_AE_mode(kal_bool AE_enable)
{
	kal_uint8 temp_AE_reg = 0;

	GC2155_write_cmos_sensor(0xfe, 0x00);
	if (AE_enable == KAL_TRUE)
	{
		// turn on AEC/AGC
		GC2155_write_cmos_sensor(0xb6, 0x01);
	}
	else
	{
		// turn off AEC/AGC
		GC2155_write_cmos_sensor(0xb6, 0x00);
	}
}


static void GC2155_set_AWB_mode(kal_bool AWB_enable)
{
	kal_uint8 temp_AWB_reg = 0;

	GC2155_write_cmos_sensor(0xfe, 0x00);
	temp_AWB_reg = GC2155_read_cmos_sensor(0x82);
	if (AWB_enable == KAL_TRUE)
	{
		//enable Auto WB
		temp_AWB_reg = temp_AWB_reg | 0x02;
	}
	else
	{
		//turn off AWB
		temp_AWB_reg = temp_AWB_reg & 0xfd;
	}
	GC2155_write_cmos_sensor(0x82, temp_AWB_reg);
}


/*************************************************************************
* FUNCTION
*	GC2155_night_mode
*
* DESCRIPTION
*	This function night mode of GC2155.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void GC2155_night_mode(kal_bool enable)
{
	
		/* ==Video Preview, Auto Mode, use 39MHz PCLK, 30fps; Night Mode use 39M, 15fps */
		if (GC2155_sensor_cap_state == KAL_FALSE) 
		{
			if (enable) 
			{
				if (GC2155_VEDIO_encode_mode == KAL_TRUE) 
				{
					GC2155_write_cmos_sensor(0xfe, 0x01);
					GC2155_write_cmos_sensor(0x3c, 0x60);
					GC2155_write_cmos_sensor(0xfe, 0x00);
				}
				else 
				{
					GC2155_write_cmos_sensor(0xfe, 0x01);
					GC2155_write_cmos_sensor(0x3c, 0x60);
					GC2155_write_cmos_sensor(0xfe, 0x00);
				}
			}
			else 
			{
				/* when enter normal mode (disable night mode) without light, the AE vibrate */
				if (GC2155_VEDIO_encode_mode == KAL_TRUE) 
				{
					GC2155_write_cmos_sensor(0xfe, 0x01);
					GC2155_write_cmos_sensor(0x3c, 0x40);
					GC2155_write_cmos_sensor(0xfe, 0x00);
				}
				else 
				{
					GC2155_write_cmos_sensor(0xfe, 0x01);
					GC2155_write_cmos_sensor(0x3c, 0x40);
					GC2155_write_cmos_sensor(0xfe, 0x00);
				}
		}
	}
}	/* GC2155_night_mode */



/*************************************************************************
* FUNCTION
*	GC2155_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 GC2155_GetSensorID(kal_uint32 *sensorID)

{
	   int  retry = 3; 
    // check if sensor ID correct
    do {
	
	*sensorID=((GC2155_read_cmos_sensor(0xf0) << 8) | GC2155_read_cmos_sensor(0xf1));	
	 if (*sensorID == GC2155_SENSOR_ID)
            break; 

	SENSORDB("GC2155_GetSensorID:%x \n",*sensorID);
	retry--;

	  } while (retry > 0);
	
	if (*sensorID != GC2155_SENSOR_ID) {		
		*sensorID = 0xFFFFFFFF;		
		return ERROR_SENSOR_CONNECT_FAIL;
	}
   return ERROR_NONE;
}   /* GC2155Open  */

static void GC2155_Sensor_Init(void)
{
	zoom_factor = 0; 
	SENSORDB("GC2155_Sensor_Init");
	GC2155_write_cmos_sensor(0xfe , 0xf0);
	GC2155_write_cmos_sensor(0xfe , 0xf0);
	GC2155_write_cmos_sensor(0xfe , 0xf0);
	GC2155_write_cmos_sensor(0xfc , 0x06);
	GC2155_write_cmos_sensor(0xf6 , 0x00);
	GC2155_write_cmos_sensor(0xf7 , 0x1d);
	GC2155_write_cmos_sensor(0xf8 , 0x84);
	GC2155_write_cmos_sensor(0xfa , 0x00);
	GC2155_write_cmos_sensor(0xf9 , 0xfe);
	GC2155_write_cmos_sensor(0xf2 , 0x00);
	/////////////////////////////////////////////////
	//////////////////ISP reg//////////////////////
	////////////////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0x03 , 0x04);
	GC2155_write_cmos_sensor(0x04 , 0xe2);
	GC2155_write_cmos_sensor(0x09 , 0x00);
	GC2155_write_cmos_sensor(0x0a , 0x00);
	GC2155_write_cmos_sensor(0x0b , 0x00);
	GC2155_write_cmos_sensor(0x0c , 0x00);
	GC2155_write_cmos_sensor(0x0d , 0x04);
	GC2155_write_cmos_sensor(0x0e , 0xc0);
	GC2155_write_cmos_sensor(0x0f , 0x06);
	GC2155_write_cmos_sensor(0x10 , 0x50);
	GC2155_write_cmos_sensor(0x12 , 0x2e);
	GC2155_write_cmos_sensor(0x17 , 0x14); // mirror
	GC2155_write_cmos_sensor(0x18 , 0x02);
	GC2155_write_cmos_sensor(0x19 , 0x0e);
	GC2155_write_cmos_sensor(0x1a , 0x01);
	GC2155_write_cmos_sensor(0x1b , 0x4b);
	GC2155_write_cmos_sensor(0x1c , 0x07);
	GC2155_write_cmos_sensor(0x1d , 0x10);
	GC2155_write_cmos_sensor(0x1e , 0x98);
	GC2155_write_cmos_sensor(0x1f , 0x78);
	GC2155_write_cmos_sensor(0x20 , 0x05);
	GC2155_write_cmos_sensor(0x21 , 0x40);
	GC2155_write_cmos_sensor(0x22 , 0xf0);
	GC2155_write_cmos_sensor(0x24 , 0x3f);//lilong 20141515 modefy,0x16->0x3f
	GC2155_write_cmos_sensor(0x25 , 0x01);
	GC2155_write_cmos_sensor(0x26 , 0x10);
	GC2155_write_cmos_sensor(0x2d , 0x40);
	GC2155_write_cmos_sensor(0x30 , 0x01);
	GC2155_write_cmos_sensor(0x31 , 0x90);
	GC2155_write_cmos_sensor(0x33 , 0x04);
	GC2155_write_cmos_sensor(0x34 , 0x01);
	/////////////////////////////////////////////////
	//////////////////ISP reg////////////////////
	/////////////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0x80 , 0xff);
	GC2155_write_cmos_sensor(0x81 , 0x2c);
	GC2155_write_cmos_sensor(0x82 , 0xfa);
	GC2155_write_cmos_sensor(0x83 , 0x00);
	GC2155_write_cmos_sensor(0x84 , 0x03); //yuv 01
	GC2155_write_cmos_sensor(0x85 , 0x08);
	GC2155_write_cmos_sensor(0x86 , 0x02);
	GC2155_write_cmos_sensor(0x89 , 0x03);
	GC2155_write_cmos_sensor(0x8a , 0x00);
	GC2155_write_cmos_sensor(0x8b , 0x00);
	GC2155_write_cmos_sensor(0xb0 , 0x55);
	GC2155_write_cmos_sensor(0xc3 , 0x11); //00
	GC2155_write_cmos_sensor(0xc4 , 0x20);
	GC2155_write_cmos_sensor(0xc5 , 0x30);
	GC2155_write_cmos_sensor(0xc6 , 0x38);
	GC2155_write_cmos_sensor(0xc7 , 0x40);
	GC2155_write_cmos_sensor(0xec , 0x02);
	GC2155_write_cmos_sensor(0xed , 0x04);
	GC2155_write_cmos_sensor(0xee , 0x60);
	GC2155_write_cmos_sensor(0xef , 0x90);
	GC2155_write_cmos_sensor(0xb6 , 0x01);
	GC2155_write_cmos_sensor(0x90 , 0x01);
	GC2155_write_cmos_sensor(0x91 , 0x00);
	GC2155_write_cmos_sensor(0x92 , 0x00);
	GC2155_write_cmos_sensor(0x93 , 0x00);
	GC2155_write_cmos_sensor(0x94 , 0x00);
	GC2155_write_cmos_sensor(0x95 , 0x04);
	GC2155_write_cmos_sensor(0x96 , 0xb0);
	GC2155_write_cmos_sensor(0x97 , 0x06);
	GC2155_write_cmos_sensor(0x98 , 0x40);
	/////////////////////////////////////////
	/////////// BLK ////////////////////////
	/////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0x18 , 0x02);
	GC2155_write_cmos_sensor(0x40 , 0x42);
	GC2155_write_cmos_sensor(0x41 , 0x00);
	GC2155_write_cmos_sensor(0x43 , 0x54);
	GC2155_write_cmos_sensor(0x5e , 0x00);
	GC2155_write_cmos_sensor(0x5f , 0x00);
	GC2155_write_cmos_sensor(0x60 , 0x00);
	GC2155_write_cmos_sensor(0x61 , 0x00);
	GC2155_write_cmos_sensor(0x62 , 0x00);
	GC2155_write_cmos_sensor(0x63 , 0x00);
	GC2155_write_cmos_sensor(0x64 , 0x00);
	GC2155_write_cmos_sensor(0x65 , 0x00);
	GC2155_write_cmos_sensor(0x66 , 0x20);
	GC2155_write_cmos_sensor(0x67 , 0x20);
	GC2155_write_cmos_sensor(0x68 , 0x20);
	GC2155_write_cmos_sensor(0x69 , 0x20);
	GC2155_write_cmos_sensor(0x6a , 0x08);
	GC2155_write_cmos_sensor(0x6b , 0x08);
	GC2155_write_cmos_sensor(0x6c , 0x08);
	GC2155_write_cmos_sensor(0x6d , 0x08);
	GC2155_write_cmos_sensor(0x6e , 0x08);
	GC2155_write_cmos_sensor(0x6f , 0x08);
	GC2155_write_cmos_sensor(0x70 , 0x08);
	GC2155_write_cmos_sensor(0x71 , 0x08);
	GC2155_write_cmos_sensor(0x72 , 0xf0);
	GC2155_write_cmos_sensor(0x7e , 0x3c);
	GC2155_write_cmos_sensor(0x7f , 0x00);
	GC2155_write_cmos_sensor(0xfe , 0x00);
	////////////////////////////////////////
	/////////// AEC ////////////////////////
	////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x01 , 0x08);
	GC2155_write_cmos_sensor(0x02 , 0xc0);
	GC2155_write_cmos_sensor(0x03 , 0x04);
	GC2155_write_cmos_sensor(0x04 , 0x90);
	GC2155_write_cmos_sensor(0x05 , 0x30);
	GC2155_write_cmos_sensor(0x06 , 0x98);
	GC2155_write_cmos_sensor(0x07 , 0x28);
	GC2155_write_cmos_sensor(0x08 , 0x6c);
	GC2155_write_cmos_sensor(0x09 , 0x00);
	GC2155_write_cmos_sensor(0x0a , 0xc2);
	GC2155_write_cmos_sensor(0x0b , 0x11);
	GC2155_write_cmos_sensor(0x0c , 0x10);
	GC2155_write_cmos_sensor(0x13 , 0x2d);
	GC2155_write_cmos_sensor(0x17 , 0x00);
	GC2155_write_cmos_sensor(0x1c , 0x11);
	GC2155_write_cmos_sensor(0x1e , 0x61);
	GC2155_write_cmos_sensor(0x1f , 0x30);
	GC2155_write_cmos_sensor(0x20 , 0x40);
	GC2155_write_cmos_sensor(0x22 , 0x80);
	GC2155_write_cmos_sensor(0x23 , 0x20);

	GC2155_write_cmos_sensor(0x12 , 0x30);//35
	GC2155_write_cmos_sensor(0x15 , 0x50);
	GC2155_write_cmos_sensor(0x10 , 0x31);
	GC2155_write_cmos_sensor(0x3e , 0x28);
	GC2155_write_cmos_sensor(0x3f , 0xe0);
	GC2155_write_cmos_sensor(0x40 , 0xe0);
	GC2155_write_cmos_sensor(0x41 , 0x08);

	GC2155_write_cmos_sensor(0xfe , 0x02);
	GC2155_write_cmos_sensor(0x0f , 0x05);
	/////////////////////////////
	//////// INTPEE /////////////
	/////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x02);
	GC2155_write_cmos_sensor(0x90 , 0x6c);
	GC2155_write_cmos_sensor(0x91 , 0x03);
	GC2155_write_cmos_sensor(0x92 , 0xc4);
	GC2155_write_cmos_sensor(0x97 , 0x74);// 64
	GC2155_write_cmos_sensor(0x98 , 0x88);
	GC2155_write_cmos_sensor(0x9d , 0x0c);// 0a
	GC2155_write_cmos_sensor(0xa2 , 0x11);
	GC2155_write_cmos_sensor(0xfe , 0x00);
	/////////////////////////////
	//////// DNDD///////////////
	/////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x02);
	GC2155_write_cmos_sensor(0x80 , 0xc1);
	GC2155_write_cmos_sensor(0x81 , 0x08);
	GC2155_write_cmos_sensor(0x82 , 0x05);
	GC2155_write_cmos_sensor(0x83 , 0x05);// 0x04
	GC2155_write_cmos_sensor(0x84 , 0x0a);
	GC2155_write_cmos_sensor(0x86 , 0x80);
	GC2155_write_cmos_sensor(0x87 , 0x30);
	GC2155_write_cmos_sensor(0x88 , 0x15);
	GC2155_write_cmos_sensor(0x89 , 0x80);
	GC2155_write_cmos_sensor(0x8a , 0x60);
	GC2155_write_cmos_sensor(0x8b , 0x30);
	/////////////////////////////////////////
	/////////// ASDE ////////////////////////
	/////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x21 , 0x14);
	GC2155_write_cmos_sensor(0xfe , 0x02);
	GC2155_write_cmos_sensor(0x3c , 0x06);
	GC2155_write_cmos_sensor(0x3d , 0x40);
	GC2155_write_cmos_sensor(0x48 , 0x30);
	GC2155_write_cmos_sensor(0x49 , 0x06);
	GC2155_write_cmos_sensor(0x4b , 0x08);
	GC2155_write_cmos_sensor(0x4c , 0x20);
	GC2155_write_cmos_sensor(0xa3 , 0x50);
	GC2155_write_cmos_sensor(0xa4 , 0x30);
	GC2155_write_cmos_sensor(0xa5 , 0x40);
	GC2155_write_cmos_sensor(0xa6 , 0x80);
	GC2155_write_cmos_sensor(0xab , 0x20);///0x40
	GC2155_write_cmos_sensor(0xae , 0x0c);
	GC2155_write_cmos_sensor(0xb3 , 0x42);
	GC2155_write_cmos_sensor(0xb4 , 0x24);
	GC2155_write_cmos_sensor(0xb6 , 0x50);
	GC2155_write_cmos_sensor(0xb7 , 0x01);
	GC2155_write_cmos_sensor(0xb9 , 0x28); 
	GC2155_write_cmos_sensor(0xfe , 0x00);	 
	///////////////////gamma1////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x02);
	GC2155_write_cmos_sensor(0x10 , 0x0A);
	GC2155_write_cmos_sensor(0x11 , 0x10);
	GC2155_write_cmos_sensor(0x12 , 0x16);
	GC2155_write_cmos_sensor(0x13 , 0x1C);
	GC2155_write_cmos_sensor(0x14 , 0x27);
	GC2155_write_cmos_sensor(0x15 , 0x34);
	GC2155_write_cmos_sensor(0x16 , 0x44);
	GC2155_write_cmos_sensor(0x17 , 0x55);
	GC2155_write_cmos_sensor(0x18 , 0x6E);
	GC2155_write_cmos_sensor(0x19 , 0x81);
	GC2155_write_cmos_sensor(0x1a , 0x91);
	GC2155_write_cmos_sensor(0x1b , 0x9F);
	GC2155_write_cmos_sensor(0x1c , 0xAA);
	GC2155_write_cmos_sensor(0x1d , 0xBC);
	GC2155_write_cmos_sensor(0x1e , 0xCA);
	GC2155_write_cmos_sensor(0x1f , 0xD6);
	GC2155_write_cmos_sensor(0x20 , 0xE0);
	GC2155_write_cmos_sensor(0x21 , 0xE7);
	GC2155_write_cmos_sensor(0x22 , 0xED);
	GC2155_write_cmos_sensor(0x23 , 0xF6);
	GC2155_write_cmos_sensor(0x24 , 0xFB);
	GC2155_write_cmos_sensor(0x25 , 0xFF);
	///////////////////gamma2////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x02);
	GC2155_write_cmos_sensor(0x26 , 0x0A);
	GC2155_write_cmos_sensor(0x27 , 0x10);
	GC2155_write_cmos_sensor(0x28 , 0x16);
	GC2155_write_cmos_sensor(0x29 , 0x1C);
	GC2155_write_cmos_sensor(0x2a , 0x27);
	GC2155_write_cmos_sensor(0x2b , 0x34);
	GC2155_write_cmos_sensor(0x2c , 0x44);
	GC2155_write_cmos_sensor(0x2d , 0x55);
	GC2155_write_cmos_sensor(0x2e , 0x6E);
	GC2155_write_cmos_sensor(0x2f , 0x81);
	GC2155_write_cmos_sensor(0x30 , 0x91);
	GC2155_write_cmos_sensor(0x31 , 0x9F);
	GC2155_write_cmos_sensor(0x32 , 0xAA);
	GC2155_write_cmos_sensor(0x33 , 0xBC);
	GC2155_write_cmos_sensor(0x34 , 0xCA);
	GC2155_write_cmos_sensor(0x35 , 0xD6);
	GC2155_write_cmos_sensor(0x36 , 0xE0);
	GC2155_write_cmos_sensor(0x37 , 0xE7);
	GC2155_write_cmos_sensor(0x38 , 0xED);
	GC2155_write_cmos_sensor(0x39 , 0xF6);
	GC2155_write_cmos_sensor(0x3a , 0xFB);
	GC2155_write_cmos_sensor(0x3b , 0xFF);
	/////////////////////////////////////////////// 
	///////////YCP /////////////////////// 
	/////////////////////////////////////////////// 
	GC2155_write_cmos_sensor(0xfe , 0x02);
	GC2155_write_cmos_sensor(0xd1 , 0x20);//28
	GC2155_write_cmos_sensor(0xd2 , 0x20);//28
	GC2155_write_cmos_sensor(0xdd , 0x14);
	GC2155_write_cmos_sensor(0xde , 0x88);
	GC2155_write_cmos_sensor(0xed , 0x80);
	////////////////////////////
	//////// LSC ///////////////
	////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0xc2 , 0x1f);
	GC2155_write_cmos_sensor(0xc3 , 0x13);
	GC2155_write_cmos_sensor(0xc4 , 0x0e);
	GC2155_write_cmos_sensor(0xc8 , 0x16);
	GC2155_write_cmos_sensor(0xc9 , 0x0f);
	GC2155_write_cmos_sensor(0xca , 0x0c);
	GC2155_write_cmos_sensor(0xbc , 0x52);
	GC2155_write_cmos_sensor(0xbd , 0x2c);
	GC2155_write_cmos_sensor(0xbe , 0x27);
	GC2155_write_cmos_sensor(0xb6 , 0x47);
	GC2155_write_cmos_sensor(0xb7 , 0x32);
	GC2155_write_cmos_sensor(0xb8 , 0x30);
	GC2155_write_cmos_sensor(0xc5 , 0x00);
	GC2155_write_cmos_sensor(0xc6 , 0x00);
	GC2155_write_cmos_sensor(0xc7 , 0x00);
	GC2155_write_cmos_sensor(0xcb , 0x00);
	GC2155_write_cmos_sensor(0xcc , 0x00);
	GC2155_write_cmos_sensor(0xcd , 0x00);
	GC2155_write_cmos_sensor(0xbf , 0x0e);
	GC2155_write_cmos_sensor(0xc0 , 0x00);
	GC2155_write_cmos_sensor(0xc1 , 0x00);
	GC2155_write_cmos_sensor(0xb9 , 0x08);
	GC2155_write_cmos_sensor(0xba , 0x00);
	GC2155_write_cmos_sensor(0xbb , 0x00);
	GC2155_write_cmos_sensor(0xaa , 0x0a);
	GC2155_write_cmos_sensor(0xab , 0x0c);
	GC2155_write_cmos_sensor(0xac , 0x0d);
	GC2155_write_cmos_sensor(0xad , 0x02);
	GC2155_write_cmos_sensor(0xae , 0x06);
	GC2155_write_cmos_sensor(0xaf , 0x05);
	GC2155_write_cmos_sensor(0xb0 , 0x00);
	GC2155_write_cmos_sensor(0xb1 , 0x05);
	GC2155_write_cmos_sensor(0xb2 , 0x02);
	GC2155_write_cmos_sensor(0xb3 , 0x04);
	GC2155_write_cmos_sensor(0xb4 , 0x04);
	GC2155_write_cmos_sensor(0xb5 , 0x05);
	GC2155_write_cmos_sensor(0xd0 , 0x00);
	GC2155_write_cmos_sensor(0xd1 , 0x00);
	GC2155_write_cmos_sensor(0xd2 , 0x00);
	GC2155_write_cmos_sensor(0xd6 , 0x02);
	GC2155_write_cmos_sensor(0xd7 , 0x00);
	GC2155_write_cmos_sensor(0xd8 , 0x00);
	GC2155_write_cmos_sensor(0xd9 , 0x00);
	GC2155_write_cmos_sensor(0xda , 0x00);
	GC2155_write_cmos_sensor(0xdb , 0x00);
	GC2155_write_cmos_sensor(0xd3 , 0x00);
	GC2155_write_cmos_sensor(0xd4 , 0x00);
	GC2155_write_cmos_sensor(0xd5 , 0x00);
	GC2155_write_cmos_sensor(0xa4 , 0x04);
	GC2155_write_cmos_sensor(0xa5 , 0x00);
	GC2155_write_cmos_sensor(0xa6 , 0x77);
	GC2155_write_cmos_sensor(0xa7 , 0x77);
	GC2155_write_cmos_sensor(0xa8 , 0x77);
	GC2155_write_cmos_sensor(0xa9 , 0x77);
	GC2155_write_cmos_sensor(0xa1 , 0x80);
	GC2155_write_cmos_sensor(0xa2 , 0x80);

	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0xdc , 0x35);
	GC2155_write_cmos_sensor(0xdd , 0x28);
	GC2155_write_cmos_sensor(0xdf , 0x0d);
	GC2155_write_cmos_sensor(0xe0 , 0x70);
	GC2155_write_cmos_sensor(0xe1 , 0x78);
	GC2155_write_cmos_sensor(0xe2 , 0x70);
	GC2155_write_cmos_sensor(0xe3 , 0x78);
	GC2155_write_cmos_sensor(0xe6 , 0x90);
	GC2155_write_cmos_sensor(0xe7 , 0x70);
	GC2155_write_cmos_sensor(0xe8 , 0x90);
	GC2155_write_cmos_sensor(0xe9 , 0x70);
	GC2155_write_cmos_sensor(0xfe , 0x00);
	///////////////////////////////////////////////
	/////////// AWB////////////////////////
	///////////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x4f , 0x00);
	GC2155_write_cmos_sensor(0x4f , 0x00);
	GC2155_write_cmos_sensor(0x4b , 0x01);
	GC2155_write_cmos_sensor(0x4f , 0x00);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x71);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x91);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x50);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x70);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x90);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xb0);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xd0);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x4f);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x6f);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x8f);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xaf);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xcf);
	GC2155_write_cmos_sensor(0x4e , 0x02);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x6e);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x8e);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xae);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xce);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x4d);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x6d);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x8d);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xad);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xcd);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x4c);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x6c);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x8c);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xac);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xcc);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xec);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x4b);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x6b);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x8b);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xab);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0x8a);
	GC2155_write_cmos_sensor(0x4e , 0x04);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xaa);
	GC2155_write_cmos_sensor(0x4e , 0x04);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xca);
	GC2155_write_cmos_sensor(0x4e , 0x04);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xa9);
	GC2155_write_cmos_sensor(0x4e , 0x04);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xc9);
	GC2155_write_cmos_sensor(0x4e , 0x04);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xcb);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	//GC2155_write_cmos_sensor(0x4c , 0x01);
	//GC2155_write_cmos_sensor(0x4d , 0xeb);
	//GC2155_write_cmos_sensor(0x4e , 0x05);
	//GC2155_write_cmos_sensor(0x4c , 0x02);
	//GC2155_write_cmos_sensor(0x4d , 0x0b);
	//GC2155_write_cmos_sensor(0x4e , 0x05);
	//GC2155_write_cmos_sensor(0x4c , 0x02);
	//GC2155_write_cmos_sensor(0x4d , 0x2b);
	//GC2155_write_cmos_sensor(0x4e , 0x05);
	//GC2155_write_cmos_sensor(0x4c , 0x02);
	//GC2155_write_cmos_sensor(0x4d , 0x4b);
	//GC2155_write_cmos_sensor(0x4e , 0x05);
	GC2155_write_cmos_sensor(0x4c , 0x01);
	GC2155_write_cmos_sensor(0x4d , 0xea);
	GC2155_write_cmos_sensor(0x4e , 0x03);
	//GC2155_write_cmos_sensor(0x4c , 0x02);
	//GC2155_write_cmos_sensor(0x4d , 0x0a);
	//GC2155_write_cmos_sensor(0x4e , 0x05);
	//GC2155_write_cmos_sensor(0x4c , 0x02);
	//GC2155_write_cmos_sensor(0x4d , 0x2a);
	//GC2155_write_cmos_sensor(0x4e , 0x05);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0x6a);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0x29);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0x49);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0x69);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0x89);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0xa9);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0xc9);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0x48);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0x68);
	GC2155_write_cmos_sensor(0x4e , 0x06);
	GC2155_write_cmos_sensor(0x4c , 0x03);
	GC2155_write_cmos_sensor(0x4d , 0x09);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0xa8);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0xc8);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0xe8);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x03);
	GC2155_write_cmos_sensor(0x4d , 0x08);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x03);
	GC2155_write_cmos_sensor(0x4d , 0x28);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0x87);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0xa7);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0xc7);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x02);
	GC2155_write_cmos_sensor(0x4d , 0xe7);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4c , 0x03);
	GC2155_write_cmos_sensor(0x4d , 0x07);
	GC2155_write_cmos_sensor(0x4e , 0x07);
	GC2155_write_cmos_sensor(0x4f , 0x01);
	GC2155_write_cmos_sensor(0xfe , 0x01);
	
	GC2155_write_cmos_sensor(0x50 , 0x80);
	GC2155_write_cmos_sensor(0x51 , 0xa8);
	GC2155_write_cmos_sensor(0x52 , 0x57);
	GC2155_write_cmos_sensor(0x53 , 0x38);
	GC2155_write_cmos_sensor(0x54 , 0xc7);
	GC2155_write_cmos_sensor(0x56 , 0x0e);
	GC2155_write_cmos_sensor(0x58 , 0x08);
	GC2155_write_cmos_sensor(0x5b , 0x00);
	GC2155_write_cmos_sensor(0x5c , 0x74);
	GC2155_write_cmos_sensor(0x5d , 0x8b);
	GC2155_write_cmos_sensor(0x61 , 0xd3);
	GC2155_write_cmos_sensor(0x62 , 0x90);
	GC2155_write_cmos_sensor(0x63 , 0xaa);
	GC2155_write_cmos_sensor(0x65 , 0x04);
	GC2155_write_cmos_sensor(0x67 , 0xb2);
	GC2155_write_cmos_sensor(0x68 , 0xac);
	GC2155_write_cmos_sensor(0x69 , 0x00);
	GC2155_write_cmos_sensor(0x6a , 0xb2);
	GC2155_write_cmos_sensor(0x6b , 0xac);
	GC2155_write_cmos_sensor(0x6c , 0xdc);
	GC2155_write_cmos_sensor(0x6d , 0xb0);
	GC2155_write_cmos_sensor(0x6e , 0x30);
	GC2155_write_cmos_sensor(0x6f , 0x40);
	GC2155_write_cmos_sensor(0x70 , 0x05);
	GC2155_write_cmos_sensor(0x71 , 0x80);
	GC2155_write_cmos_sensor(0x72 , 0x80);
	GC2155_write_cmos_sensor(0x73 , 0xf0);//30
	GC2155_write_cmos_sensor(0x74 , 0x01);
	GC2155_write_cmos_sensor(0x75 , 0x01);
	GC2155_write_cmos_sensor(0x7f , 0x08);
	GC2155_write_cmos_sensor(0x76 , 0x70);
	GC2155_write_cmos_sensor(0x77 , 0x48);
	GC2155_write_cmos_sensor(0x78 , 0xa0);
	GC2155_write_cmos_sensor(0xfe , 0x00);

	//////////////////////////////////////////
	///////////CC////////////////////////
	//////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x02);
	GC2155_write_cmos_sensor(0xc0 , 0x01);
	GC2155_write_cmos_sensor(0xc1 , 0x4a);
	GC2155_write_cmos_sensor(0xc2 , 0xf3);
	GC2155_write_cmos_sensor(0xc3 , 0xfc);
	GC2155_write_cmos_sensor(0xc4 , 0xe4);
	GC2155_write_cmos_sensor(0xc5 , 0x48);
	GC2155_write_cmos_sensor(0xc6 , 0xec);
	GC2155_write_cmos_sensor(0xc7 , 0x45);
	GC2155_write_cmos_sensor(0xc8 , 0xf8);
	GC2155_write_cmos_sensor(0xc9 , 0x02);
	GC2155_write_cmos_sensor(0xca , 0xfe);
	GC2155_write_cmos_sensor(0xcb , 0x42);
	GC2155_write_cmos_sensor(0xcc , 0x00);
	GC2155_write_cmos_sensor(0xcd , 0x45);
	GC2155_write_cmos_sensor(0xce , 0xf0);
	GC2155_write_cmos_sensor(0xcf , 0x00);
	GC2155_write_cmos_sensor(0xe3 , 0xf0);
	GC2155_write_cmos_sensor(0xe4 , 0x45);
	GC2155_write_cmos_sensor(0xe5 , 0xe8); 
	//////////////////////////////////////////
	///////////ABS ////////////////////
	//////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x9f , 0x42);
	GC2155_write_cmos_sensor(0xfe , 0x00); 
	//////////////////////////////////////////
	///////////OUTPUT ////////////////////
	//////////////////////////////////////////
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0xf2 , 0x0f);

	//////////////frame rate 50Hz/////////
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0x05 , 0x01);
	GC2155_write_cmos_sensor(0x06 , 0x56);
	GC2155_write_cmos_sensor(0x07 , 0x00);
	GC2155_write_cmos_sensor(0x08 , 0x32);
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x25 , 0x00);
	GC2155_write_cmos_sensor(0x26 , 0xfa); 
	GC2155_write_cmos_sensor(0x27 , 0x04); 
	GC2155_write_cmos_sensor(0x28 , 0xe2); //20fps 
	GC2155_write_cmos_sensor(0x29 , 0x06); 
	GC2155_write_cmos_sensor(0x2a , 0xd6); //16fps 
	GC2155_write_cmos_sensor(0x2b , 0x07); 
	GC2155_write_cmos_sensor(0x2c , 0xd0); //12fps
	GC2155_write_cmos_sensor(0x2d , 0x0b); 
	GC2155_write_cmos_sensor(0x2e , 0xb8); //8fps
	GC2155_write_cmos_sensor(0xfe , 0x00);
}



static void GC2155_Sensor_SVGA(void)
{
	SENSORDB("GC2155_Sensor_SVGA");
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0xfa , 0x00);
	GC2155_write_cmos_sensor(0xfd , 0x01); 
	//// crop window              
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0x90 , 0x01); 
	GC2155_write_cmos_sensor(0x91 , 0x00);
	GC2155_write_cmos_sensor(0x92 , 0x00);
	GC2155_write_cmos_sensor(0x93 , 0x00);
	GC2155_write_cmos_sensor(0x94 , 0x00);
	GC2155_write_cmos_sensor(0x95 , 0x02);
	GC2155_write_cmos_sensor(0x96 , 0x58);
	GC2155_write_cmos_sensor(0x97 , 0x03);
	GC2155_write_cmos_sensor(0x98 , 0x20);
	GC2155_write_cmos_sensor(0x99 , 0x11);
	GC2155_write_cmos_sensor(0x9a , 0x06);
	//// AWB                      
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0xec , 0x01); 
	GC2155_write_cmos_sensor(0xed , 0x02);
	GC2155_write_cmos_sensor(0xee , 0x30);
	GC2155_write_cmos_sensor(0xef , 0x48);
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x74 , 0x00); 
	//// AEC                      
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x01 , 0x04);
	GC2155_write_cmos_sensor(0x02 , 0x60);
	GC2155_write_cmos_sensor(0x03 , 0x02);
	GC2155_write_cmos_sensor(0x04 , 0x48);
	GC2155_write_cmos_sensor(0x05 , 0x18);
	GC2155_write_cmos_sensor(0x06 , 0x4c);
	GC2155_write_cmos_sensor(0x07 , 0x14);
	GC2155_write_cmos_sensor(0x08 , 0x36);
	GC2155_write_cmos_sensor(0x0a , 0xc0); 
	GC2155_write_cmos_sensor(0x21 , 0x14); 
	GC2155_write_cmos_sensor(0xfe , 0x00);
	//// gamma
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0xc3 , 0x11);
	GC2155_write_cmos_sensor(0xc4 , 0x20);
	GC2155_write_cmos_sensor(0xc5 , 0x30);
	GC2155_write_cmos_sensor(0xfe , 0x00);
	
}

static void GC2155_Sensor_2M(void)
{
	SENSORDB("GC2155_Sensor_2M");
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0xfa , 0x11);
	GC2155_write_cmos_sensor(0xfd , 0x00);
	//// crop window
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0x90 , 0x01);
	GC2155_write_cmos_sensor(0x91 , 0x00);
	GC2155_write_cmos_sensor(0x92 , 0x00);
	GC2155_write_cmos_sensor(0x93 , 0x00);
	GC2155_write_cmos_sensor(0x94 , 0x00);
	GC2155_write_cmos_sensor(0x95 , 0x04);
	GC2155_write_cmos_sensor(0x96 , 0xb0);
	GC2155_write_cmos_sensor(0x97 , 0x06);
	GC2155_write_cmos_sensor(0x98 , 0x40);
	GC2155_write_cmos_sensor(0x99 , 0x11); 
	GC2155_write_cmos_sensor(0x9a , 0x06);
	//// AWB   
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0xec , 0x02);
	GC2155_write_cmos_sensor(0xed , 0x04);
	GC2155_write_cmos_sensor(0xee , 0x60);
	GC2155_write_cmos_sensor(0xef , 0x90);
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x74 , 0x01);
	//// AEC	  
	GC2155_write_cmos_sensor(0xfe , 0x01);
	GC2155_write_cmos_sensor(0x01 , 0x08);
	GC2155_write_cmos_sensor(0x02 , 0xc0);
	GC2155_write_cmos_sensor(0x03 , 0x04);
	GC2155_write_cmos_sensor(0x04 , 0x90);
	GC2155_write_cmos_sensor(0x05 , 0x30);
	GC2155_write_cmos_sensor(0x06 , 0x98);
	GC2155_write_cmos_sensor(0x07 , 0x28);
	GC2155_write_cmos_sensor(0x08 , 0x6c);
	GC2155_write_cmos_sensor(0x0a , 0xc2); 
	GC2155_write_cmos_sensor(0x21 , 0x15); //if 0xfa=11,then 0x21=15;else if 0xfa=00,then 0x21=14
	GC2155_write_cmos_sensor(0xfe , 0x00);
	//// gamma
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_cmos_sensor(0xc3 , 0x00); //if shutter/2 when capture,then exp_gamma_th/2
	GC2155_write_cmos_sensor(0xc4 , 0x90);
	GC2155_write_cmos_sensor(0xc5 , 0x98);
	GC2155_write_cmos_sensor(0xfe , 0x00);
}


/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	GC2155Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC2155Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	zoom_factor = 0; 
	Sleep(10);


	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id=((GC2155_read_cmos_sensor(0xf0) << 8) | GC2155_read_cmos_sensor(0xf1));   
		SENSORDB("GC2155_Open, sensor_id:%x \n",sensor_id);
		if (sensor_id != GC2155_SENSOR_ID)
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
		SENSORDB("GC2155 Sensor Read ID OK \r\n");
		GC2155_Sensor_Init();
	
	Preview_Shutter =GC2155_read_shutter();
	
	return ERROR_NONE;
}	/* GC2155Open() */

/*************************************************************************
* FUNCTION
*	GC2155Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC2155Close(void)
{
//	CISModulePowerOn(FALSE);
	return ERROR_NONE;
}	/* GC2155Close() */

/*************************************************************************
* FUNCTION
*	GC2155Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC2155Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint8 iTemp, temp_AE_reg, temp_AWB_reg;
	kal_uint16 iDummyPixels = 0, iDummyLines = 0, iStartX = 0, iStartY = 0;

	SENSORDB("GC2155Previe\n");

	GC2155_sensor_cap_state = KAL_FALSE;

	GC2155_Sensor_SVGA();
	GC2155_write_cmos_sensor(0xfe , 0x02);
    GC2155_write_cmos_sensor(0xd1 , 0x20);
	GC2155_write_cmos_sensor(0xd2 , 0x20);
	GC2155_write_cmos_sensor(0xfe , 0x00);
	GC2155_write_shutter(Preview_Shutter);
    Sleep(500);
	GC2155_set_AE_mode(KAL_TRUE); 

	memcpy(&GC2155SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* GC2155Preview() */




UINT32 GC2155Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    volatile kal_uint32 shutter = GC2155_exposure_lines, temp_reg;
    kal_uint8 temp_AE_reg, temp;
    kal_uint16 AE_setting_delay = 0;

    SENSORDB("GC2155Capture\n");

  if(GC2155_sensor_cap_state == KAL_FALSE)
 	{

	     GC2155_write_cmos_sensor(0xfe , 0x02);//28
         GC2155_write_cmos_sensor(0xd1 , 0x28);//28
	     GC2155_write_cmos_sensor(0xd2 , 0x28);//28
	     GC2155_write_cmos_sensor(0xfe , 0x00);//28
		// turn off AEC/AGC
		GC2155_set_AE_mode(KAL_FALSE);
		
		shutter = GC2155_read_shutter();
		Preview_Shutter = shutter;
		
		GC2155_Sensor_2M();
		
		Capture_Shutter = shutter / 2; 
		// set shutter
		GC2155_write_shutter(Capture_Shutter);
		Sleep(200);
      }

    	 GC2155_sensor_cap_state = KAL_TRUE;

	image_window->GrabStartX=1;
        image_window->GrabStartY=1;
        image_window->ExposureWindowWidth=GC2155_IMAGE_SENSOR_FULL_WIDTH - image_window->GrabStartX;
        image_window->ExposureWindowHeight=GC2155_IMAGE_SENSOR_FULL_HEIGHT -image_window->GrabStartY;    	 

    memcpy(&GC2155SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* GC2155Capture() */



UINT32 GC2155GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=GC2155_IMAGE_SENSOR_FULL_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorFullHeight=GC2155_IMAGE_SENSOR_FULL_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	pSensorResolution->SensorPreviewWidth=GC2155_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorPreviewHeight=GC2155_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	pSensorResolution->SensorVideoWidth=GC2155_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorVideoHeight=GC2155_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	return ERROR_NONE;
}	/* GC2155GetResolution() */

UINT32 GC2155GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=GC2155_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=GC2155_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=GC2155_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=GC2155_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->CaptureDelayFrame = 4; 
	pSensorInfo->PreviewDelayFrame = 4; //1
	pSensorInfo->VideoDelayFrame = 0; 
       pSensorInfo->YUVAwbDelayFrame = 2;  // add by lanking
	pSensorInfo->YUVEffectDelayFrame = 2;  // add by lanking
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA; //lilong 20141215 modefy,ISP_DRIVING_6MA -> ISP_DRIVING_8MA
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
                     pSensorInfo->SensorGrabStartX = 2; 
                     pSensorInfo->SensorGrabStartY = 2;
	
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
                     pSensorInfo->SensorGrabStartX = 2; 
                     pSensorInfo->SensorGrabStartY = 2;

		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
                     pSensorInfo->SensorGrabStartX = 2; 
                     pSensorInfo->SensorGrabStartY = 2;             
			
		break;
	}
	memcpy(pSensorConfigData, &GC2155SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* GC2155GetInfo() */


UINT32 GC2155Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			GC2155Preview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD: ///added by maxyu 140926 for ��������
			GC2155Capture(pImageWindow, pSensorConfigData);
		break;
		default:
		    break; 
	}
	return TRUE;
}	/* GC2155Control() */

BOOL GC2155_set_param_wb(UINT16 para)
{
	switch (para)
	{
		case AWB_MODE_AUTO:
			GC2155_set_AWB_mode(KAL_TRUE);
		break;
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			GC2155_set_AWB_mode(KAL_FALSE);
			GC2155_write_cmos_sensor(0xb3, 0x58);
			GC2155_write_cmos_sensor(0xb4, 0x40);
			GC2155_write_cmos_sensor(0xb5, 0x50);
		break;
		case AWB_MODE_DAYLIGHT: //sunny
			GC2155_set_AWB_mode(KAL_FALSE);
			GC2155_write_cmos_sensor(0xb3, 0x70);
			GC2155_write_cmos_sensor(0xb4, 0x40);
			GC2155_write_cmos_sensor(0xb5, 0x50);
		break;
		case AWB_MODE_INCANDESCENT: //office
			GC2155_set_AWB_mode(KAL_FALSE);
			GC2155_write_cmos_sensor(0xb3, 0x50);
			GC2155_write_cmos_sensor(0xb4, 0x40);
			GC2155_write_cmos_sensor(0xb5, 0xa8);
		break;
		case AWB_MODE_TUNGSTEN: //home
			GC2155_set_AWB_mode(KAL_FALSE);
			GC2155_write_cmos_sensor(0xb3, 0xa0);
			GC2155_write_cmos_sensor(0xb4, 0x45);
			GC2155_write_cmos_sensor(0xb5, 0x40);
		break;
		case AWB_MODE_FLUORESCENT:
			GC2155_set_AWB_mode(KAL_FALSE);
			GC2155_write_cmos_sensor(0xb3, 0x72);
			GC2155_write_cmos_sensor(0xb4, 0x40);
			GC2155_write_cmos_sensor(0xb5, 0x5b);
		break;	
		default:
		return FALSE;
	}
	return TRUE;
} /* GC2155_set_param_wb */

BOOL GC2155_set_param_effect(UINT16 para)
{
	kal_uint32 ret = KAL_TRUE;
	switch (para)
	{
		case MEFFECT_OFF:
			GC2155_write_cmos_sensor(0xfe, 0x00);
			GC2155_write_cmos_sensor(0x83, 0xe0);
		break;

		case MEFFECT_SEPIA:
			GC2155_write_cmos_sensor(0xfe, 0x00);
			GC2155_write_cmos_sensor(0x83, 0x82);
		break;  

		case MEFFECT_NEGATIVE:		
			GC2155_write_cmos_sensor(0xfe, 0x00);
			GC2155_write_cmos_sensor(0x83, 0x01);
		break; 

		case MEFFECT_SEPIAGREEN:		
			GC2155_write_cmos_sensor(0xfe, 0x00);
			GC2155_write_cmos_sensor(0x83, 0x52);
		break;

		case MEFFECT_SEPIABLUE:	
			GC2155_write_cmos_sensor(0xfe, 0x00);
			GC2155_write_cmos_sensor(0x83, 0x62);
		break;

		case MEFFECT_MONO:				
			GC2155_write_cmos_sensor(0xfe, 0x00);
			GC2155_write_cmos_sensor(0x83, 0x12);
		break;

		default:
		return FALSE;
	}

	return ret;
} /* GC2155_set_param_effect */

BOOL GC2155_set_param_banding(UINT16 para)
{
    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
			
		GC2155_write_cmos_sensor(0xfe , 0x00);
		GC2155_write_cmos_sensor(0x05 , 0x01);
		GC2155_write_cmos_sensor(0x06 , 0x56);
		GC2155_write_cmos_sensor(0x07 , 0x00);
		GC2155_write_cmos_sensor(0x08 , 0x32);
		GC2155_write_cmos_sensor(0xfe , 0x01);
		GC2155_write_cmos_sensor(0x25 , 0x00);
		GC2155_write_cmos_sensor(0x26 , 0xfa); 
		GC2155_write_cmos_sensor(0x27 , 0x05); 
		GC2155_write_cmos_sensor(0x28 , 0xdc); //20fps 
		GC2155_write_cmos_sensor(0x29 , 0x07); 
		GC2155_write_cmos_sensor(0x2a , 0xd0); //14fps 
		GC2155_write_cmos_sensor(0x2b , 0x0b); 
		GC2155_write_cmos_sensor(0x2c , 0xb8); //12fps
		GC2155_write_cmos_sensor(0x2d , 0x0e); 
		GC2155_write_cmos_sensor(0x2e , 0xa6); //8fps
		GC2155_write_cmos_sensor(0xfe , 0x00);
            break;

        case AE_FLICKER_MODE_60HZ:
		GC2155_write_cmos_sensor(0xfe, 0x00);
		GC2155_write_cmos_sensor(0x05, 0x01);
		GC2155_write_cmos_sensor(0x06, 0x58);
		GC2155_write_cmos_sensor(0x07, 0x00);
		GC2155_write_cmos_sensor(0x08, 0x32);
		GC2155_write_cmos_sensor(0xfe, 0x01);
		GC2155_write_cmos_sensor(0x25, 0x00);
		GC2155_write_cmos_sensor(0x26, 0xd0); 
		GC2155_write_cmos_sensor(0x27, 0x06);
		GC2155_write_cmos_sensor(0x28, 0x80); //20fps 
		GC2155_write_cmos_sensor(0x29, 0x08);
		GC2155_write_cmos_sensor(0x2a, 0x20); //16fps 
		GC2155_write_cmos_sensor(0x2b, 0x0b);
		GC2155_write_cmos_sensor(0x2c, 0x60); //16fps  
		GC2155_write_cmos_sensor(0x2d, 0x0e);
		GC2155_write_cmos_sensor(0x2e, 0xa0); //8fps 
		GC2155_write_cmos_sensor(0xfe, 0x00); 
            break;

          default:
              return FALSE;
    }

    return TRUE;
} /* GC2155_set_param_banding */

BOOL GC2155_set_param_exposure(UINT16 para)
{
	switch (para)
	{
		case AE_EV_COMP_n13:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x10);
			GC2155_SET_PAGE0;
		break;
		case AE_EV_COMP_n10:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x15);
			GC2155_SET_PAGE0;
		break;
		case AE_EV_COMP_n07:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x20);
			GC2155_SET_PAGE0;
		break;
		case AE_EV_COMP_n03:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x25);
			GC2155_SET_PAGE0;
		break;
		case AE_EV_COMP_00:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x2d);
			GC2155_SET_PAGE0;
		break;
		case AE_EV_COMP_03:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x35);
			GC2155_SET_PAGE0;
		break;
		case AE_EV_COMP_07:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x40);
			GC2155_SET_PAGE0;
		break;
		case AE_EV_COMP_10:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x45);
			GC2155_SET_PAGE0;
		break;
		case AE_EV_COMP_13:
			GC2155_SET_PAGE1;
			GC2155_write_cmos_sensor(0x13,0x50);
			GC2155_SET_PAGE0;
		break;
		default:
		return FALSE;
	}
	return TRUE;
} /* GC2155_set_param_exposure */

UINT32 GC2155YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
//   if( GC2155_sensor_cap_state == KAL_TRUE)
//	   return TRUE;

	switch (iCmd) {
	case FID_SCENE_MODE:	    
//	    printk("Set Scene Mode:%d\n", iPara); 
	    if (iPara == SCENE_MODE_OFF)
	    {
	        GC2155_night_mode(0); 
	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
               GC2155_night_mode(1); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
	    printk("Set AWB Mode:%d\n", iPara); 	    
           GC2155_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
	    printk("Set Color Effect:%d\n", iPara); 	    	    
           GC2155_set_param_effect(iPara);
	break;
	case FID_AE_EV:
           printk("Set EV:%d\n", iPara); 	    	    
           GC2155_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
          printk("Set Flicker:%d\n", iPara); 	    	    	    
           GC2155_set_param_banding(iPara);
	break;
        case FID_AE_SCENE_MODE: 
            if (iPara == AE_MODE_OFF) {
                GC2155_set_AE_mode(KAL_FALSE);
            }
            else {
                GC2155_set_AE_mode(KAL_TRUE);
	    }
            break; 
	case FID_ZOOM_FACTOR:
	    zoom_factor = iPara; 
        break; 
	default:
	break;
	}
	return TRUE;
}   /* GC2155YUVSensorSetting */

UINT32 GC2155YUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    /* to fix VSYNC, to fix frame rate */
    //printk("Set YUV Video Mode \n");  

    if (u2FrameRate == 30)
    {
    }
    else if (u2FrameRate == 15)       
    {
    }
    else 
    {
        printk("Wrong frame rate setting \n");
    }
    GC2155_VEDIO_encode_mode = KAL_TRUE; 
        
    return TRUE;
}

#define FLASH_BV_THRESHOLD 0x25   //0x10
static void GC2155_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
	unsigned int NormBr;	   
	GC2155_write_cmos_sensor(0xfe,  0x01);
	NormBr = GC2155_read_cmos_sensor(0x14); 
	GC2155_write_cmos_sensor(0xfe,  0x00);
	SENSORDB("GC2155_FlashTriggerCheck reg14 = %x",NormBr);
	if (NormBr > FLASH_BV_THRESHOLD)
	{
	   *pFeatureReturnPara32 = FALSE;
		return;
	}
	*pFeatureReturnPara32 = TRUE;
	return;
}

UINT32 GC2155FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=GC2155_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=GC2155_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=GC2155_IMAGE_SENSOR_PV_WIDTH;
			*pFeatureReturnPara16=GC2155_IMAGE_SENSOR_PV_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			//*pFeatureReturnPara32 = GC2155_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			GC2155_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			GC2155_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			GC2155_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = GC2155_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &GC2155SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
	    case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
            GC2155_FlashTriggerCheck(pFeatureData32);
            break;		
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			 GC2155_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		       //printk("GC2155 YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			GC2155YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       GC2155YUVSetVideoMode(*pFeatureData16);
		       break; 
		 case SENSOR_FEATURE_SET_TEST_PATTERN:
		   GC2155SetTestPatternMode((BOOL)*pFeatureData16);
		   break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing
			*pFeatureReturnPara32= GC2155_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break; 
		default:
			break;			
	}
	return ERROR_NONE;
}	/* GC2155FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncGC2155=
{
	GC2155Open,
	GC2155GetInfo,
	GC2155GetResolution,
	GC2155FeatureControl,
	GC2155Control,
	GC2155Close
};

UINT32 GC2155_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncGC2155;

	return ERROR_NONE;
}	/* SensorInit() */
