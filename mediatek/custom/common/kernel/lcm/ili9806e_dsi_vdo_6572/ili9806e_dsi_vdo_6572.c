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
#ifdef BUILD_LK
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (800)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID_ILI9806E       (0x0604)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};



static struct LCM_setting_table lcm_initialization_setting_ips2_lg[] = {
/*
// EXTC Command Set enable register
{0xFF,3,{0xFF,0x98,0x16}},
// SPI Interface Setting
{0xBA,1, {0x60}},
 // Interface Mode Control
{0XB0,1,{0x01}},
 // Display Function Control
//{0xB6,1,{0x32}},//22

 // GIP 1
{0xBC,23,{0x01,0x0D,0x61,0xFF,0x01,0x01,0x0B,0x0e,0x6a,0x13,0x0e,0x6a,0x01,0x01,0x00,0x00,0x55,0X50,0x00,0x0c,0x0d,0x43,0X0b,}},

 // GIP 2
{0xBD,8,{0x01,0x23,0x45,0x67,0x01,0x23,0x45,0x67}},

 // GIP 3
{0xBE,17,{0x02,0x22,0x11,0xAA,0xBB,0x66,0x00,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22}},

 // en_volt_reg measure VGMP
{0xED,2,{0x7F,0x0F}},

{0xF3,1,{0x70}},

 // Panel Control
{0XB9,1,{0x08}},//08

 // Display Inversion Control
{0XB4,1,{0x02}},  //00

 // Power Control 1


 // Power Control 2
{0XC1,4,{0x17,0x8A,0x83,0x10}},

 // VGLO Selection
{0XD8,1,{0x50}},

 // VGLO Selection
{0XFC,1,{0x07}},//07

 // Positive Gamma Control
{0XE0,16,{0x00,0x16,0x19,0x0C,0x0E,0x00,0XC7,0x07,0x04,0x08,0x10,0x0F,0x10,0x1C,0x15,0x00}},

 // Negative Gamma Control
{0XE1,16,{0x00,0x01,0x0F,0x0D,0x0F,0x18,0X79,0x06,0x07,0x0D,0x0C,0x0E,0x0B,0x10,0x0B,0x00}},

 // Source Timing Adjust
{0XD5,8,{0x0C,0x09,0x09,0x07,0xCB,0XA5,0x01,0x04}},


{0XC0,3,{0x0F,0x0B,0x0a}},

 // Resolution
{0XF7,1,{0x8A}},

 // Vcom
{0XC7,1,{0x45}},//5A  70

{0X36,1,{0x08}},

{0x3a, 1, {0x77}},

{0x11, 1, {0x00}},
{REGFLAG_DELAY, 120, {}},
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 120, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
*/
{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},// Change to Page 1
//delay_ms(1),
{0x08,1,{0x10}}, // output SDA
//delay_ms(1),
{0x21,1,{0x01}}, // DE = 1 Active
//delay_ms(1),
{0x30,1,{0x02}},// 480 X 800
//delay_ms(1),
{0x31,1,{0x02}},// Column Inversion
//delay_ms(1),
{0x60,1,{0x07}},// SDTI
//delay_ms(1),
{0x61,1,{0x06}}, // CRTI
//delay_ms(1),
{0x62,1,{0x06}}, // EQTI
//delay_ms(1),
{0x63,1,{0x04}}, // PCTI
//delay_ms(1),
{0x40,1,{0x15}}, // BT  +2.5/-2.5 pump for DDVDH-L
//delay_ms(1),
{0x41,1,{0x55}},// DVDDH DVDDL clamp  
//delay_ms(1), 
{0x42,1,{0x02}}, // VGH/VGL 
//delay_ms(1),
{0x43,1,{0x89}}, // VGH/VGL 
//delay_ms(1),
{0x44,1,{0x8C}}, // VGH/VGL 
//delay_ms(1),
{0x45,1,{0x1B}},  // VGL_REG  -10V 
//delay_ms(1),
{0x50,1,{0x80}},   // VGMP
//delay_ms(1),
{0x51,1,{0x80}},  // VGMN
//delay_ms(1),
{0x52,1,{0x00}}, //Flicker
//delay_ms(1),
{0x53,1,{0x83}},  //Flicker4F
//delay_ms(1),
//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
{0xA0,1,{0x00}},  // Gamma 0 /255
//delay_ms(1),
{0xA1,1,{0x09}},  // Gamma 4 /251
//delay_ms(1),
{0xA2,1,{0x10}},  // Gamma 8 /247
//delay_ms(1),
{0xA3,1,{0x0E}},  // Gamma 16	/239
//delay_ms(1),
{0xA4,1,{0x05}},  // Gamma 24 /231
//delay_ms(1),
{0xA5,1,{0x0a}},  // Gamma 52 / 203
//delay_ms(1),
{0xA6,1,{0x0B}},  // Gamma 80 / 175
//delay_ms(1),
{0xA7,1,{0x0C}},  // Gamma 108 /147
//delay_ms(1),
{0xA8,1,{0x0D}},  // Gamma 147 /108
//delay_ms(1),
{0xA9,1,{0x0F}},  // Gamma 175 / 80
//delay_ms(1),
{0xAA,1,{0x13}}, // Gamma 203 / 52
//delay_ms(1),
{0xAB,1,{0x0A}},  // Gamma 231 / 24
//delay_ms(1),
{0xAC,1,{0x14}},  // Gamma 239 / 16
//delay_ms(1),
{0xAD,1,{0x13}},  // Gamma 247 / 8
//delay_ms(1),
{0xAE,1,{0x12}},  // Gamma 251 / 4
//delay_ms(1),
{0xAF,1,{0x00}},  // Gamma 255 / 0
//delay_ms(1),
///==============Nagitive
{0xC0,1,{0x00}},  // Gamma 0 
//delay_ms(1),
{0xC1,1,{0x0E}},  // Gamma 4
//delay_ms(1),
{0xC2,1,{0x16}},  // Gamma 8
//delay_ms(1),
{0xC3,1,{0x0E}},  // Gamma 16
//delay_ms(1),
{0xC4,1,{0x06}},  // Gamma 24
//delay_ms(1),
{0xC5,1,{0x0a}},  // Gamma 52
//delay_ms(1),
{0xC6,1,{0x02}},  // Gamma 80
//delay_ms(1),
{0xC7,1,{0x00}},  // Gamma 108
//delay_ms(1),
{0xC8,1,{0x06}}, // Gamma 147
//delay_ms(1),
{0xC9,1,{0x0B}},  // Gamma 175
//delay_ms(1),
{0xCA,1,{0x0a}},  // Gamma 203
//delay_ms(1),
{0xCB,1,{0x0c}},  // Gamma 231
//delay_ms(1),
{0xCC,1,{0x0e}},  // Gamma 239
//delay_ms(1),
{0xCD,1,{0x15}},  // Gamma 247
//delay_ms(1),
{0xCE,1,{0x18}},  // Gamma 251
//delay_ms(1),
{0xCF,1,{0x00}},  // Gamma 255
//delay_ms(1),

{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     // Change to Page 7
//delay_ms(1),
{0x18,1,{0x1D}},
//delay_ms(1),
{0x17,1,{0x12}},  // VGL_REG ON
//delay_ms(1),

//+++++++++++++++++++++++++++++++++++++++++++++++++++//


{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6
//delay_ms(1),
{0x00,1,{0xa0}},
//delay_ms(1),
{0x01,1,{0x05}},
//delay_ms(1),
{0x02,1,{0x00}},    
//delay_ms(1),
{0x03,1,{0x00}},
//delay_ms(1),
{0x04,1,{0x01}},
//delay_ms(1),
{0x05,1,{0x01}},
//delay_ms(1),
{0x06,1,{0x88}},   
//delay_ms(1),
{0x07,1,{0x04}},
//delay_ms(1),
{0x08,1,{0x01}},
//delay_ms(1),
{0x09,1,{0x90}},    
//delay_ms(1),
{0x0A,1,{0x04}},    
//delay_ms(1),
{0x0B,1,{0x01}},    
//delay_ms(1),
{0x0C,1,{0x01}},
//delay_ms(1),
{0x0D,1,{0x01}},
//delay_ms(1),
{0x0E,1,{0x00}},
//delay_ms(1),
{0x0F,1,{0x00}},
//delay_ms(1),
{0x10,1,{0x55}},
//delay_ms(1),
{0x11,1,{0x50}},
//delay_ms(1),
{0x12,1,{0x01}},
//delay_ms(1),
{0x13,1,{0x85}},
//delay_ms(1),
{0x14,1,{0x85}},
//delay_ms(1),
{0x15,1,{0xc0}},
//delay_ms(1),
{0x16,1,{0x0B}},
//delay_ms(1),
{0x17,1,{0x00}},
//delay_ms(1),
{0x18,1,{0x00}},
//delay_ms(1),
{0x19,1,{0x00}},
//delay_ms(1),
{0x1A,1,{0x00}},
//delay_ms(1),
{0x1B,1,{0x00}},
//delay_ms(1),
{0x1C,1,{0x00}},
//delay_ms(1),
{0x1D,1,{0x00}},
//delay_ms(1),

{0x20,1,{0x01}},
//delay_ms(1),
{0x21,1,{0x23}},
//delay_ms(1),
{0x22,1,{0x45}},
//delay_ms(1),
{0x23,1,{0x67}},
//delay_ms(1),
{0x24,1,{0x01}},
//delay_ms(1),
{0x25,1,{0x23}},
//delay_ms(1),
{0x26,1,{0x45}},
//delay_ms(1),
{0x27,1,{0x67}},
//delay_ms(1),

{0x30,1,{0x02}},
//delay_ms(1),
{0x31,1,{0x22}},
//delay_ms(1),
{0x32,1,{0x11}},
//delay_ms(1),
{0x33,1,{0xAA}},
//delay_ms(1),
{0x34,1,{0xBB}},
//delay_ms(1),
{0x35,1,{0x66}},
//delay_ms(1),
{0x36,1,{0x00}},
//delay_ms(1),
{0x37,1,{0x22}},
//delay_ms(1),
{0x38,1,{0x22}},
//delay_ms(1),
{0x39,1,{0x22}},
//delay_ms(1),
{0x3A,1,{0x22}},
//delay_ms(1),
{0x3B,1,{0x22}},
//delay_ms(1),
{0x3C,1,{0x22}},
//delay_ms(1),
{0x3D,1,{0x22}},
//delay_ms(1),
{0x3E,1,{0x22}},
//delay_ms(1),
{0x3F,1,{0x22}},
//delay_ms(1),
{0x40,1,{0x22}},
//delay_ms(1),
{0x53,1,{0x1A}},  //VGLO refer VGL_REG
//delay_ms(1),

{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page 0
//delay_ms(1),
{0x11,1,{0x00}},                 // Sleep-Out
{REGFLAG_DELAY, 120, {}},
{0x29,1,{0x00}},                // Display On
{REGFLAG_DELAY, 120, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_initialization_setting_zgd_boe[] = {///added by maxyu 140429 for S1025/S1021

{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},// Change to Page 1
//delay_ms(1),
{0x08,1,{0x10}}, // output SDA
//delay_ms(1),
{0x21,1,{0x01}}, // DE = 1 Active
//delay_ms(1),
{0x30,1,{0x02}},// 480 X 800
//delay_ms(1),
{0x31,1,{0x02}},// Column Inversion
//delay_ms(1),
{0x60,1,{0x07}},// SDTI
//delay_ms(1),
{0x61,1,{0x06}}, // CRTI
//delay_ms(1),
{0x62,1,{0x06}}, // EQTI
//delay_ms(1),
{0x63,1,{0x04}}, // PCTI
//delay_ms(1),
{0x40,1,{0x14}}, // BT  +2.5/-2.5 pump for DDVDH-L
//delay_ms(1),
{0x41,1,{0x44}},// DVDDH DVDDL clamp  
//delay_ms(1), 
{0x42,1,{0x01}}, // VGH/VGL 
//delay_ms(1),
{0x43,1,{0x09}}, // VGH/VGL 
//delay_ms(1),
{0x44,1,{0x0A}}, // VGH/VGL 
//delay_ms(1),
{0x45,1,{0x1B}},  // VGL_REG  -10V 
{0x46,1,{0x44}},  // VGL_REG  -10V 
{0x47,1,{0x44}},  // VGL_REG  -10V 
{0x50,1,{0x78}},   //0x85 // VGMP
//delay_ms(1),
{0x51,1,{0x78}},  ///0x85 // VGMN
//delay_ms(1),
{0x52,1,{0x00}}, //Flicker
//delay_ms(1),
{0x53,1,{0x68}},  //64//Flicker4F

{0x57,1,{0x50}},
//delay_ms(1),
//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
{0xA0,1,{0x00}},  // Gamma 0 /255
//delay_ms(1),
{0xA1,1,{0x00}},  // Gamma 4 /251
//delay_ms(1),
{0xA2,1,{0x03}},  // Gamma 8 /247
//delay_ms(1),
{0xA3,1,{0x0E}},  // Gamma 16	/239
//delay_ms(1),
{0xA4,1,{0x08}},  // Gamma 24 /231
//delay_ms(1),
{0xA5,1,{0x1F}},  // Gamma 52 / 203
//delay_ms(1),
{0xA6,1,{0x0F}},  // Gamma 80 / 175
//delay_ms(1),
{0xA7,1,{0x0B}},  // Gamma 108 /147
//delay_ms(1),
{0xA8,1,{0x03}},  // Gamma 147 /108
//delay_ms(1),
{0xA9,1,{0x06}},  // Gamma 175 / 80
//delay_ms(1),
{0xAA,1,{0x05}}, // Gamma 203 / 52
//delay_ms(1),
{0xAB,1,{0x02}},  // Gamma 231 / 24
//delay_ms(1),
{0xAC,1,{0x0E}},  // Gamma 239 / 16
//delay_ms(1),
{0xAD,1,{0x25}},  // Gamma 247 / 8
//delay_ms(1),
{0xAE,1,{0x1D}},  // Gamma 251 / 4
//delay_ms(1),
{0xAF,1,{0x00}},  // Gamma 255 / 0
//delay_ms(1),
///==============Nagitive
{0xC0,1,{0x00}},  // Gamma 0 
//delay_ms(1),
{0xC1,1,{0x04}},  // Gamma 4
//delay_ms(1),
{0xC2,1,{0x0F}},  // Gamma 8
//delay_ms(1),
{0xC3,1,{0x10}},  // Gamma 16
//delay_ms(1),
{0xC4,1,{0x0B}},  // Gamma 24
//delay_ms(1),
{0xC5,1,{0x1E}},  // Gamma 52
//delay_ms(1),
{0xC6,1,{0x09}},  // Gamma 80
//delay_ms(1),
{0xC7,1,{0x0A}},  // Gamma 108
//delay_ms(1),
{0xC8,1,{0x00}}, // Gamma 147
//delay_ms(1),
{0xC9,1,{0x0A}},  // Gamma 175
//delay_ms(1),
{0xCA,1,{0x01}},  // Gamma 203
//delay_ms(1),
{0xCB,1,{0x06}},  // Gamma 231
//delay_ms(1),
{0xCC,1,{0x09}},  // Gamma 239
//delay_ms(1),
{0xCD,1,{0x2A}},  // Gamma 247
//delay_ms(1),
{0xCE,1,{0x28}},  // Gamma 251
//delay_ms(1),
{0xCF,1,{0x00}},  // Gamma 255
//delay_ms(1),

//+++++++++++++++++++++++++++++++++++++++++++++++++++//


{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6
//delay_ms(1),
{0x00,1,{0xa0}},
//delay_ms(1),
{0x01,1,{0x05}},
//delay_ms(1),
{0x02,1,{0x00}},    
//delay_ms(1),
{0x03,1,{0x00}},
//delay_ms(1),
{0x04,1,{0x01}},
//delay_ms(1),
{0x05,1,{0x01}},
//delay_ms(1),
{0x06,1,{0x88}},   
//delay_ms(1),
{0x07,1,{0x04}},
//delay_ms(1),
{0x08,1,{0x01}},
//delay_ms(1),
{0x09,1,{0x90}},    
//delay_ms(1),
{0x0A,1,{0x04}},    
//delay_ms(1),
{0x0B,1,{0x01}},    
//delay_ms(1),
{0x0C,1,{0x01}},
//delay_ms(1),
{0x0D,1,{0x01}},
//delay_ms(1),
{0x0E,1,{0x00}},
//delay_ms(1),
{0x0F,1,{0x00}},
//delay_ms(1),
{0x10,1,{0x55}},
//delay_ms(1),
{0x11,1,{0x50}},
//delay_ms(1),
{0x12,1,{0x01}},
//delay_ms(1),
{0x13,1,{0x85}},
//delay_ms(1),
{0x14,1,{0x85}},
//delay_ms(1),
{0x15,1,{0xc0}},
//delay_ms(1),
{0x16,1,{0x0B}},
//delay_ms(1),
{0x17,1,{0x00}},
//delay_ms(1),
{0x18,1,{0x00}},
//delay_ms(1),
{0x19,1,{0x00}},
//delay_ms(1),
{0x1A,1,{0x00}},
//delay_ms(1),
{0x1B,1,{0x00}},
//delay_ms(1),
{0x1C,1,{0x00}},
//delay_ms(1),
{0x1D,1,{0x00}},
//delay_ms(1),

{0x20,1,{0x01}},
//delay_ms(1),
{0x21,1,{0x23}},
//delay_ms(1),
{0x22,1,{0x45}},
//delay_ms(1),
{0x23,1,{0x67}},
//delay_ms(1),
{0x24,1,{0x01}},
//delay_ms(1),
{0x25,1,{0x23}},
//delay_ms(1),
{0x26,1,{0x45}},
//delay_ms(1),
{0x27,1,{0x67}},
//delay_ms(1),

{0x30,1,{0x02}},
//delay_ms(1),
{0x31,1,{0x22}},
//delay_ms(1),
{0x32,1,{0x11}},
//delay_ms(1),
{0x33,1,{0xAA}},
//delay_ms(1),
{0x34,1,{0xBB}},
//delay_ms(1),
{0x35,1,{0x66}},
//delay_ms(1),
{0x36,1,{0x00}},
//delay_ms(1),
{0x37,1,{0x22}},
//delay_ms(1),
{0x38,1,{0x22}},
//delay_ms(1),
{0x39,1,{0x22}},
//delay_ms(1),
{0x3A,1,{0x22}},
//delay_ms(1),
{0x3B,1,{0x22}},
//delay_ms(1),
{0x3C,1,{0x22}},
//delay_ms(1),
{0x3D,1,{0x22}},
//delay_ms(1),
{0x3E,1,{0x22}},
//delay_ms(1),
{0x3F,1,{0x22}},
//delay_ms(1),
{0x40,1,{0x22}},
//delay_ms(1),
{0x53,1,{0x10}},  //VGLO refer VGL_REG
//delay_ms(1),

{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     // Change to Page 7
//delay_ms(1),
{0x18,1,{0x1D}},
//delay_ms(1),
{0x17,1,{0x22}},
//delay_ms(1),
{0x02,1,{0x77}},
//delay_ms(1),
{0xE1,1,{0x79}},
//delay_ms(1),
{0x06,1,{0x13}},
//delay_ms(1),
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page 0
//delay_ms(1),
{0x11,1,{0x00}},                 // Sleep-Out
{REGFLAG_DELAY, 120, {}},
{0x29,1,{0x00}},                // Display On
{REGFLAG_DELAY, 10, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}

};


static struct LCM_setting_table lcm_sleep_out_setting_bak[] = {
{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page
{0x40,1,{0x14}},
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page

    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting_bak[] = {

{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page
{0x28, 1, {0x00}},
{REGFLAG_DELAY, 120, {}},
{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page

    // Display off sequence
{0x40, 1, {0x95}},
{REGFLAG_DELAY, 20, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page

    // Display off sequence
	{0x40, 1, {0x14}},
	{REGFLAG_DELAY, 20, {}},
	

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},	
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page

    // Display off sequence
	{0x40, 1, {0x95}},
	{REGFLAG_DELAY, 20, {}},
    // Display off sequence

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}}, 
    {0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned int cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
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
    params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   = SYNC_PULSE_VDO_MODE; //BURST_VDO_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
    
    params->dsi.packet_size=256;
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577

    
	#if 1
	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 16;//10 16
	params->dsi.vertical_frontporch = 20;//8  20
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10; //8
	params->dsi.horizontal_backporch = 50; //50
	params->dsi.horizontal_frontporch = 50; //50
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	#endif



    
		#if 0///
        params->dsi.pll_div1=1;             // div1=0,1,2,3;div1_real=1,2,4,4
        params->dsi.pll_div2=0;             // div2=0,1,2,3;div2_real=1,2,4,4
        params->dsi.fbk_div =16;   //18
		#endif

		params->dsi.PLL_CLOCK = 148;///160 173; //dsi clock customization: should config clock value directly
		params->dsi.ssc_disable = 1;
		params->dsi.ssc_range = 4;
}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);


	///push_table(lcm_initialization_setting_ips2_lg, sizeof(lcm_initialization_setting_ips2_lg) / sizeof(struct LCM_setting_table), 1);///for LG
	push_table(lcm_initialization_setting_zgd_boe, sizeof(lcm_initialization_setting_zgd_boe) / sizeof(struct LCM_setting_table), 1);///for S1025


}

#if !defined(BUILD_LK)		
extern unsigned char lcd_bl_delay;
#endif

static void lcm_suspend(void)
{
	#if 0
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
	#else
    	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	#endif
}


static void lcm_resume(void)
{
	#if 0
    	lcm_init();
	#else
#if !defined(BUILD_LK)		
	lcd_bl_delay=1;
#endif
    	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	#endif
}
         



static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned int id_midd = 0;
	unsigned int id_low = 0;
	unsigned char buffer[5];
	unsigned char buffer1[5];
	unsigned int array[16];	

	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(150); 	

	array[0] = 0x00063902;
	array[1] = 0x0698FFFF;
	array[2] = 0x00000104;
	dsi_set_cmdq(array, 3, 1);

	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10); 

	///read_reg_v2(0x00, buffer, 1);
	///id_midd = buffer[1]; ///0x98

	read_reg_v2(0x01, buffer, 1);
	id_midd = buffer[1];///0x06
	read_reg_v2(0x02, buffer1, 1);
	id_low = buffer1[1];///0x04
	id = (id_midd << 8 | id_low);	//we only need ID

#if defined(BUILD_LK)		
	printf("%s LK ILI9806e id = 0x%08x \n", __func__, id);
#else		
	printk("%s ILI9806e id = 0x%08x \n", __func__, id);
#endif


	return (LCM_ID_ILI9806E == id)?1:0;
	///return 1;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_dsi_vdo_6572_drv = 
{
    .name			= "zechin_mt6572m_ili9806e_dsi_vdo_6572",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
};

