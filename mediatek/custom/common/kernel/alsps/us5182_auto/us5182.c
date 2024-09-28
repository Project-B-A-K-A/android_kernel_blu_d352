/*
 * Copyright (C) 2012 UPI semi <Finley_huang@upi-semi.com>. All Rights Reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/hwmsen_helper.h>
#include "us5182.h"

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <linux/wakelock.h> 

#define POWER_NONE_MACRO MT65XX_POWER_NONE
/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_US5182 5182

#define US5182_I2C_ADDR_RAR 0   /*!< the index in obj->hw->i2c_addr: alert response address */
#define US5182_I2C_ADDR_ALS 1   /*!< the index in obj->hw->i2c_addr: ALS address */
#define US5182_I2C_ADDR_PS  2   /*!< the index in obj->hw->i2c_addr: PS address */
#define US5182_DEV_NAME     "US5182"
/******************************************************************************
 * MACRO
*******************************************************************************/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO fmt, ##args)  
/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

/*----------------------------------------------------------------------------*/
static struct i2c_client *us5182_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id us5182_i2c_id[] = {{US5182_DEV_NAME,0},{}};
static const struct i2c_board_info __initdata i2c_US5182= {I2C_BOARD_INFO("US5182",0x39)};
/*the adapter id & i2c address will be available in customization*/
//static unsigned short us5182_force[] = {0x00, US5182_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const us5182_forces[] = { us5182_force, NULL };
//static struct i2c_client_address_data us5182_addr_data = { .forces = us5182_forces,};
/******************************************************************************
 * function declaration
*******************************************************************************/
static int us5182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int us5182_i2c_remove(struct i2c_client *client);
	#if !defined(MTK_AUTO_DETECT_ALSPS)
static int us5182_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
	#else
static int us5182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
	#endif
static int us5182_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int us5182_i2c_resume(struct i2c_client *client);
static struct us5182_priv *g_us5182_ptr = NULL;

struct wake_lock ps_lock_us5182;

/******************************************************************************
 * structure
*******************************************************************************/
typedef enum {
    CMC_TRC_ALS_DATA = 0x0001,
    CMC_TRC_PS_DATA  = 0x0002,
    CMC_TRC_EINT     = 0x0004,
    CMC_TRC_IOCTL    = 0x0008,
    CMC_TRC_I2C      = 0x0010,
    CMC_TRC_CVT_ALS  = 0x0020,
    CMC_TRC_CVT_PS   = 0x0040,
    CMC_TRC_DEBUG    = 0x8000,
} CMC_TRC;

typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

struct us5182_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
	//struct timer_list   first_read_ps_timer;
	//struct timer_list   first_read_als_timer;
    
    /*misc*/
    atomic_t    trace;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u8         	_align1;
    u8          _align2;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
	u32			als;
	u32			ps;
	
    bool    		als_enable;    			/*record current als status*/
	unsigned int    als_widow_loss; 
	
    bool    		ps_enable;     			/*record current ps status*/
    unsigned int    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       	enable;         	/*record HAL enalbe status*/
    ulong       	pending_intr;   	/*pending interrupt*/
    u8				polarity;
    //ulong        first_read;   	// record first read ps and als
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
static struct i2c_driver us5182_i2c_driver = {	
	.probe      = us5182_i2c_probe,
	.remove     = us5182_i2c_remove,
//	.detect     = us5182_i2c_detect,
	.suspend    = us5182_i2c_suspend,
	.resume     = us5182_i2c_resume,
	.id_table   = us5182_i2c_id,
//	.address_data = &us5182_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = US5182_DEV_NAME,
	},
};

static struct us5182_priv *us5182_obj = NULL;

#if defined(MTK_AUTO_DETECT_ALSPS)
static int	us5182_init_flag = -1;	// 0<==>OK -1 <==> fail
static int  us5182_local_init(void);
static int  us5182_local_uninit(void);
static struct sensor_init_info us5182_init_info = {
		.name = "US5182",
		.init = us5182_local_init,
		.uninit = us5182_local_uninit,

};
#else
static struct platform_driver us5182_alsps_driver;
#endif

static int us5182_get_ps_value(struct us5182_priv *obj, u32 ps);
static int us5182_get_als_value(struct us5182_priv *obj, u32 als);
/*----------------------------------------------------------------------------*/
static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
   u8 buf;
    int ret = 0;
	
    client->addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG |I2C_RS_FLAG;
    buf = addr;
	ret = i2c_master_send(client, (const char*)&buf, 1<<8 | 1);
    //ret = i2c_master_send(client, (const char*)&buf, 1);
    if (ret < 0) {
        APS_ERR("send command error!!\n");
        return -EFAULT;
    }

    *data = buf;
	client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}
/*----------------------------------------------------------------------------*/
int us5182_reg_write(struct i2c_client *client, u8 reg,u8 mask, u8 shift, int val ) {
	//struct us5182_data *data = i2c_get_clientdata(client);
	//int err;
        u8 regdata;
    
	hwmsen_read_byte_sr(client,reg,&regdata);
	regdata &= ~mask;
	regdata |= val << shift;
	
	if(hwmsen_write_byte(client,reg,regdata))
	{
	  return -EFAULT;
	}

    //APS_LOG("%s %d i2c transfer error\n", __func__, __LINE__);
    return 0;
}
/*----------------------------------------------------------------------------*/
static void us5182_dumpReg(struct i2c_client *client)
{
	int i=0;
  	u8 addr = 0x00;
  	u8 regdata=0;
  	for(i=0; i<0x11; i++)
  	{
    	//dump all
    	hwmsen_read_byte_sr(client,addr,&regdata);
		APS_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
		//snprintf(buf,1,"%c",regdata);
		addr++;
  	}
  	addr = 0x16;
  	hwmsen_read_byte_sr(client,addr,&regdata);
  	APS_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
  	addr = 0x20;
	for(i=0; i<3; i++)
  	{
    	//dump all
    	hwmsen_read_byte_sr(client,addr,&regdata);
		APS_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
		//snprintf(buf,1,"%c",regdata);
		addr++;
  	}
  	addr = 0x29;
	for(i=0; i<3; i++)
  	{
    	//dump all
    	hwmsen_read_byte_sr(client,addr,&regdata);
		APS_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
		//snprintf(buf,1,"%c",regdata);
		addr++;
  	}
  	return;
}
/*----------------------------------------------------------------------------*/
static int us5182_enable_eint(struct i2c_client *client)
{
	//struct us5182_priv *obj = i2c_get_clientdata(client);        

	//g_us5182_ptr = obj;
	
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP); 

	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
    return 0;
}

static int us5182_disable_eint(struct i2c_client *client)
{
	//struct us5182_priv *obj = i2c_get_clientdata(client);        

	//g_us5182_ptr = obj;
	
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, FALSE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_DOWN); 

	mt65xx_eint_mask(CUST_EINT_ALS_NUM);
    return 0;
}
/*----------------------------------------------------------------------------*/
int us5182_get_timing(void)
{
return 200;
/*
	u32 base = I2C2_BASE; 
	return (__raw_readw(mt6516_I2C_HS) << 16) | (__raw_readw(mt6516_I2C_TIMING));
*/
}
/*----------------------------------------------------------------------------*/
/* resolution for als*/
static u8 us5182_get_als_resolution(struct i2c_client *client)
{		
	u8 res;
	
	hwmsen_read_byte_sr(client, REGS_CR01, &res); 
	res = (res & 0x18) >> 3;
    
	return res; 
}

static int us5182_set_als_resolution(struct i2c_client *client, int res)
{
        return us5182_reg_write(client,REGS_CR01,
                CR1_ALS_RES_MASK, CR1_ALS_RES_SHIFT, res);
}

int us5182_read_als_data(struct i2c_client *client, u32 *data)
{
	struct us5182_priv *obj = i2c_get_clientdata(client);    
	u32 ret = 0;
	u8 lsb, msb, bitdepth;

	if(hwmsen_read_byte_sr(client,REGS_LSB_SENSOR, &lsb))
	{
		APS_ERR("reads als lsb = %d\n", ret);
		return -EFAULT;
	}
	
	if(hwmsen_read_byte_sr(client,REGS_MSB_SENSOR, &msb))
	{
		APS_ERR("reads als msb = %d\n", ret);
		return -EFAULT;
	}
	
	bitdepth = us5182_get_als_resolution(client);
	
	switch(bitdepth){
	case 0:	
		lsb &= 0xF0;
		break;
	case 1:
		lsb &= 0xFC;
		break;
	default:	
		break;
	}
	
	*data = (msb << 8) | lsb;
	
	if(atomic_read(&obj->trace) & CMC_TRC_ALS_DATA)
	{
		APS_DBG("ALS:  0x%04X\n", (u32)(*data));
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
/* resolution for als*/
static u8 us5182_get_ps_resolution(struct i2c_client *client)
{		
	u8 res;
	
	hwmsen_read_byte_sr(client, REGS_CR02, &res); 
	res = (res & 0x18) >> 3;
    
	return res; 
}

static int us5182_set_ps_resolution(struct i2c_client *client, int res)
{
        return us5182_reg_write(client,REGS_CR02,
                CR2_PS_RES_MASK, CR2_PS_RES_SHIFT, res);
}

int us5182_read_ps_data(struct i2c_client *client, u32 *data)
{
	struct us5182_priv *obj = i2c_get_clientdata(client);    
	u32 ret = 0;
	u8 lsb, msb, bitdepth;

	if(hwmsen_read_byte_sr(client,REGS_LSB_SENSOR_PS,&lsb))
	{
		///APS_ERR("reads ps lsb = %d\n", ret);   ///marked by maxyu 140425 for speed+;
		return -EFAULT;
	}
	
	if(hwmsen_read_byte_sr(client,REGS_MSB_SENSOR_PS,&msb))
	{
		///APS_ERR("reads ps msb = %d\n", ret);  ///marked by maxyu 140425 for speed+;
		return -EFAULT;
	}
	
	bitdepth = us5182_get_ps_resolution(client);
	
	switch(bitdepth){
	case 0:	
		lsb &= 0xF0;
		break;
	case 1:
		lsb &= 0xFC;
		break;
	default:	
		break;
	}
	
	*data = (msb << 8) | lsb;
	
	if(atomic_read(&obj->trace) & CMC_TRC_PS_DATA)
	{
		///APS_DBG("PS:  0x%04X\n", *data);  ///marked by maxyu 140425 for speed+;
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
int us5182_init_device(struct i2c_client *client)
{
	struct us5182_priv *obj = i2c_get_clientdata(client);        
	u8 MSB = 0, LSB = 0;

	APS_LOG("us5182_init_device.........\r\n");
	// ADC clock/2; 50/60Hz rejection disable
	if(hwmsen_write_byte(client,REGS_CR10,0x08)) ///0x01 ///0x00,for 50 60 hz
	{
	  return -EFAULT;
	}
	if(hwmsen_write_byte(client,0x17,0x10)) ///0x15 ///add
	{
	  return -EFAULT;
	}
	
	if(hwmsen_write_byte(client,REGS_CR2B,0x80))
	{
	  return -EFAULT;
	}
    if(hwmsen_write_byte(client,REGS_CR29,0x14))
    {
      return -EFAULT;
    }
    if(hwmsen_write_byte(client,REGS_CR2A,0x00))
    {
      return -EFAULT;
    }
    // operation mode
    if(hwmsen_write_byte(client,REGS_CR00,0x00))
    {
      return -EFAULT;
    }
    if(hwmsen_write_byte(client,REGS_CR21,0x01)) 
    {
	return -EFAULT;
    }
    // ALS_FQ once, ALS 16bit; als gain*128
    if(hwmsen_write_byte(client,REGS_CR01,0x17))
    {
      return -EFAULT;
    }
    // ALS_OFG=0
	if(hwmsen_write_byte(client,REGS_CR16,0x14)) 
	{
	  return -EFAULT;
	}
	// PS_FQ once; INT level; PS 16bit; PS gain*128
	if(hwmsen_write_byte(client,REGS_CR02,0x10)) //0x17,ps gain
	{
	  return -EFAULT;
	}
	// no wait time; LED_DR 100ma; PS approach
	if(hwmsen_write_byte(client,REGS_CR03,0x3C)) ///0x2C,for led
	{
	  return -EFAULT;
	}
	if(hwmsen_write_byte(client,REGS_CR17,0x10)) 
	{
	  return -EFAULT;
	}
	if(hwmsen_write_byte(client,REGS_CR20,0x00)) 
	{
	  return -EFAULT;
	}
	if(hwmsen_write_byte(client,REGS_CR22,0xE0)) 
	{
	  return -EFAULT;
	}
	if(obj->hw->polling_mode_ps == 1)
	{	
		//PS high and low threshold level setting
		if(hwmsen_write_byte(client,REGS_INT_LSB_TH_LO_PS,0x00)) 
		{
		  return -EFAULT;
		}
		if(hwmsen_write_byte(client,REGS_INT_MSB_TH_LO_PS,0x00)) 
		{
		  return -EFAULT;
		}
		if(hwmsen_write_byte(client,REGS_INT_LSB_TH_HI_PS,0xff)) 
		{
		  return -EFAULT;
		}
		if(hwmsen_write_byte(client,REGS_INT_MSB_TH_HI_PS,0xff)) 
		{
		  return -EFAULT;
		}
	}
	else	//interrupt mode
	{	
		LSB = (u8)(obj->hw->ps_threshold_low & 0x00FF);
		APS_DBG("PS low threshold lsb:  0x%X\n", LSB);
		//PS high and low threshold level setting
		if(hwmsen_write_byte(client,REGS_INT_LSB_TH_LO_PS,LSB)) 
		{
		  return -EFAULT;
		}
		MSB = (u8)(obj->hw->ps_threshold_low >> 8);
		APS_DBG("PS low threshold msb:  0x%X\n", MSB);
		if(hwmsen_write_byte(client,REGS_INT_MSB_TH_LO_PS,MSB)) 
		{
		  return -EFAULT;
		}
		LSB = (u8)(obj->hw->ps_threshold_high & 0x00FF);
		APS_DBG("PS high threshold lsb:  0x%X\n", LSB);
		if(hwmsen_write_byte(client,REGS_INT_LSB_TH_HI_PS,LSB)) 
		{
		  return -EFAULT;
		}
		MSB = (u8)(obj->hw->ps_threshold_high >> 8);
		APS_DBG("PS high threshold msb:  0x%X\n", MSB);
		if(hwmsen_write_byte(client,REGS_INT_MSB_TH_HI_PS,MSB)) 
		{
		  return -EFAULT;
		}
	}			
	return 0;
}
/*----------------------------------------------------------------------------*/
static void us5182_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	hwPowerOn(MT65XX_POWER_LDO_VGP1, VOL_2800, "TP");///added by maxyu 140625 for i2c init fail;
	//mdelay(100);

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "US5182")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "US5182")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;


}
/*----------------------------------------------------------------------------*/
static int us5182_enable_als(struct i2c_client *client, bool enable)
{
	struct us5182_priv *obj = i2c_get_clientdata(client);
	int trc = atomic_read(&obj->trace);
	u8 regdata=0, temp = 0, oridata = 0;

	APS_LOG(" us5182_enable_als %d, als:%d, ps:%d \n", enable, obj->als_enable, obj->ps_enable); 
	
	if(hwmsen_read_byte_sr(client, REGS_CR00, &regdata))
	{
		APS_ERR("read APS_CONFIGUATION register err!\n");
		return -1;
	}
	oridata = regdata;
	regdata &= 0b01001111; //first set 00 to clear bit
	
	//if(enable == TRUE)//enable als
	{
	     APS_LOG("first enable als!\n");
		 if(true == obj->ps_enable)
		 {
		   APS_LOG("ALS(1): enable both \n");
		 }
		 else	//if(false == obj->ps_enable)
		 {
		   APS_LOG("ALS(1): enable als only \n");
		   regdata |= 0b00010000; //only enable als
		 }
		 atomic_set(&obj->als_deb_on, 1);
		 atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));

	}
	/*else
	{
		if(true == obj->ps_enable)
		 {
		   APS_LOG("ALS(0):enable ps only \n");
		   regdata |= 0b00100000;//only enable ps

		 }
		 else	//if(false == obj->ps_enable)
		 {
		   APS_LOG("ALS(0): disable both \n");
		   regdata |= 0b10000000;//bit7:1=>shut down, disable both
		 }
		 //del_timer_sync(&obj->first_read_als_timer);
		 atomic_set(&obj->als_deb_on, 0);
	}*/
	
	/*temp = oridata | 0b10000000;
	APS_LOG("1.shut down! 0x%x\n", temp);
	if(hwmsen_write_byte(client,REGS_CR00,temp))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}
	
	temp = regdata | 0b10000000;
	if(hwmsen_write_byte(client,REGS_CR00,temp))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}*/
    	
	if(hwmsen_write_byte(client,REGS_CR00,regdata))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}

	if(0 == obj->ps_enable)
	{
		//if(hwmsen_write_byte(client,REGS_CR21,0x01))
		{
			//APS_LOG("us5182_enable_als failed!\n");
			//return -1;
		}
	}

	/*mdelay(60);
	temp = regdata | 0b10000000;
	if(hwmsen_write_byte(client,REGS_CR00,temp))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}
	mdelay(60);
	if(hwmsen_write_byte(client,REGS_CR00,regdata))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}	


	mdelay(60);
	temp = regdata | 0b10000000;
	if(hwmsen_write_byte(client,REGS_CR00,temp))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}
	mdelay(60);
	if(hwmsen_write_byte(client,REGS_CR00,regdata))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}*/	
	obj->als_enable = 1;///enable;
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int us5182_enable_ps(struct i2c_client *client, bool enable)
{
     
	struct us5182_priv *obj = i2c_get_clientdata(client);
	int trc = atomic_read(&obj->trace);
	int res;
	u8 regdata=0, temp = 0, oridata = 0;
		
	APS_LOG(" us5182_enable_ps %d, als:%d, ps:%d \n", enable, obj->als_enable, obj->ps_enable);

	if(hwmsen_read_byte_sr(client, REGS_CR00, &regdata))
	{
		APS_ERR("read APS_CONFIGUATION register err!\n");
		return -1;
	}
	oridata = regdata;
	regdata &= 0b01001111; //first set 00 to clear bit
	
	if(enable == TRUE)//enable ps
	{
		if(obj->hw->polling_mode_ps == 0)
		{
			res = us5182_enable_eint(client);
			if(res!=0)
			{
				APS_ERR("enable eint fail: %d\n", res);
				return res;
			}
		}		
	     APS_LOG("first enable ps!\n");
		 
		if(1 == obj->hw->polling_mode_ps)
			wake_lock(&ps_lock_us5182);
		 
		if(true == obj->als_enable)
		{
		  	//initial regdata value is ok
		  	APS_LOG("PS(1): enable ps both !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		}
		else 	//if(false == obj->als_enable)
		{
			///regdata |= 0b00100000; //only enable ps
		  	APS_LOG("PS(1): enable ps only !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
	}
	else//disable ps
	{
		if(obj->hw->polling_mode_ps == 0)
		{	
			res = us5182_disable_eint(client);
			if(res!=0)
			{
				APS_ERR("disable eint fail: %d\n", res);
				return res;
			}
		}	
		

		if(1 == obj->hw->polling_mode_ps)		
			wake_unlock(&ps_lock_us5182);
	
		if(true == obj->als_enable)
		{
			APS_LOG("PS(0): disable ps only enalbe als !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		   	regdata |= 0b00010000;//only enable als
		}
		else	//if(false == obj->als_enable)
		{
		   	APS_LOG("PS(0): disable both !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		   	regdata |= 0b00010000;//only enable als //regdata |= 0b10000000;//disable both
		}
		atomic_set(&obj->ps_deb_on, 0);
	}
	
	/*temp = oridata | 0b10000000;
	APS_LOG("1.shut down! 0x%x\n", temp);
	if(hwmsen_write_byte(client,REGS_CR00,temp))
	{
		APS_LOG("us5182_enable_ps failed!\n");
		return -1;
	}
    	
	temp = regdata | 0b10000000;
	if(hwmsen_write_byte(client,REGS_CR00,temp))
	{
		APS_LOG("us5182_enable_ps failed!\n");
		return -1;
	}*/
//	if(hwmsen_write_byte(client,REGS_CR21,0x01))
	{
//		APS_LOG("us5182_enable_ps failed!\n");
//		return -1;
	}
	if(hwmsen_write_byte(client,REGS_CR00,regdata))
	{
		APS_LOG("us5182_enable_ps failed!\n");
		return -1;
	}
	if(0 == enable)
	{
		if(hwmsen_write_byte(client,REGS_CR21,0x01))
		{
			APS_LOG("us5182_enable_als failed!\n");
			return -1;
		}
	}
	
	/*mdelay(60);
	temp = regdata | 0b10000000;
	if(hwmsen_write_byte(client,REGS_CR00,temp))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}
	mdelay(60);
	if(hwmsen_write_byte(client,REGS_CR00,regdata))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}
	
	
	mdelay(60);
	temp = regdata | 0b10000000;
	if(hwmsen_write_byte(client,REGS_CR00,temp))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}
	mdelay(60);
	if(hwmsen_write_byte(client,REGS_CR00,regdata))
	{
		APS_LOG("us5182_enable_als failed!\n");
		return -1;
	}*/
	
	obj->ps_enable = enable;
	
	return 0;
}
/*----------------------------------interrupt functions--------------------------------*/
static int intr_flag = 0;
/*----------------------------------------------------------------------------*/
static int us5182_check_intr(struct i2c_client *client) 
{
	struct us5182_priv *obj = i2c_get_clientdata(client);
	int err;
	u8 data=0;

	err = hwmsen_read_byte_sr(client,REGS_CR00,&data);
	APS_LOG("INT flage: = %x\n", data);

	if(err)
	{
		APS_ERR("WARNING: read int status: %d\n", err);
		return 0;
	}
	
	if(data & 0x08)//bit3 = 1,approach
	{
		intr_flag = 0;//for close
	}
	else		//bit3 = 0,not approach
	{
		intr_flag = 1;//for away
	}
	APS_LOG("INT flag: = %x\n", intr_flag);
	
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG)
	{
		APS_LOG("check intr: 0x%08lX\n", obj->pending_intr);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
void us5182_eint_func(void)
{
	struct us5182_priv *obj = g_us5182_ptr;
	APS_FUN();
	if(!obj)
	{
		return;
	}
	
	schedule_delayed_work(&obj->eint_work,0);

}
/*----------------------------------------------------------------------------*/
static void us5182_eint_work(struct work_struct *work)
{
	struct us5182_priv *obj = (struct us5182_priv *)container_of((struct delayed_work *)work, struct us5182_priv, eint_work);
	hwm_sensor_data sensor_data;
	int res = 0;
	int PsData;
	
	APS_FUN();

	if((res = us5182_read_ps_data(obj->client,&obj->ps))){
		  APS_ERR("us5182 read ps data error %d\n", res);
		  goto EXIT;
	}
	sensor_data.values[0] = us5182_get_ps_value(obj, obj->ps);
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	
	if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data))){
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
		  goto EXIT;
	}
	//add
	if(obj->polarity == 0){
		mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_POLARITY_HIGH);
		obj->polarity = 1;
	} else {	
		mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_POLARITY_LOW);
		obj->polarity = 0;
	}
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
	return;

EXIT:
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
	APS_ERR("us5182_eint_work err: %d\n", res);
}
/*----------------------------------------------------------------------------*/
int us5182_setup_eint(struct i2c_client *client)
{
	struct us5182_priv *obj = i2c_get_clientdata(client);        

	g_us5182_ptr = obj;
	/*configure to GPIO function, external interrupt*/
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
	
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, us5182_eint_func, 0);
	
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int us5182_init_client(struct i2c_client *client)
{
	//struct us5182_priv *obj = i2c_get_clientdata(client);
	int err=0;
	APS_LOG("us5182_init_client.........\r\n");

	if((err = us5182_setup_eint(client)))
	{
		APS_ERR("setup eint error: %d\n", err);
		return err;
	}
	
	if((err = us5182_disable_eint(client)))
	{
		APS_ERR("disable eint error: %d\n", err);
		return err;
	}
	
	if((err = us5182_init_device(client)))
	{
		APS_ERR("init dev error: %d\n", err);
		return err;
	}
	
	return err;
}
/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t us5182_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&us5182_obj->i2c_retry), atomic_read(&us5182_obj->als_debounce), 
		atomic_read(&us5182_obj->ps_mask), us5182_obj->ps_thd_val, atomic_read(&us5182_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&us5182_obj->i2c_retry, retry);
		atomic_set(&us5182_obj->als_debounce, als_deb);
		atomic_set(&us5182_obj->ps_mask, mask);
		us5182_obj->ps_thd_val= thres;        
		atomic_set(&us5182_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&us5182_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&us5182_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	u32 dat = 0;
	
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	if((res = us5182_read_als_data(us5182_obj->client, &us5182_obj->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{   dat = us5182_obj->als;
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	u32 dat=0;
	
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	
	if((res = us5182_read_ps_data(us5182_obj->client, &us5182_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
	    dat = us5182_obj->ps;
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_show_reg(struct device_driver *ddri, char *buf)
{
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	
	/*read*/
	us5182_dumpReg(us5182_obj->client);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	
	if(us5182_obj->hw)
	{
	
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			us5182_obj->hw->i2c_num, us5182_obj->hw->power_id, us5182_obj->hw->power_vol);
		
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	#ifdef MT6516
	len += snprintf(buf+len, PAGE_SIZE-len, "EINT: %d (%d %d %d %d)\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN),
				CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_DEBOUNCE_CN);

	len += snprintf(buf+len, PAGE_SIZE-len, "GPIO: %d (%d %d %d %d)\n",	GPIO_ALS_EINT_PIN, 
				mt_get_gpio_dir(GPIO_ALS_EINT_PIN), mt_get_gpio_mode(GPIO_ALS_EINT_PIN), 
				mt_get_gpio_pull_enable(GPIO_ALS_EINT_PIN), mt_get_gpio_pull_select(GPIO_ALS_EINT_PIN));
	#endif

	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&us5182_obj->als_suspend), atomic_read(&us5182_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
#ifndef MTK_AUTO_DETECT_ALSPS
static ssize_t us5182_show_i2c(struct device_driver *ddri, char *buf)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_store_i2c(struct device_driver *ddri, const char *buf, size_t count)
{
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct us5182_priv *obj, const char* buf, size_t count,
                             u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t us5182_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < us5182_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", us5182_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}

static ssize_t us5182_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	//struct us5182_priv *obj;
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(us5182_obj->als_level, us5182_obj->hw->als_level, sizeof(us5182_obj->als_level));
	}
	else if(us5182_obj->als_level_num != read_int_from_buf(us5182_obj, buf, count, 
			us5182_obj->hw->als_level, us5182_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
} 
/*----------------------------------------------------------------------------*/
static ssize_t us5182_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < us5182_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", us5182_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}

static ssize_t us5182_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!us5182_obj)
	{
		APS_ERR("us5182_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(us5182_obj->als_value, us5182_obj->hw->als_value, sizeof(us5182_obj->als_value));
	}
	else if(us5182_obj->als_value_num != read_int_from_buf(us5182_obj, buf, count, 
			us5182_obj->hw->als_value, us5182_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, us5182_show_als,   NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, us5182_show_ps,    NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, us5182_show_config,us5182_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, us5182_show_alslv, us5182_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, us5182_show_alsval,us5182_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, us5182_show_trace, us5182_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, us5182_show_status,  NULL);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, us5182_show_reg,   NULL);
#ifndef MTK_AUTO_DETECT_ALSPS
static DRIVER_ATTR(i2c,     S_IWUSR | S_IRUGO, us5182_show_i2c,   us5182_store_i2c);
#endif
/*----------------------------------------------------------------------------*/
static struct driver_attribute *us5182_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
#ifndef MTK_AUTO_DETECT_ALSPS
    &driver_attr_i2c,
#endif
    &driver_attr_reg,
}; 
/*----------------------------------------------------------------------------*/
static int us5182_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(us5182_attr_list)/sizeof(us5182_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, us5182_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", us5182_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int us5182_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(us5182_attr_list)/sizeof(us5182_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, us5182_attr_list[idx]);
	}
	
	return err;
} 
/****************************************************************************** 
 * Function 
******************************************************************************/
static int us5182_get_als_value(struct us5182_priv *obj, u32 als)
{
	int idx;
	int invalid = 0;
	//als = als & 0x3f;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		}
		
		return obj->hw->als_value[idx];
	}
	else
	{
		if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		}
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
static int us5182_get_ps_value(struct us5182_priv *obj, u32 ps)
{
    int val= -1;
	int invalid = 0;

	if(ps > obj->hw->ps_threshold_high)///0x500
	{
		val = 0;  /*close*/
	}
	else //if(ps < obj->hw->ps_threshold_low)
	{
		val = 1;  /*far away*/
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
		   ///APS_DBG("PS:  %05d => %05d\n", ps, val); ///marked by maxyu 140425 for speed+;
		}
		return val;
		
	}	
	else
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			///APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);  ///marked by maxyu 140425 for speed+;
		}
		return -1;
	}	

	APS_LOG("9.us5182_get_ps_value : %x", val);
	return val;
}
/*----------------------------------------------------------------------------*/
static int us5182_open(struct inode *inode, struct file *file)
{
	file->private_data = us5182_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int us5182_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
/*20130617 this funetion wait finley edit*/
static long us5182_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct us5182_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	u32 dat;
	uint32_t enable;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = us5182_enable_ps(obj->client, true)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = us5182_enable_ps(obj->client, false)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = us5182_read_ps_data(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			dat = us5182_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = us5182_read_ps_data(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;            

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = us5182_enable_als(obj->client, true)))
				{
					APS_ERR("enable als fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = us5182_enable_als(obj->client, false)))
				{
					APS_ERR("disable als fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = us5182_read_als_data(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = us5182_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = us5182_read_als_data(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations us5182_fops = {
//	.owner = THIS_MODULE,
	.open = us5182_open,
	.release = us5182_release,
	.unlocked_ioctl = us5182_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice us5182_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &us5182_fops,
}; 
/*----------------------------------------------------------------------------*/
static int us5182_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	APS_FUN();
	return 0;
#if 0	
	struct us5182_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if((err = us5182_enable_als(client, false)))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if((err = us5182_enable_ps(client, false)))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		us5182_power(obj->hw, 0);
	}
	return 0;
#endif
}
/*----------------------------------------------------------------------------*/
static int us5182_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;

#if 0
	struct us5182_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	us5182_power(obj->hw, 1);
	if((err = us5182_init_client(client)))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = us5182_enable_als(client, true)))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if((err = us5182_enable_ps(client, true)))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}

	return 0;
#endif
} 
/*----------------------------------------------------------------------------*/
static void us5182_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct us5182_priv *obj = container_of(h, struct us5182_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1);    
	if((err = us5182_enable_als(obj->client, false)))
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
}
/*----------------------------------------------------------------------------*/
static void us5182_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct us5182_priv *obj = container_of(h, struct us5182_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = us5182_enable_als(obj->client, true)))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
}
/*----------------------------------------------------------------------------*/
int us5182_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct us5182_priv *obj = (struct us5182_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if((err = us5182_enable_ps(obj->client, true)))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((err = us5182_enable_ps(obj->client, false)))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;
		case SENSOR_GET_DATA:
			//APS_LOG("fwq get ps data !!!!!!\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				
				if((err = us5182_read_ps_data(obj->client, &obj->ps)))
				{
					err = -1;;
				}
				else
				{
				    while(-1 == us5182_get_ps_value(obj, obj->ps))
				    {
				      us5182_read_ps_data(obj->client, &obj->ps);
				      msleep(50);
				    }
				   
					sensor_data->values[0] = us5182_get_ps_value(obj, obj->ps);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					//printk("US5182 get ps RAW data =0X%0x,value=%d\n",obj->ps,sensor_data->values[0]);
					//APS_LOG("fwq get ps data =%d\n",sensor_data->values[0]);
				}				
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/ 
int us5182_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct us5182_priv *obj = (struct us5182_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if((err = us5182_enable_als(obj->client, true)))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = us5182_enable_als(obj->client, false)))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			//APS_LOG("fwq get als data !!!!!!\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
								
				if((err = us5182_read_als_data(obj->client, &obj->als)))
				{
					err = -1;;
				}
				else
				{
				    while(-1 == us5182_get_als_value(obj, obj->als))
				    {
				      us5182_read_als_data(obj->client, &obj->als);
				      msleep(50);
				    }
					sensor_data->values[0] = us5182_get_als_value(obj, obj->als);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}				
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
#if !defined(MTK_AUTO_DETECT_ALSPS)
static int us5182_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
	#else
static int us5182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
	#endif
{    
	strcpy(info->type, US5182_DEV_NAME);
	return 0;
} 
/*----------------------------------------------------------------------------*/
static int us5182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    
	struct us5182_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	APS_FUN();
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	us5182_obj = obj;

	obj->hw = us5182_get_cust_alsps_hw();


	INIT_DELAYED_WORK(&obj->eint_work, us5182_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 30);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);

	obj->ps_enable = 0;
	obj->als_enable = 0;
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	obj->polarity = 0;
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);
    //pre set ps threshold
	//atomic_set(&obj->ps_thd_val, obj->hw->ps_threshold);
	obj->ps_thd_val = obj->hw->ps_threshold;

	us5182_i2c_client = client;

	if((err = us5182_init_client(client)))
	{
		goto exit_init_failed;
	}
	
	if((err = misc_register(&us5182_device)))
	{
		APS_ERR("us5182_device register failed\n");
		goto exit_misc_device_register_failed;
	}

#if defined(MTK_AUTO_DETECT_ALSPS)
	if((err = us5182_create_attr(&(us5182_init_info.platform_diver_addr->driver))))
#else
	if((err = us5182_create_attr(&us5182_alsps_driver.driver)))
#endif
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	obj_ps.self = us5182_obj;
	obj_ps.polling = obj->hw->polling_mode_ps;
	obj_ps.sensor_operate = us5182_ps_operate;

	if(1 == obj->hw->polling_mode_ps)
	{
		wake_lock_init(&ps_lock_us5182,WAKE_LOCK_SUSPEND,"ps wakelock");
	}

	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = us5182_obj;
	obj_als.polling = obj->hw->polling_mode_als;
	obj_als.sensor_operate = us5182_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	obj->early_drv.suspend  = us5182_early_suspend;
	obj->early_drv.resume   = us5182_late_resume;  
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
#if defined(MTK_AUTO_DETECT_ALSPS)		
	us5182_init_flag =0;
#endif	
	return 0;

exit_create_attr_failed:
	misc_deregister(&us5182_device);
exit_misc_device_register_failed:
exit_init_failed:
	//i2c_detach_client(client);
	kfree(obj);
exit:
	us5182_i2c_client = NULL;           
#ifdef MT6516        
	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
#endif
	APS_ERR("%s: err = %d\n", __func__, err);
#if defined(MTK_AUTO_DETECT_ALSPS)	
	us5182_init_flag =-1;
#endif	
	return err;
}
/*----------------------------------------------------------------------------*/
static int us5182_i2c_remove(struct i2c_client *client)
{
	int err;	
	
#if defined(MTK_AUTO_DETECT_ALSPS)
	if(err = us5182_delete_attr(&(us5182_init_info.platform_diver_addr->driver)))
#else
	if((err = us5182_delete_attr(&us5182_i2c_driver.driver)))
#endif		
	{
		APS_ERR("us5182_delete_attr fail: %d\n", err);
	} 

	if((err = misc_deregister(&us5182_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	us5182_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
} 
/*----------------------------------------------------------------------------*/
#if defined(MTK_AUTO_DETECT_ALSPS)
static int  us5182_local_uninit(void)
{
	struct alsps_hw *hw = us5182_get_cust_alsps_hw();
	APS_FUN();    
	us5182_power(hw, 0);    
	us5182_i2c_client = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/

static int us5182_local_init(void) 
{
	struct alsps_hw *hw = us5182_get_cust_alsps_hw();
	///struct us5182_i2c_addr addr;

	us5182_power(hw, 1);    
	///us5182_get_addr(hw, &addr);
	#if 0///(LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	
	us5182_force[0] = hw->i2c_num;
	us5182_force[1] = hw->i2c_addr[0];
	#endif
	if(i2c_add_driver(&us5182_i2c_driver))
	{
		APS_ERR("us5182 add driver error\n");
		return -1;
	} 

    if(-1 == us5182_init_flag)
    {
        return -1;
    }
	return 0;
}
#else
static int us5182_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = us5182_get_cust_alsps_hw();
	//struct us5182_i2c_addr addr;
	
	APS_FUN();
	us5182_power(hw, 1);    
	//us5182_get_addr(hw, &addr);
	//us5182_force[0] = hw->i2c_num;
	//us5182_force[1] = addr.init;
	if(i2c_add_driver(&us5182_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int us5182_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = us5182_get_cust_alsps_hw();
	APS_FUN();    
	us5182_power(hw, 0);    
	i2c_del_driver(&us5182_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver us5182_alsps_driver = {
	.probe      = us5182_probe,
	.remove     = us5182_remove,    
	.driver     = {
		.name  = "als_ps",
	}
};
#endif
/*----------------------------------------------------------------------------*/
static int __init us5182_init(void)
{
	APS_FUN();

	struct alsps_hw *hw = us5182_get_cust_alsps_hw();
	
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_US5182, 1);


#if defined(MTK_AUTO_DETECT_ALSPS)
    hwmsen_alsps_sensor_add(&us5182_init_info);
#else

	if(platform_driver_register(&us5182_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit us5182_exit(void)
{
	APS_FUN();
#ifndef MTK_AUTO_DETECT_ALSPS
	platform_driver_unregister(&us5182_alsps_driver);
#endif	
}
/*----------------------------------------------------------------------------*/
module_init(us5182_init);
module_exit(us5182_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Finley Huang");
MODULE_DESCRIPTION("us5182 Proximity & Ambient light sensor driver");
MODULE_LICENSE("GPL");
