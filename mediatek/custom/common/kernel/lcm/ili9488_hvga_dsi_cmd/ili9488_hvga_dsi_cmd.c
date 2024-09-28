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

#define FRAME_WIDTH  (320)
#define FRAME_HEIGHT (480)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      						0xAA   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


#define LCM_ID_ILI9488       0x9488

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

	{0xF7,	4,	{ 0xa9,0x51,0x2c, 0x8a}},
	{0xc0,	2,	{0x10,0x10}},
	{0xC1,	1,	{0x41}},
	{0xC2,	1,	{0x33}},
	{0xC5,	3,	{0x00,0x2e,0x80}},//0x23 0x27

	//lingjinming modify 2014 01 26 for jiaodu!!!!
	//{0x36,	1,	{0xC8}},//ZHENGBOWEN FORM
	{0x36,	1,	{0x08}},//ZHENGBOWEN FORM
	//lingjinming


	{0x3A,   1,      {0x66}},
	{0xB1,   2,       {0xa0,0x11}},
	{0x35,1, {0x00}},
	{0x44,2 ,{0x01,0x22}},
	{0xB4,   1,      {0x02}},
	{0xB6,	3,	{0x02, 0x42,0x3B}},  //{0xB6,	3,	{0x00, 0x42,0x3B}},

	{0xBe,	 2, 	  {0x00,0x04}},
	{0xE0,	15,	{0x00,0x0d,0x14, 0x06,0x14,0x08, 0x3c,0x37,0x45, 0x08,0x0d,0x09,0x17, 0x19,0x0F}},
	//{REGFLAG_DELAY, 10, {}},
	{0xE1,	15,	{0x00,0x1a,0x1f, 0x03,0x0f,0x05, 0x33,0x23,0x48, 0x02,0x0a,0x08,0x31, 0x39,0x0f}},
	//{REGFLAG_DELAY, 10, {}},

	{0xB6,	3,	{0x02, 0x22,0x3B}},  //{0xB6,	3,	{0x02, 0x42,0x3B}},
	{0x35,	1,	0x00},
	//{0x44,	 2, 	  {0x00,0x1f}},  //{0x44,	 2, 	  {0x00,0x01}},//0x95时又变成两条线了 ///{0x44,	 2, 	  {0x00,0x90}},/{0x44,	 2, 	  {0x00,0x01}},
	{0x44,	 2, 	  {0x00,0x00}},

	{0x11, 0 ,{0x00}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 0 ,{0x00}},
	{REGFLAG_DELAY, 100, {}},

	{0x2C, 0 ,{0x00}},
	//{REGFLAG_DELAY, 500, {}},////

	{REGFLAG_END_OF_TABLE, 0x00, {}}

};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
       {REGFLAG_DELAY, 150, {}},

    // Display ON
	{0x29, 1, {0x00}},
	//{REGFLAG_DELAY, 1000, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
       {REGFLAG_DELAY, 200, {}},
     //Sleep Mode On
	{0x10, 1, {0x00}},
       {REGFLAG_DELAY, 200, {}},



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

#if 1
		memset(params, 0, sizeof(LCM_PARAMS));

		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

		///params->dsi.lcm_ext_te_enable = 1;

		params->dsi.mode   = CMD_MODE;

		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_ONE_LANE;///LCM_TWO_LANE

		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB666;///LCM_DSI_FORMAT_RGB888

		params->dsi.packet_size=256;
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_18BIT_RGB666;///LCM_PACKED_PS_24BIT_RGB888

		params->dsi.word_count=320*3;	//DSI CMD mode need set these two bellow params, different to 6577
		params->dsi.vertical_active_line=480;

		// Bit rate calculation
#ifdef CONFIG_MT6589_FPGA
		params->dsi.pll_div1=2;		// div1=0,1,2,3;div1_real=1,2,4,4
		params->dsi.pll_div2=2;		// div2=0,1,2,3;div2_real=1,2,4,4
		params->dsi.fbk_div =8;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else
		///////////////////////////////
		//params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
		//params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
		//params->dsi.fbk_div =22;///22,17		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#endif
		params->dsi.PLL_CLOCK = 160;///173; //dsi clock customization: should config clock value directly
		params->dsi.ssc_disable = 1;
		params->dsi.ssc_range = 4;


#else

		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

		params->dsi.mode   = CMD_MODE;

		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_ONE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB666;

		// Highly depends on LCD driver capability.
		params->dsi.packet_size=256;

		// Video mode setting
		params->dsi.PS=LCM_PACKED_PS_18BIT_RGB666;

		params->dsi.word_count=320*3;
		params->dsi.vertical_sync_active=2;
		params->dsi.vertical_backporch=2;
		params->dsi.vertical_frontporch=2;
		params->dsi.vertical_active_line=480;

		params->dsi.line_byte=2180;		// 2256 = 752*3
		params->dsi.horizontal_sync_active_byte=5;
		params->dsi.horizontal_backporch_byte=15;
		params->dsi.horizontal_frontporch_byte=15;
		params->dsi.rgb_byte=(320*3+6);

		params->dsi.horizontal_sync_active_word_count=6;
		params->dsi.horizontal_backporch_word_count=33;
		params->dsi.horizontal_frontporch_word_count=33;

		// Bit rate calculation
		//params->dsi.pll_div1=30;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		//params->dsi.pll_div2=1;			// div2=0~15: fout=fvo/(2*div2)  zhengbowenmodify form 1
		// Bit rate calculation
		params->dsi.pll_div1=0;		//
		params->dsi.pll_div2=0;			//
		params->dsi.fbk_div =5; // 19;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#endif
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(20);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];
#if 1
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
#else
	char null = 0;
	dsi_set_cmdq_V2(0x28, 1, &null, 1);
	MDELAY(10);
	dsi_set_cmdq_V2(0x10, 1, &null, 1);
	MDELAY(120);
	//SET_RESET_PIN(0);
	//dsi_set_cmdq_V2(0x78, 1, &null, 1);
	//MDELAY(1);
	//SET_RESET_PIN(1);
#endif
}

static void lcm_resume(void)
{
	unsigned int data_array[2];

#if 1
	lcm_init();
#else
	char null = 0;

	//MDELAY(20);

	int i=0;
	for(i=0;i<2;i++)
	{
	dsi_set_cmdq_V2(0x11, 1, &null, 1);
	}

	//data_array=0x00110500;
	//data_array[0]=0x00013902;
	// data_array[1]=0x11;
	// dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

	dsi_set_cmdq_V2(0x29, 1, &null, 1);

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
    dsi_set_cmdq(&data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(&data_array, 3, 1);

    //data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    //dsi_set_cmdq(&data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(&data_array, 1, 0);
}


static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


#if defined(BUILD_LK)
    printf("%s, %d\n", __func__, level);
#elif defined(BUILD_UBOOT)
    printf("%s, %d\n", __func__, level);
#else
    printk("lcm_setbacklight = %d\n", level);
#endif

    if(level > 255)
        level = 255;

    data_array[0]= 0x00023902;
    data_array[1] =(0x51|(level<<8));
    dsi_set_cmdq(&data_array, 2, 1);
}


static void lcm_setpwm(unsigned int divider)
{
    // TBD
}


static unsigned int lcm_getpwm(unsigned int divider)
{
    // ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
    // pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
    unsigned int pwm_clk = 23706 / (1<<divider);


    return pwm_clk;
}

///extern void DSI_clk_HS_mode(kal_bool enter);

static unsigned int lcm_compare_id(void)
{
	unsigned char buffer[4];
	unsigned int data_array[16];

	char id_high=0;
	char id_low=0;
	int id=0;

	return 1;///


	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(150);


	#ifdef BUILD_LK
        DSI_clk_HS_mode(1);
        #endif

        MDELAY(10);
        #ifdef BUILD_LK
        DSI_clk_HS_mode(0);
        #endif

	//*************Enable CMD2 Page1  *******************//
	//data_array[0]=0x00063902;
	//data_array[1]=0x52AA55F0;
	//data_array[2]=0x00000108;
	//dsi_set_cmdq(&data_array, 3, 1);
	//MDELAY(10);

	data_array[0] = 0x00043700;// read id return two byte,version and id
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xD3, buffer, 4);
	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;

#if defined(BUILD_LK)
	printf("%s LK ILI9488 id = 0x%08x \n", __func__, id);
#else
	printk("%s ILI9488 id = 0x%08x \n", __func__, id);
#endif


	return (LCM_ID_ILI9488 == id)?1:0;
	///return 1;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9488_dsi_hvga_lcm_drv =
{
        .name			= "zechin_mt6572m_ili9488_dsi_hvga",
	.set_util_funcs       = lcm_set_util_funcs,
	.get_params          = lcm_get_params,
	.init                       = lcm_init,
	.suspend               = lcm_suspend,
	.resume                = lcm_resume,
	.update         = lcm_update,
	//.set_backlight	= lcm_setbacklight,
	//.set_pwm        = lcm_setpwm,
	//.get_pwm        = lcm_getpwm,
	.compare_id    = lcm_compare_id,
};

