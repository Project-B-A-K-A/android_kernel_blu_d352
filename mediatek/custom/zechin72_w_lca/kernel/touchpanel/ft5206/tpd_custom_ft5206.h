#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE         
#define TPD_I2C_NUMBER           0
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_VELOCITY_CUSTOM_X 15
#define TPD_VELOCITY_CUSTOM_Y 20

#define TPD_POWER_SOURCE_CUSTOM         MT6323_POWER_LDO_VGP2
///#define TPD_POWER_SOURCE_CUSTOM		MT6323_POWER_LDO_VGP1

///#define TPD_AUTO_UPGRADE				// if need upgrade CTP FW when POWER ON,pls enable this MACRO

#define TPD_DELAY                (2*HZ/100)

#define TPD_RES_X                320
#define TPD_RES_Y                480

//#define TPD_RES_X                480
//#define TPD_RES_Y                800
#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};



#if 1//for S1015 3.5 HVGA
#define TPD_HAVE_BUTTON
#define TPD_KEY_COUNT           3

#define TPD_KEYS                { KEY_MENU, KEY_HOMEPAGE, KEY_BACK}

#define TPD_KEYS_DIM            {{60,540,40,40},{180,540,40,40},{300,540,40,40}}
#endif


#endif /* TOUCHPANEL_H__ */
