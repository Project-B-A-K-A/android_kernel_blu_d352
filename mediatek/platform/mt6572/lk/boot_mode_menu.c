/*This file implements MTK boot mode.*/

#include <sys/types.h>
#include <debug.h>
#include <err.h>
#include <reg.h>
#include <video.h>

#include <platform/mt_typedefs.h>
#include <platform/boot_mode.h>
#include <platform/mt_reg_base.h>
#include <platform/mtk_key.h>
#include <platform/mt_gpt.h>
#include <platform/mtk_wdt.h>

#include <target/cust_key.h>

#define MODULE_NAME "[BOOT_MENU]"
extern char g_CMDLINE [];
bool g_boot_menu = false;

#if defined(MEM_PRESERVED_MODE_ENABLE)
extern void platform_mem_preserved_config(unsigned int enable);
#endif //#if defined(MEM_PRESERVED_MODE_ENABLE)
extern void mtk_wdt_disable(void);
extern BOOL recovery_check_command_trigger(void);
int unshield_recovery_detection(void);

void boot_mode_menu_select()
{
          int select = 0;  // 0=recovery mode, 1=fastboot.  2=normal boot 3=normal boot + ftrace.5=slub debug off
          const char* title_msg = "Select Boot Mode:\n[VOLUME_UP to select.  VOLUME_DOWN is OK.]\n\n";
          video_clean_screen();
          video_set_cursor(video_get_rows()/2, 0);

	#if 1 ///add by maxyu 140404,不选择直接进入RECOVERY MODE;
          video_printf("[Recovery    Mode]         <<==\n");

		select =0;
	#else
          video_printf(title_msg);
          video_printf("[Recovery    Mode]         <<==\n");
#ifdef MTK_FASTBOOT_SUPPORT
          video_printf("[Fastboot    Mode]             \n");
#endif
          video_printf("[Normal      Boot]             \n");
#ifndef USER_BUILD
          video_printf("[Normal      Boot +ftrace]     \n");
#if defined(MEM_PRESERVED_MODE_ENABLE)
          video_printf("[Normal      + mem-pre]       \n");
#endif //#if defined(MEM_PRESERVED_MODE_ENABLE)
          video_printf("[Normal      slub debug off]     \n");
#endif
          while(1)
          {
             if(mtk_detect_key(MT65XX_MENU_SELECT_KEY))//VOL_UP
             {
                    g_boot_menu = true;
                    switch(select) {

                        case 0:
#ifdef MTK_FASTBOOT_SUPPORT
                            select = 1;

                            video_set_cursor(video_get_rows()/2, 0);
                            video_printf(title_msg);
                            video_printf("[Recovery    Mode]             \n");
                            video_printf("[Fastboot    Mode]         <<==\n");
                            video_printf("[Normal      Boot]             \n");
#ifndef USER_BUILD
                            video_printf("[Normal      Boot +ftrace]     \n");
#if defined(MEM_PRESERVED_MODE_ENABLE)
                            video_printf("[Normal      + mem-pre]       \n");
#endif //#if defined(MEM_PRESERVED_MODE_ENABLE)
                            video_printf("[Normal      slub debug off]     \n");
#endif
                        break;
#endif
                        case 1:
                            select = 2;

                            video_set_cursor(video_get_rows()/2, 0);
                            video_printf(title_msg);
                            video_printf("[Recovery    Mode]             \n");
#ifdef MTK_FASTBOOT_SUPPORT
                            video_printf("[Fastboot    Mode]             \n");
#endif
                            video_printf("[Normal      Boot]         <<==\n");
#ifndef USER_BUILD
                            video_printf("[Normal      Boot +ftrace]     \n");
#if defined(MEM_PRESERVED_MODE_ENABLE)
                            video_printf("[Normal      + mem-pre]       \n");
#endif //#if defined(MEM_PRESERVED_MODE_ENABLE)
                            video_printf("[Normal      slub debug off]     \n");
#endif
                        break;

                        case 2:
#ifdef USER_BUILD
                            select = 0;
#else
                            select = 3;
#endif

                            video_set_cursor(video_get_rows()/2, 0);
                            video_printf(title_msg);
#ifdef USER_BUILD
                            video_printf("[Recovery    Mode]         <<==\n");
#ifdef MTK_FASTBOOT_SUPPORT
                            video_printf("[Fastboot    Mode]             \n");
#endif
                            video_printf("[Normal      Boot]             \n");
#else
                            video_printf("[Recovery    Mode]             \n");
#ifdef MTK_FASTBOOT_SUPPORT
                            video_printf("[Fastboot    Mode]             \n");
#endif
                            video_printf("[Normal      Boot]             \n");
                            video_printf("[Normal      Boot +ftrace] <<==\n");
#if defined(MEM_PRESERVED_MODE_ENABLE)
                            video_printf("[Normal      + mem-pre]       \n");
#endif //#if defined(MEM_PRESERVED_MODE_ENABLE)
                            video_printf("[Normal      slub debug off]     \n");
#endif
                        break;

#ifndef USER_BUILD
                        case 3:
#if defined(MEM_PRESERVED_MODE_ENABLE)
                            select = 4;
#else
                            select = 5;
#endif
                            video_set_cursor(video_get_rows()/2, 0);
                            video_printf(title_msg);
                            video_printf("[Recovery    Mode]             \n");
#ifdef MTK_FASTBOOT_SUPPORT
                            video_printf("[Fastboot    Mode]             \n");
#endif
                            video_printf("[Normal      Boot]             \n");
                            video_printf("[Normal      Boot +ftrace]     \n");
#if defined(MEM_PRESERVED_MODE_ENABLE)
                            video_printf("[Normal      + mem-pre]   <<==\n");
                            video_printf("[Normal      slub debug off]     \n");
#else
                            video_printf("[Normal      slub debug off] <<==\n");
#endif //#if defined(MEM_PRESERVED_MODE_ENABLE)
                        break;

#if defined(MEM_PRESERVED_MODE_ENABLE)
                        case 4:
                            select = 5;
                            video_set_cursor(video_get_rows()/2, 0);
                            video_printf(title_msg);
                            video_printf("[Recovery    Mode]             \n");
#ifdef MTK_FASTBOOT_SUPPORT
                            video_printf("[Fastboot    Mode]             \n");
#endif
                            video_printf("[Normal      Boot]             \n");
                            video_printf("[Normal      Boot +ftrace]     \n");
                            video_printf("[Normal      + mem-pre]       \n");
                            video_printf("[Normal      slub debug off] <<==\n");
                        break;
#endif  //#if defined(MEM_PRESERVED_MODE_ENABLE)

                        case 5:
                            select = 0;
                            video_set_cursor(video_get_rows()/2, 0);
                            video_printf(title_msg);
                            video_printf("[Recovery    Mode]         <<==\n");
#ifdef MTK_FASTBOOT_SUPPORT
                            video_printf("[Fastboot    Mode]             \n");
#endif
                            video_printf("[Normal      Boot]             \n");
                            video_printf("[Normal      Boot +ftrace]     \n");
#if defined(MEM_PRESERVED_MODE_ENABLE)
                            video_printf("[Normal      + mem-pre]       \n");
#endif
                            video_printf("[Normal      slub debug off]     \n");                            
                        break;
#endif  //#ifndef USER_BUILD

                        default:
                        break;
                    }
                    dprintf(0,  "[VOL_UP]Key Detect, current select:%d\n", select);
                    mdelay(300);
             }
             else if(mtk_detect_key(MT65XX_MENU_OK_KEY))//VOL_DOWN,
             {
                 //use for OK
                 break;
             }
             else
             {
                //pass
             }
          }
	#endif
          if(select == 0)
          {
               g_boot_mode = RECOVERY_BOOT;
          }
          else if(select == 1)
          {
                g_boot_mode = FASTBOOT;
          }
          else if(select == 2)
          {
                g_boot_mode = NORMAL_BOOT;
          }
          else if(select == 3)
          {
                sprintf(g_CMDLINE, "%s trace_buf_size=11m boot_time_ftrace", g_CMDLINE);
                g_boot_mode = NORMAL_BOOT;
          }
#if defined(MEM_PRESERVED_MODE_ENABLE)
          else if(select == 4)
          {
                dprintf(0, "With memory preserved mode, current select:%d\n", select);
                //enable mem-preserved mode.
                platform_mem_preserved_config(1);
                g_boot_mode = NORMAL_BOOT;
          }
#endif //#if defined(MEM_PRESERVED_MODE_ENABLE)
          else if (select == 5)
          {
                sprintf(g_CMDLINE, "%s slub_debug=-", g_CMDLINE);
                g_boot_mode = NORMAL_BOOT;
          }
          else{
                //pass
          }

          video_set_cursor(video_get_rows()/2 +8, 0);
          return;
}

BOOL boot_menu_key_trigger(void)
{
#if 1
	//wait
    ulong begin = get_timer(0);
    printf("\n%s Check  boot menu\n",MODULE_NAME);
    printf("%s Wait 50ms for special keys\n",MODULE_NAME);

    //let some case of recovery mode pass.
    if(unshield_recovery_detection())
    {
        return TRUE;
    }

    while(get_timer(begin)<50)
    {
		if(mtk_detect_key(MT65XX_BOOT_MENU_KEY))
		{
                           mtk_wdt_disable();
                           boot_mode_menu_select();
                           mtk_wdt_init();
                           return TRUE;
		}
    }
#endif
     return FALSE;
}

BOOL boot_menu_detection(void)
{
    return boot_menu_key_trigger();
}


int unshield_recovery_detection(void)
{
    //because recovery_check_command_trigger's BOOL is different from the BOOL in this file.
    //so use code like this type.
    return recovery_check_command_trigger()? TRUE:FALSE;
}

