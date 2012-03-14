#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <mach/lprintk.h>
#include <mach/board-star-debug.h>

int star_default_loglevel = 0;  
void star_set_loglevel(int level)
{
  console_loglevel = level;
}

void star_save_default_loglevel()
{
   star_default_loglevel = console_loglevel;  
}

void star_restore_loglevel()
{
   console_loglevel = star_default_loglevel;
}

void star_set_verbose_loglevel()
{
    console_loglevel = 15;
}

#if defined (CONFIG_STAR_DEBUG)
extern int console_suspend_enabled;
int star_debug_enable = 0;
int __init star_get_debug_type(char *str)
{
    unsigned char reset_reason[5];
    strcpy(reset_reason,str);
    if (reset_reason[0] == 'z' )
    {
      printk("suspend debug mode enabled!\n");
      star_save_default_loglevel();
      console_suspend_enabled = 0;
      star_debug_enable = 1;
    }
    return 0;
}

int is_star_suspend_debug()
{
    return star_debug_enable;
}
#else
int star_get_debug_type(char *str)
{
  return 0;
}

int is_star_suspend_debug()
{
 return 0;
}
#endif
