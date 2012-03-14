#ifndef __BOARD_STAR_DEBUG_H
#define __BOARD_STAR_DEBUG_H

#if 1
#define STAR_SUSPEND_DEBUG_DISABLE			0            
#define STAR_SUSPEND_DEBBUG_WATCHDOG        1
#define STAR_SUSPEND_DEBUG_WATCHDOG_LOG     2

extern int is_star_suspend_debug();
#define SUSPEND_LOG(format, args...) if (is_star_suspend_debug()) { \
                                      printk(format, ## args); \
                                  }
#else
#define SUSPEND_LOG(x...) do { } while(0)
#endif

#endif
 
 
