
#include "zf_common_headfile.h"

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"

int main(int, char**) 
{

    while(1)
    {
        printf("key_0 = %d\r\n", gpio_get_level(KEY_0));
        printf("key_1 = %d\r\n", gpio_get_level(KEY_1));
        printf("key_2 = %d\r\n", gpio_get_level(KEY_2));
        printf("key_3 = %d\r\n", gpio_get_level(KEY_3));

        printf("switch_0 = %d\r\n", gpio_get_level(SWITCH_0));
        printf("switch_1 = %d\r\n", gpio_get_level(SWITCH_1));

        system_delay_ms(100);
    }
}