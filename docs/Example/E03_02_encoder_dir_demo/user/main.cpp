
#include "zf_common_headfile.h"


int16 encoder_left;
int16 encoder_right;

#define ENCODER_1           "/dev/zf_encoder_1"
#define ENCODER_2           "/dev/zf_encoder_2"

void pit_callback()
{
    encoder_left  = encoder_get_count(ENCODER_1);
    encoder_right = encoder_get_count(ENCODER_2);
}


int main(int, char**) 
{

    // 创建一个定时器10ms周期，回调函数为pit_callback
    pit_ms_init(5, pit_callback);
 
    while(1)
    {

        printf("zf_encoder_left = %d\r\n", encoder_left);
        printf("zf_encoder_right = %d\r\n", encoder_right);

        system_delay_ms(100);
    }
}