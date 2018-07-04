#include "my_delay.h"

uint16_t delay_count=0;
#define delay_const  (unsigned long) (336000000 / 15574784)

void TimeTickDec(void)
{
	if(delay_count != 0x00)
		--delay_count;
}
void delay_ms(uint16_t ms)
{
	delay_count=ms;
	while(delay_count != 0x00);
}
/*
void delay_us(uint32_t us)
{
	while(ms != 0x00 )
	{
		--ms;
		delay_us(1000);
	}

	 us*= delay_const;
	 while(us--);
}
*/

