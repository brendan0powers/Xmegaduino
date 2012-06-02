#include <Arduino.h>

int main(void)
{
	init();

#if defined(USB)
        USBDevice.attach();
#endif

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

