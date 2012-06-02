#include <Arduino.h>

int main(void)
{
	init();

#if defined(USB)
        USBConfig.attach();
#endif

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

