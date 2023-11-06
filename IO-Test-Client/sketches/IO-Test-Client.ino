#if defined(ARDUINO_RASPBERRY_PI_PICO)
#define IO0 0
#define IO1 1
#define IO2 2
#define IO3 3
#define IO4 4
#define IO5 5
#define IO6 6
#define IO7 7
#define DIRECTION 11
#define STROBE 12
#define ACK 13
#else
#define IO0 2
#define IO1 3
#define IO2 4
#define IO3 5
#define IO4 6
#define IO5 7
#define IO6 8
#define IO7 9
#define DIRECTION 10
#define STROBE 11
#define ACK 12
#endif

#if defined(ARDUINO_RASPBERRY_PI_PICO)
#define WAIT_LEADING_EDGEB( pin ) while( gpio_get(pin+9) ){} while( !gpio_get(pin+9) ){}
#define READ_PINS ((gpio_get_all() >> 3) | gpio_get_all()) & 0x1FF
#else
#define PINB_READ( pin ) (PINB&(1<<(pin)))
#define READ_PINS ((PINB & 0b00000111) << 6) | ((PIND & 0b11111100) >> 2);
#define WAIT_LEADING_EDGEB( pin ) while( PINB_READ(pin) ){} while( !PINB_READ(pin) ){}
#endif

static int count = 0;

void setup()
{
	Serial.begin(9600);
	
	while (!Serial) ;
	
	count = 0;
#if defined(ARDUINO_RASPBERRY_PI_PICO)
	for (int i = 0; i < 14; ++i)
	{
		pinMode(i, INPUT_PULLUP);
	}
#else
	for (int i = 2; i < 13; ++i)
	{
		pinMode(i, INPUT_PULLUP);
	}
#endif
}

void loop()
{
	WAIT_LEADING_EDGEB(4);
	for (int i = 0; i < 512; ++i)
	{ 
		WAIT_LEADING_EDGEB(3);
		
		int val = READ_PINS;

		if (val != i)
		{
			interrupts();
			Serial.print("Expected: ");
			Serial.print(i);
			Serial.print("  Got: ");
			Serial.println(val);
			Serial.print("Cycle #");
			Serial.print(++count);
			Serial.println(" FAILED");
			while (true) ;
		}
		
		Serial.print("Cycle #");
		Serial.print(++count);
		Serial.println(" PASSED");	
	}
}
