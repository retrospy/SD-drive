#define IO0 0
#define IO1 1
#define IO2 2
#define IO3 3
#define IO4 4
#define IO5 5
#define IO6 6
#define IO7 7
#define DIR_245 10
#define DIRECTION 11
#define STROBE 12
#define ACK 13

#define WAIT_LEADING_EDGEB( pin ) while( gpio_get(pin) ){} while( !gpio_get(pin) ){}
#define READ_PINS ((((gpio_get_all() >> 3) & 0xFF) | gpio_get_all()) & 0x1FF)


static int count = 0;

void setup()
{
	Serial.begin(9600);
	
	while (!Serial) ;
	
	count = 0;

	for (int i = 0; i < 14; ++i)
	{
		if (i == DIR_245)
		{
			pinMode(i, OUTPUT);
			digitalWriteFast(DIR_245, LOW);
		}
		else
			pinMode(i, INPUT);
	}
	
}

void loop()
{
	WAIT_LEADING_EDGEB(ACK);
	for (int i = 0; i < 512; ++i)
	{ 
		WAIT_LEADING_EDGEB(STROBE);
		
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
