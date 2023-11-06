
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

void setup()
{
	Serial.begin(9600);
	
	for (int i = 0; i < 8; ++i)
		pinMode(i, OUTPUT);
	
	pinMode(DIRECTION, OUTPUT);
	pinMode(STROBE, OUTPUT);
	pinMode(ACK, OUTPUT);
}

void loop()
{
	int r = 0;
	while (true)
	{
		Serial.print("here :");
		Serial.println(r++);
   
		digitalWrite(ACK, HIGH);
		delay(1);
  
		for (int i = 0; i < 512; ++i)
		{
			digitalWrite(DIRECTION, (i & 0b100000000) != 0 ? HIGH : LOW);
			digitalWrite(IO7, (i & 0b010000000) != 0 ? HIGH : LOW);
			digitalWrite(IO6, (i & 0b001000000) != 0 ? HIGH : LOW);
			digitalWrite(IO5, (i & 0b000100000) != 0 ? HIGH : LOW);
			digitalWrite(IO4, (i & 0b000010000) != 0 ? HIGH : LOW);
			digitalWrite(IO3, (i & 0b000001000) != 0 ? HIGH : LOW);
			digitalWrite(IO2, (i & 0b000000100) != 0 ? HIGH : LOW);
			digitalWrite(IO1, (i & 0b000000010) != 0 ? HIGH : LOW);
			digitalWrite(IO0, (i & 0b000000001) != 0 ? HIGH : LOW);

			digitalWrite(STROBE, HIGH);
			delay(1);
			digitalWrite(STROBE, LOW);
			delay(1);
		}
		digitalWrite(ACK, LOW);
		delay(1);
	}
}
