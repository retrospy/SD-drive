#define DIRECTION 10
#define STROBE 11
#define ACK 12

//#define DIR_245 10

void setup()
{
	pinMode(DIRECTION, OUTPUT);
	pinMode(STROBE, OUTPUT);
	//  pinMode(DIR_245, OUTPUT);
	pinMode(ACK, INPUT);
  
	//  digitalWrite(DIR_245, HIGH);
  
	for (int i = 2; i < 10; ++i)
		pinMode(i, OUTPUT);
  
	Serial.begin(115200);
  
	while (!Serial) ;
  
	Serial.println("setup complete");
}

#define DEBOUNCE_COUNT 5

bool debounceInputPin(int pin)
{
	bool val, last; // it's okay not to initialize them
	int goodCount = 0;

	do
	{
		if ((last = digitalRead(pin)) == val)
		{
			goodCount++;
		}
		else
		{
			val = last; // new value
			goodCount = 0; // start counting again
		}
	} while (goodCount < DEBOUNCE_COUNT);

	return val;
}

int i = 0;

void loop()
{
	delay(1000);
  
	Serial.print("SETTTING DATA TO ");
	Serial.println(i, HEX);
	digitalWrite(2, (i & 0x01) == 0 ? LOW : HIGH);
	digitalWrite(3, (i & 0x02) == 0 ? LOW : HIGH);
	digitalWrite(4, (i & 0x04) == 0 ? LOW : HIGH);
	digitalWrite(5, (i & 0x08) == 0 ? LOW : HIGH);
	digitalWrite(6, (i & 0x10) == 0 ? LOW : HIGH);
	digitalWrite(7, (i & 0x20) == 0 ? LOW : HIGH);
	digitalWrite(8, (i & 0x40) == 0 ? LOW : HIGH);
	digitalWrite(9, (i & 0x80) == 0 ? LOW : HIGH);
  
	Serial.println("STROBE GOES HIGH");
	digitalWrite(STROBE, HIGH);
  
	Serial.println("DIRECTION GOES HIGH");
	digitalWrite(DIRECTION, HIGH);
  
	Serial.println("WAITING FOR ACK TO GO HIGH");
	while (debounceInputPin(ACK) == LOW) ;
	Serial.println("ACK GOES HIGH");
  
	Serial.println("STROBE GOES LOW");
	digitalWrite(STROBE, LOW);
  
	Serial.println("WAITING FOR ACK TO GO LOW");
	while (debounceInputPin(ACK) == HIGH) ;
	Serial.println("ACK GOES LOW");
  
	Serial.println("DIRECTION GOES LOW");
	digitalWrite(DIRECTION, LOW);

	//  // RECEIVE
	Serial.println("++++++++++++++++++++++");

	Serial.println("WAITING FOR ACK TO GO HIGH");
	while (debounceInputPin(ACK) == LOW) ;
	Serial.println("ACK GOES HIGH");

	for (int j = 2; j < 10; ++j)
		pinMode(j, INPUT);

	int data = 0;
	for (int j = 2; j < 10; ++j)
	{
		if (digitalRead(j) == HIGH)
			data |= (1 << (j - 2));
	}

	Serial.println("STROBE GOES HIGH");
	digitalWrite(STROBE, HIGH);

	Serial.println("WAITING FOR ACK TO GO LOW");
	while (debounceInputPin(ACK) == HIGH) ;
	Serial.println("ACK GOES LOW");

	Serial.println("STROBE GOES LOW");
	digitalWrite(STROBE, LOW);

	Serial.print("Got byte from host: ");
	Serial.println(data, HEX);  

	for (int j = 2; j < 10; ++j)
		pinMode(j, OUTPUT);


	Serial.println("----------------------");

	i = (i + 1) % 256;
}
