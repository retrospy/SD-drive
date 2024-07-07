#include "parallel_io.h"

#define DIRECTION 11
#define STROBE 12
#define ACK 13

#define DIR_245 10

#define OPTION_1_PIN    14
#define OPTION_2_PIN    15
#define OPTION_3_PIN    9
#define OPTION_4_PIN    10
#define TIMER_OUT_PIN   22    //maps to pin 18, CB1
#define PRESENCE_PIN  8
#define RED_LED_PIN  26
#define YELLOW_LED_PIN  27
#define GREEN_LED_PIN  28

#define LED_OFF  HIGH
#define LED_ON   LOW

bool classicMode = false;

void setup()
{

	Serial.begin(115200);	
	while (!Serial) ;
	
	pinMode(OPTION_1_PIN, INPUT_PULLUP);
	pinMode(OPTION_2_PIN, INPUT_PULLUP);
	pinMode(OPTION_3_PIN, INPUT_PULLUP);
	pinMode(OPTION_4_PIN, INPUT_PULLUP);
	
	if (digitalRead(OPTION_1_PIN) == HIGH)
	{
		Serial.println("starting classic mode");
		classicMode = true;
		pinMode(DIRECTION, INPUT);
		pinMode(STROBE, INPUT);
		pinMode(DIR_245, OUTPUT);
		pinMode(ACK, OUTPUT);
	
		for (int j = 0; j < 8; ++j)
			pinMode(j, INPUT);
		
		digitalWrite(ACK, LOW);
		digitalWrite(DIR_245, LOW);
	}
	else
	{
		Serial.println("starting PIO mode");
		classicMode = false;
		uint offset = pio_add_program(pio0, &parallel_io_program);
		parallel_output_program_init(pio0, 0, offset);
	}
	
	pinMode(RED_LED_PIN, OUTPUT);
	digitalWrite(RED_LED_PIN, LED_OFF);
        
	pinMode(YELLOW_LED_PIN, OUTPUT);
	digitalWrite(YELLOW_LED_PIN, LED_OFF);
        
	pinMode(GREEN_LED_PIN, OUTPUT);
	digitalWrite(GREEN_LED_PIN, LED_OFF);
		
	pinMode(TIMER_OUT_PIN, OUTPUT);

	
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

int i = 0xFF;

void loop()
{
	
	int data;
	if (classicMode)
	{
		if (digitalRead(STROBE) == HIGH)
		{
			Serial.println("HOST HAS DATA");		
			
			digitalWrite(DIR_245, LOW);
			data = 0;
			for (int j = 0; j < 8; ++j)
			{
				if (digitalRead(j) == HIGH)
					data |= (1 << j);
			}
			
			Serial.println("ACK GOES HIGH");
			digitalWrite(ACK, HIGH);
			
			Serial.println("WAITING FOR STROBE TO GO LOW");
			while (digitalRead(STROBE) == HIGH) ;
			Serial.println("STROBE GOES LOW");
			
			Serial.println("ACK GOES LOW");
			digitalWrite(ACK, LOW);
			
			Serial.print("Got byte from host: ");
			Serial.println(data, HEX);	
			
			
			Serial.print("Wait to send to host: ");
			Serial.println(i, HEX);
			
			Serial.println("WAITING FOR DIRECTION TO GO LOW");
			while (digitalRead(DIRECTION) == HIGH) ;
			Serial.println("DIRECTION GOES LOW");
			
			for (int j = 0; j < 8; ++j)
				pinMode(j, OUTPUT);
			
			digitalWrite(DIR_245, HIGH); 
			
			Serial.print("SETTTING DATA TO ");
			Serial.println(i, HEX);
			digitalWrite(0, (i & 0x01) == 0 ? LOW : HIGH);
			digitalWrite(1, (i & 0x02) == 0 ? LOW : HIGH);
			digitalWrite(2, (i & 0x04) == 0 ? LOW : HIGH);
			digitalWrite(3, (i & 0x08) == 0 ? LOW : HIGH);
			digitalWrite(4, (i & 0x10) == 0 ? LOW : HIGH);
			digitalWrite(5, (i & 0x20) == 0 ? LOW : HIGH);
			digitalWrite(6, (i & 0x40) == 0 ? LOW : HIGH);
			digitalWrite(7, (i & 0x80) == 0 ? LOW : HIGH);
			
			Serial.println("ACK GOES HIGH");
			digitalWrite(ACK, HIGH);
			
			Serial.println("WAITING FOR STROBE TO GO HIGH");
			while (digitalRead(STROBE) == LOW) ;
			Serial.println("STROBE GOES HIGH");
			
			Serial.println("ACK GOES LOW");
			digitalWrite(ACK, LOW);
		
			Serial.println("WAITING FOR STROBE TO GO LOW");
			while (digitalRead(STROBE) == HIGH) ;
			Serial.println("STROBE GOES LOW");
			
			for (int j = 0; j < 8; ++j)
				pinMode(j, INPUT);
			
			i = (i - 1);
			if (i < 0)
				i = 0xFF;
		}
	}
	else
	{
		if (parallel_io_has_data(pio0, 0))
		{
			Serial.println("HAS DATA IN FIFO");	
			data = parallel_io_getc(pio0, 0);				
			Serial.print("Got byte from host: ");
			Serial.println(data, HEX);
			
			Serial.print("Sent byte to host: ");
			Serial.println(i, HEX);
			parallel_io_putc(pio0, 0, i << 8 | 0xFF);				
	
			i = (i - 1);
			if (i < 0)
				i = 0xFF;
		}
	}
	


}
