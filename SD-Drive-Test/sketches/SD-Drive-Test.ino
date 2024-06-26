//=============================================================================
// SD Shield Tester
//
// This is for testing the Corsham Technologies, LLC SD Shield.  It might be
// useful for other things, but I really doubt it.  It is a menu based program
// that tests basic functionality of the board.  Generally speaking, as soon as
// the Arduino headers, SD socket and buffer IC are installed, I run this to
// make sure the SD is card is detected ('I' command) and the SD card data can
// be read ('S' command).  If they pass, the other parts are installed and the
// rest of the tests are checked.
//
// Mid-2017 by Bob Applegate K2UT, bob@corshamtech.com
//
// v1.2 01/01/2019  Bob Applegate
//                  Added much better RTC test code.  Added more comments.
// v1.3 03/23/2020  Bob Applegate
//                  Lots of little changes and fixes.
// v1.4 11/05/2023  Christopher 'Zoggins' Mallery
//                  Ported to Raspberry Pi Pico

#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>

// The pins used on the newer SD Shields.

#if defined(ARDUINO_AVR_MEGA2560)
#define NEW_SHIELD_PIN  38
#define OPTION_1_PIN    42
#define OPTION_2_PIN    41
#define OPTION_3_PIN    40
#define OPTION_4_PIN    39
#define TIMER_OUT_PIN   43    //maps to pin 18, CB1
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define NEW_SHIELD_PIN  XX
#define OPTION_1_PIN    14
#define OPTION_2_PIN    15
#define OPTION_3_PIN    9
#define OPTION_4_PIN    XX
#define TIMER_OUT_PIN   22    //maps to pin 18, CB1
#endif

// Pin used by the SD card

#if defined(ARDUINO_AVR_MEGA2560)
#define SD_PIN  53
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define SD_PIN  17
#endif

// Pin with the presence sensor

#if defined(ARDUINO_AVR_MEGA2560)
#define PRESENCE_PIN 19
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define PRESENCE_PIN  8
#endif

// Define the pins used for the LEDs

#if defined(ARDUINO_AVR_MEGA2560)
#define RED_LED_PIN  24
#define YELLOW_LED_PIN  23
#define GREEN_LED_PIN  22
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define RED_LED_PIN  26
#define YELLOW_LED_PIN  27
#define GREEN_LED_PIN  28
#endif

// This is the I2C address of the DS3231 RTC.

#define DS3231_I2C_ADDRESS  0x68

// Trying to remember if HIGH or LOW turns the LEDs on or off, so these
// two macros make it easier.

#define LED_OFF  HIGH
#define LED_ON   LOW

int shieldRevision;

bool sdPresent;
bool rtcPresent;



//=============================================================================
// Initialization routine.

void setup()
{
	Serial.begin(9600);
	while (!Serial) ;
	
#if defined(ARDUINO_RASPBERRY_PI_PICO)
	Wire.setSCL(21);
	Wire.setSDA(20);
#endif
	
	Serial.println("\n\n\nCorsham Tech SD Shield Tester version 1.4");
	Serial.println("Written by Bob Applegate K2UT, bob@corshamtech.com");
	Serial.println("Updated by Christopher 'Zoggins' Mallery, zoggins@retro-spy.com");
	
	// Find what revision this shield is and set a variable.
	// This value changes what needs to be tested.

	shieldRevision = getShieldRevision();
	Serial.print("This appears to be a revision ");
	Serial.print(shieldRevision);
	Serial.println(" shield.  Verify this is correct.");

	pinMode(SD_PIN, OUTPUT); // required by SD library

	// Configure the new option pins as inputs with pull-ups
#if defined(ARDUINO_AVR_MEGA2560)
	pinMode(NEW_SHIELD_PIN, INPUT_PULLUP);
	pinMode(OPTION_4_PIN, INPUT_PULLUP);
#endif
	pinMode(OPTION_1_PIN, INPUT_PULLUP);
	pinMode(OPTION_2_PIN, INPUT_PULLUP);
	pinMode(OPTION_3_PIN, INPUT_PULLUP);


	pinMode(PRESENCE_PIN, INPUT); // pin with presence bit
	sdPresent = digitalRead(PRESENCE_PIN); // get current state

	pinMode(RED_LED_PIN, OUTPUT);
	digitalWrite(RED_LED_PIN, LED_OFF);
        
	pinMode(YELLOW_LED_PIN, OUTPUT);
	digitalWrite(YELLOW_LED_PIN, LED_OFF);
        
	pinMode(GREEN_LED_PIN, OUTPUT);
	digitalWrite(GREEN_LED_PIN, LED_OFF);

	// Configure new output pins
	pinMode(TIMER_OUT_PIN, OUTPUT);

	SD.begin(SD_PIN);

	Wire.begin();
}




//=============================================================================
// Get keys, process commands.

void loop()
{
	int key;
        
	printMenu();

	// Now wait for the user to send a command

	do
	{
		key = Serial.read();
	} while (!isprint(key));
        
	switch (key)
	{
	case 'c':
	case 'C':
		cycleLEDs();
		break;

	case 'i':
	case 'I':
		doInstalledTest();
		break;

	case 'o':
	case 'O':
		doShowOptionSwitches();
		break;

	case 's':
	case 'S':
		sdAccessTest();
		break;

	case 'R':
	case 'r':
		doRtcTest();
		break;

	case 'T':
	case 't':
		setTime();
		break;

	case 'G':
	case 'g':
		getTime();
		break;
	}
}




//=============================================================================
// Prints the menu of valid commands.  Takes into account which shield version
// this is.

void printMenu(void)
{
	Serial.println("");
	Serial.println("Options:");
	Serial.println("   C = Cycle all LEDs");

	if (shieldRevision > 1)
	{
		Serial.println("   O = Show option switch settings");
	}
	Serial.println("   I = Show if SD card is installed or not");
	Serial.println("   S = SD card access test");
	Serial.println("   R = RTC test");
	Serial.println("   T = Set RTC time");
	Serial.println("   G = Get RTC tiime");
	Serial.println("");
}




//=============================================================================
// Display whether the SD is installed or not.

void doInstalledTest(void)
{
	sdPresent = digitalRead(PRESENCE_PIN);
        
	if (sdPresent)
	{
		Serial.println("SD is not installed");
	}
	else
	{
		Serial.println("SD is installed");
	}
}




//=============================================================================
// Displays the current state of each of the option switches.

void doShowOptionSwitches(void)
{
	Serial.print("Option 1: ");
	Serial.println(digitalRead(OPTION_1_PIN) ? "Off" : "On");

	Serial.print("Option 2: ");
	Serial.println(digitalRead(OPTION_2_PIN) ? "Off" : "On");

	Serial.print("Option 3: ");
	Serial.println(digitalRead(OPTION_3_PIN) ? "Off" : "On");

#if defined(ARDUINO_AVR_MEGA2560)
	Serial.print("Option 4: ");
	Serial.println(digitalRead(OPTION_4_PIN) ? "Off" : "On");
#endif
}




//=============================================================================
// This returns the version of the shield.  Currently it only supports old
// (version 1) or new (version 2).  Returns 1 or 2.

int getShieldRevision(void)
{
#if defined(ARDUINO_AVR_MEGA2560)
	if (digitalRead(NEW_SHIELD_PIN))
		return 1;
	else
		return 2;
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
	return 3;
#endif
}




//=============================================================================
// This test to see if we can read the SD card.  It doesn't need to do much,
// just basic access is enough to prove the board is working.

void sdAccessTest(void)
{
	File root; // root directory

	if (digitalRead(PRESENCE_PIN))
	{
		Serial.println("SD is not installed; can't run test.");
		return;
	}

	SD.begin(SD_PIN);
	root = SD.open("/");
	if (root < 0)
	{
		Serial.println("Could not read the card");
		return;
	}
	Serial.println("Opened the SD, attempting to read files...");

	bool done = true;
	while (done)
	{
		File entry =  root.openNextFile();
		if (!entry)
		{
			// no more files
			done = false;
		}
		else
		{
			Serial.print(entry.name());
			if (entry.isDirectory())
			{

			}
			else
			{
				// files have sizes, directories do not
				Serial.print("\t\t");
				Serial.println(entry.size(), DEC);
			}
		}
		entry.close();
	}
	
	File file = SD.open("SD00BOOT.dsk", O_RDWR);   
	if (!file)
	{
		Serial.println("Error opening file!");
		// should set an error code
	}
	else
	{
		Serial.println("File opened");
		file.close();
	}
	
	if (SD.exists("TEST"))
		SD.rmdir("TEST");
	
	//SD.close();
	Serial.println("SD card access succeeded");
}




//=============================================================================
// The RTC test sets the time/date to a specific value, delays a couple of
// seconds and verifies the clock is ticking.

void doRtcTest(void)
{
	byte second, minute, hour, dayOfWeek, dayOfMonth, month, year, second2;

	Serial.print("Testing the RTC... ");

	// Just in case the clock is already set and running, grab the
	// current time, delay a bit and see if it's ticking...
        
	readDS3231time(&second,
		&minute,
		&hour,
		&dayOfWeek,
		&dayOfMonth,
		&month,
		&year);
	delay(2000);
	readDS3231time(&second2,
		&minute,
		&hour,
		&dayOfWeek,
		&dayOfMonth,
		&month,
		&year);
	if (second != second2)
	{
		Serial.println("RTC was already set and is ticking - passed");
		return;
	}

	// Clock might not be ticking, so let's set the time which
	// will start the clock.

	Serial.println("clock wasn't set already, trying to set it now...");
        
	setRtcTime(1, 1, 1, 1, 1, 1, 19);

	// Make sure the time just set was actually set.  I've had
	// chips always return the same value when not running.
        
	readDS3231time(&second,
		&minute,
		&hour,
		&dayOfWeek,
		&dayOfMonth,
		&month,
		&year);
	if (minute != 1 || hour != 1)
	{
		Serial.println("Clock didn't take value; seems bad");
	}
	else
	{
		delay(2500); // delay just over two seconds

		readDS3231time(&second,
			&minute,
			&hour,
			&dayOfWeek,
			&dayOfMonth,
			&month,
			&year);
		//Serial.print("Read seconds: ");
		//Serial.println(second);
        
		if (second >= 3)
		{
			Serial.println("RTC is ticking - passed");
		}
		else
		{
			Serial.println("Clock didn't seem to tick, but try again.");
			Serial.println(second);
		}
	}
}




//=============================================================================
// This turns on each lED for one second.

void cycleLEDs(void)
{
	digitalWrite(YELLOW_LED_PIN, LED_OFF);
	digitalWrite(RED_LED_PIN, LED_OFF);

	digitalWrite(GREEN_LED_PIN, LED_ON);
	delay(1000);
	digitalWrite(GREEN_LED_PIN, LED_OFF);

	digitalWrite(YELLOW_LED_PIN, LED_ON);
	delay(1000);
	digitalWrite(YELLOW_LED_PIN, LED_OFF);

	digitalWrite(RED_LED_PIN, LED_ON);
	delay(1000);
	digitalWrite(RED_LED_PIN, LED_OFF);
}




//=============================================================================
// The low-level routine to set the time and date.
// Directly from tronixstuff.  Thank them!

void setRtcTime(byte second,
	byte minute,
	byte hour,
	byte dayOfWeek,
	byte dayOfMonth,
	byte month,
	byte year)
{
	// sets time and date data to DS3231
	Wire.beginTransmission(DS3231_I2C_ADDRESS);
	Wire.write(0); // set next input to start at the seconds register
	Wire.write(decToBcd(second)); // set seconds
	Wire.write(decToBcd(minute)); // set minutes
	Wire.write(decToBcd(hour)); // set hours
	Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
	Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
	Wire.write(decToBcd(month)); // set month
	Wire.write(decToBcd(year)); // set year (0 to 99)
	Wire.endTransmission();
}




//=============================================================================
// Low level function to get the time and date from the RTC.
// Directly from tronixstuff.  Thank them!

void readDS3231time(byte *second,
	byte *minute,
	byte *hour,
	byte *dayOfWeek,
	byte *dayOfMonth,
	byte *month,
	byte *year)
{
	Wire.beginTransmission(DS3231_I2C_ADDRESS);
	Wire.write(0); // set DS3231 register pointer to 00h
	Wire.endTransmission();
	Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
	// request seven bytes of data from DS3231 starting from register 00h
	*second = bcdToDec(Wire.read() & 0x7f);
	*minute = bcdToDec(Wire.read());
	*hour = bcdToDec(Wire.read() & 0x3f);
	*dayOfWeek = bcdToDec(Wire.read());
	*dayOfMonth = bcdToDec(Wire.read());
	*month = bcdToDec(Wire.read());
	*year = bcdToDec(Wire.read());
}




//=============================================================================
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
	return (val / 10 * 16) + (val % 10);
}




//=============================================================================
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
	return (val / 16 * 10) + (val % 16);
}




//=============================================================================
// Set the RTC time.  Usually used when done with tests to set the initial time
// in the hardware.

void setTime(void)
{
	byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

	Serial.setTimeout(30 * 1000);
	Serial.print("Enter month: ");
	month = Serial.parseInt();

	Serial.print("\nEnter day of month: ");
	dayOfMonth = Serial.parseInt();

	Serial.print("\nEnter last two digits of year: ");
	year = Serial.parseInt();

	Serial.print("\nEnter day of week, 1 = Sunday, 2 = Monday, 3 = Tuesday, etc: ");
	dayOfWeek = Serial.parseInt();

	Serial.print("\nEnter hour: ");
	hour = Serial.parseInt();

	Serial.print("\nEnter minute: ");
	minute = Serial.parseInt();

	Serial.print("\nEnter second: ");
	second = Serial.parseInt();

#if 0
	Serial.println(month);
	Serial.println(dayOfMonth);
	Serial.println(year);
	Serial.println(dayOfWeek);
	Serial.println(hour);
	Serial.println(minute);
	Serial.println(second);
#endif

	setRtcTime(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
}




//=============================================================================
// This gets and displays the RTC values in an easy to understand format.  The
// primary use for this is to get/display the time after the RTC has been set
// and then power cycled.  Ie, did the battery keep the clock ticking.

void getTime(void)
{
	byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

	// Get the values from the RTC...
        
	readDS3231time(&second,
		&minute,
		&hour,
		&dayOfWeek,
		&dayOfMonth,
		&month,
		&year);

	Serial.print("Date: ");
	Serial.print(month);
	Serial.print("/");
	Serial.print(dayOfMonth);
	Serial.print("/");
	Serial.println(2000 + year);

	// Print the day of the week as a day, not number.
        
	Serial.print("Day of week: ");
	switch (dayOfWeek)
	{
	case 1:
		Serial.println("Sunday (1)");
		break;

	case 2:
		Serial.println("Monday (2)");
		break;

	case 3:
		Serial.println("Tuesday (3)");
		break;

	case 4:
		Serial.println("Wednesday (4)");
		break;

	case 5:
		Serial.println("Thursday (5)");
		break;

	case 6:
		Serial.println("Friday (6)");
		break;

	case 7:
		Serial.println("Saturday (7)");
		break;
	}

	// Not perfect; displays small values as single digits.
	// Oh, to have a real printf funcion...
        
	Serial.print("Time: ");
	Serial.print(hour);
	Serial.print(":");
	Serial.print(minute);
	Serial.print(":");
	Serial.println(second);
}