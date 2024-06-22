//=============================================================================
// This sketch provides an interface between older 8 bit processors and an
// SD card via a parallel port.  This is not a perfectly clean implementation
// and can be improved, but it's good enough to get the job done.
//
// This came about when I was building an SWTPC clone system for Corsham
// Technologies and people kept asking if I was going to design a disk controller
// card.  I looked at existing designs but decided there were better modern
// technologies and decided to use a micro SD card instead.  It would allow
// a modern computer to put files to/from the SD, and use the common DSK file
// format to access the large number of available collections of files for
// older machines.
//
// This uses a Link class object to communicate with the other device.  This
// was initially a parallel connection but can be almost any kind of data
// path.
//
// For terminology, the other processor is the host, and the Arduino (this
// code) is the slave.  The host drives all communications and controls
// the flow of data.
//
// This was originally done on a basic Arduino with an ATMEGA328 but it was
// a constant battle to reduce RAM usage, so I switched to a MEGA instead.
// The code is a bit easier to follow now, and a lot more functionality
// can be added as desired without running out of code space.  However, you'll
// notice inconsistent coding styles as I was trying to make this lean-and-mean
// at first, but then starting doing things a bit cleaner later.  Feel free
// to clean up code or make it better... please!
//
// Legal mumbo-jumbo: I don't really follow all the current licensing
// nonsense, so I'll just clearly state my desires: This code includes
// other open code, such as the Arduino SD library.  Observe their
// requirements.  I simply ask: (A) anything derived from this code must
// include my name/email as the original author, (B) any commerical use must
// make the current code available for free to users, and (C) I'd like an
// email letting me know what you're doing with the code, what platforms
// you're using it on, etc... it's fun to see how others are using it!
//
// Note that the protocol used between the host and this code is The
// Remote Disk Protocol Guide, available for download from the
// Corsham Tech website.
//
// This is NOT a complete implementation of the protocol!  Know problems
// (but there are probably others):
//
//    * Only supports 256 byte sectors
//    * RTC messages do not handle fields with values of FF nor the century byte
//
// 04/11/2014 - Bob Applegate, Corsham Technologies.  bob@corshamtech.com
//              wwww.corshamtech.com
//
// Revision 1.1 enhancements:
//    * Supports new SD Shield from Corsham Technologies, which has more
//      pins brought to the parallel port, interrupt capabilities, option
//      switches, etc.
//
// Revision 1.2:
//    * Now supports the new sector read/write LONG commands in the
//      protocol guide rev 1.1.  Also added the ability to boot from an
//      alternate config file.
//
// Revision 1.3:
//    * General code clean-up.  Added debounce code for all digital inputs.
//
// Revision 1.4:
//    * Fixed bug that caused all DSK file updates to be appended rather than applied
//      to the requested sector.  Removed a bit of excess debugging output.
//
// Revision 1.5:
//    * Ported to Raspberry Pi Pico

#include <Arduino.h>

#ifdef USE_SDFAT
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>
#endif

#include <SPI.h>
#include <SD.h>
#include "link.h"
#include "Disks.h"
#include "UserInt.h"
#include <Wire.h>
#include "RTC.h"
#include "Errors.h"
#include "SdFuncs.h"


// Debugging options.  They usually produce lots of serial output so be careful what you turn on.

#undef DEBUG_SECTOR_READ
#undef DEBUG_DIR
#undef DEBUG_FILE_READ
#undef DEBUG_FILE_WRITE
#undef DEBUG_SAVE_CONFIG
#undef DEBUG_SET_TIMER



// This is the speed of the faster timer processing, expressed in milliseconds.

#define FAST_POLL_DELAY 10    // 1/100th of a second

// This sets the intervale between the passes through the poll functions, expressed
// in milliseconds.

#define POLL_DELAY  100      // 1/10th of a second

// Number of times an input pin must be the same before it is counted.

#define DEBOUNCE_COUNT      5

// The link to the remote system.

Link *link;

// This is the collection of disks

Disks *disks;

// This is the user interface.  Its a singleton.

UserInt *uInt;

// The real time clock

RTC *rtc;

static unsigned long nextPoll;

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
#define OPTION_4_PIN    10
#define TIMER_OUT_PIN   22    //maps to pin 18, CB1
#endif

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

#define LED_OFF  HIGH
#define LED_ON   LOW

// This is the I2C address of the DS3231 RTC.
#define DS3231_I2C_ADDRESS  0x68

// Counters used for the timer.
unsigned int timerValue = 0;
unsigned int timerCount = 0;
int pollCounter = 0;
        
static bool testerOn = false;

static int shieldRevision;
static bool sdPresent;
static bool rtcPresent;

//=============================================================================
// This is the usual Arduino setup function.  Do initialization, then return.

void setup()
{
	pinMode(OPTION_2_PIN, INPUT_PULLUP);
	if (digitalRead(OPTION_2_PIN) == HIGH)
	{
		testerOn = false;
		Serial.begin(9600);

		//while (!Serial);
	
		#if defined(ARDUINO_RASPBERRY_PI_PICO)
		Wire.setSCL(21);
		Wire.setSDA(20);
		#endif
	
		Serial.println("");
		Serial.println("SD Drive version 1.5");
		Serial.println("Brought to you by Bob Applegate and Corsham Technologies");
		Serial.println("bob@corshamtech.com, www.corshamtech.com");
		Serial.println("Updated by Christopher 'Zoggins' Mallery and RetroSpy Technologies");
		Serial.println("zoggins@retro-spy.com, retro-spy.com");
        
		// Start up the UI soon so it can display some initial info
		// while the rest of the system comes up.
        
		uInt = UserInt::getInstance();
        
		pinMode(SD_PIN, OUTPUT); // required by SD library

		// Configure the new option pins as inputs with pull-ups
        
		#if defined(ARDUINO_AVR_MEGA2560)
		pinMode(NEW_SHIELD_PIN, INPUT_PULLUP);
		#endif
		pinMode(OPTION_1_PIN, INPUT_PULLUP);
		pinMode(OPTION_2_PIN, INPUT_PULLUP);
		pinMode(OPTION_3_PIN, INPUT_PULLUP);
		pinMode(OPTION_4_PIN, INPUT_PULLUP);

		// Configure new output pins
		pinMode(TIMER_OUT_PIN, OUTPUT);

		int WhichConfigFile = CONFIG_FILE_PRIMARY;
        
		// If it's a new board with the option switches, display the setting
		// of the switches.
        
		if (isNewBoard())
		{
			Serial.println("This is a new SD Shield");

			Serial.println("Configuration switch settings:");
                
			Serial.print("Config file: ");
			Serial.println(debounceInputPin(OPTION_1_PIN) ? "Default" : "Alternate");
			if (!debounceInputPin(OPTION_1_PIN))
			{
				WhichConfigFile = CONFIG_FILE_ALTERNATE;
			}

			Serial.print("Option 2: ");
			Serial.println(debounceInputPin(OPTION_2_PIN) ? "Off" : "On");

			Serial.print("Option 3: ");
			Serial.println(debounceInputPin(OPTION_3_PIN) ? "Off" : "On");

			Serial.print("Option 4: ");
			Serial.println(debounceInputPin(OPTION_4_PIN) ? "Off" : "On");
		}
        
		link = new Link();
		link->begin();

		disks = new Disks();
		disks->mountDefaults(WhichConfigFile);
        
		Wire.begin();

		rtc = new RTC();

		// The timers are based on a fairly fast timer and everything is
		// derived from it.  This sets up the initial timer time-out.
        
		nextPoll = millis() + FAST_POLL_DELAY;
		pollCounter = 0;

		freeRam("Initialization complete");
	}
	else
	{
		Serial.begin(9600);
		while (!Serial) ;
	
#if defined(ARDUINO_RASPBERRY_PI_PICO)
		Wire.setSCL(21);
		Wire.setSDA(20);
#endif
		testerOn = true;
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
	
}



//=============================================================================
// Displays the amount of free memory to the serial port.  Useful for debugging
// but will be removed eventually.

#if defined(ARDUINO_AVR_MEGA2560)
int freeRam(const char *text)
{
        extern int __heap_start, *__brkval;
        int v;
        int freemem = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
        Serial.print(text);
        Serial.print(" - Free memory: ");
        Serial.println(freemem);
}
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
uint32_t getTotalHeap(void) {
	extern char __StackLimit, __bss_end__;
   
	return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeap(void) {
	struct mallinfo m = mallinfo();

	return getTotalHeap() - m.uordblks;
}

int freeRam(const char *text) 
{
	// The difference is (approximately) the free, available ram.
	int freemem = getFreeHeap();
	
	Serial.print(text);
	Serial.print(" - Free memory: ");
	Serial.println(freemem);
	
	return freemem;
}

#endif


//=============================================================================
// This is the main loop.  Loop gets called over and over by the Arduino
// framework, so it can return and get called again, or just have an infinite
// loop.

void loop()
{
	if (!testerOn)
	{
		// Poll for any incoming commands.  If there is something to process,
		// go ahead and handle it now.  There is no time delay here; check often
		// and process incoming messages ASAP.
        
		if (link->poll())
		{
			Event *ep = link->getEvent(); // this waits for an event
			bool deleteEvent = processEvent(ep);
                
			// If the event isn't needed, delete it.  Some events need
			// responses sent back and the event is re-used.
                
			if (deleteEvent)
			{
				link->freeAnEvent(ep);
			}
		}
        
		// See if it's time to poll the various subsystems.  This is a
		// slow poll, so put high speed polling before this logic.
        
		if (nextPoll <= millis())
		{
			nextPoll = millis() + FAST_POLL_DELAY;

			// See if there is an active real time timer running
			// and process it if so.
                
			if (timerValue)
			{
				if (++timerCount >= timerValue)
				{
					timerCount = 0; // reset counter
					//digitalWrite(TIMER_OUT_PIN, !debounceInputPin(TIMER_OUT_PIN));
					digitalWrite(TIMER_OUT_PIN, LOW);
					digitalWrite(TIMER_OUT_PIN, HIGH);
				}
			}
                
			// Now see if it's time to do the slow poll logic.  This should
			// always be 100 ms.  Again, there is no guarantee about jitter,
			// but this should be close enough for most cases.

			if (++pollCounter >= POLL_DELAY / FAST_POLL_DELAY)
			{
				// Poll the various subsystems
        
				uInt->poll(); // user interface
				disks->poll(); // disk subsystem

				pollCounter = 0;
			}
		}
	}
	else
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
}




//=============================================================================
// This processes an event that came from the host.  Returns a flag indicating
// if the event should be freed or not.  If the event is reused, returns false.
// Else returns true indicating the caller should dispose of the event.

static bool processEvent(Event *ep)
{
        bool deleteEvent = false;    // assume event should not be deleted
        
        // Now process the event based on the type.
                
        switch (ep->getType())
        {
                case EVT_GET_DIRECTORY:
#ifdef DEBUG_DIR
                        Serial.println("Got GET DIRECTORY");
#endif
                        link->freeAnEvent(ep);    // free it up so sendDirectory can use it
                        sendDirectory();
                        break;
                                
                case EVT_TYPE_FILE:
#ifdef DEBUG_FILE_READ
                        Serial.print("Got TYPE FILE: ");
#endif
                        Serial.println((char *)(ep->getData()));
                        openFileForRead(ep);
                        break;
                                
                case EVT_SEND_DATA:
#ifdef DEBUG_FILE_READ
                        Serial.println("Main loop got SEND_DATA");
#endif
                        // they want bytes from the open file
                        nextDataBlock(ep);
                        break;

                case EVT_GET_MOUNTED:
                        //Serial.println("Got request for mounted drives");
                        link->freeAnEvent(ep); // free it up for sendMounted to use
                        sendMounted();
                        break;
                        
                case EVT_MOUNT:
                {
                        byte *bptr = ep->getData(); // pointer to data
                        byte drive = *bptr++;       // drive number
                        byte readonly = *bptr++;    // read-only flag
                        if (disks->mount(drive, (char *)bptr, readonly))
                        {
                                ep->clean(EVT_ACK);
                        }
                        else
                        {
                                ep->clean(EVT_NAK);
                                ep->addByte(disks->getErrorCode());
                        }
                        link->sendEvent(ep);
                        break;
                }
                                
                case EVT_UNMOUNT:   // unmount a drive.  One more byte which is the drive number
                {
                        byte *bptr = ep->getData();
                        disks->unmount(*bptr);
                        ep->clean(EVT_ACK);
                        link->sendEvent(ep);
                        break;
                }
                                
                case EVT_READ_SECTOR:
                        readSector(ep);
                        break;

                case EVT_READ_SECTOR_LONG:
                        readSectorLong(ep);
                        break;
                        
                case EVT_WRITE_SECTOR:
                        writeSector(ep);
                        break;

                 case EVT_WRITE_SECTOR_LONG:
                        writeSectorLong(ep);
                        break;
                        
                case EVT_GET_STATUS:
                        getDriveStatus(ep);
                        break;
                        
                case EVT_GET_VERSION:
                {
                        ep->clean(EVT_VERSION_INFO);  // same event type but clear all other data
                        byte *ptr = ep->getData();
                        strcpy((char *)ptr, "Corsham Technology\r\nv1.4");
                        link->sendEvent(ep);
                        break;
                }

                case EVT_GET_CLOCK:
                        //Serial.println("Got a clock request");
                        ep->clean(EVT_CLOCK_DATA);
                        if (rtc->getClock(ep->getData()) == false)
                        {
                                // Failed to get the data, so send back an error.
                                ep->clean(EVT_NAK);
                                ep->addByte(ERR_DEVICE_NOT_PRESENT);
                        }
                        link->sendEvent(ep);
                        break;

                case EVT_SET_CLOCK:
                        //Serial.println("Got a set clock request");
                        if (rtc->setClock(ep->getData()))
                        {
                                ep->clean(EVT_ACK);
                        }
                        else
                        {
                                ep->clean(EVT_NAK);
                                ep->addByte(ERR_DEVICE_NOT_PRESENT);
                        }
                        link->sendEvent(ep);
                        break;

                case EVT_DONE:
                        // Close any open file.

                        closeFiles();
                        deleteEvent = true;
                        break;

                case EVT_WRITE_FILE:
                        openFileForWrite(ep);
                        break;

                case EVT_WRITE_BYTES:
                        writeBytes(ep);
                        break;

                case EVT_SAVE_CONFIG:
#ifdef DEBUG_SAVE_CONFIG
                        Serial.println("Got request to save configuration file");
#endif
                        if (disks->saveConfig())
                        {
                                ep->clean(EVT_ACK);
                        }
                        else
                        {
                                ep->clean(EVT_NAK);
                                ep->addByte(ERR_WRITE_ERROR);
                        }
                        link->sendEvent(ep);
                        break;

                case EVT_SET_TIMER:
                {
                        if (isNewBoard())
                        {
                                byte *bptr = ep->getData(); // pointer to data
                                byte interval = *bptr++;
                                setTimerValue(interval);
                                
                                ep->clean(EVT_ACK);   // new boards support timers
                        }
                        else
                        {
                                ep->clean(EVT_NAK);
                                ep->addByte(ERR_NOT_IMPLEMENTED);
                        }
                        link->sendEvent(ep);
                        break;
                }
                        
                default:
                        // All the unwanted toys end up here.  Maybe a garbage Event,
                        // maybe an old type, or one we haven't implemented yet.
                        //
                        // This recovery logic should be re-worked.  One solution might
                        // be to grab/discard bytes until the DIRECTION line changes
                        // state, then send back a NAK.
                        
                        Serial.print("processEvent: Got unknown event type: 0x");
                        Serial.println(ep->getType(), HEX);
                        deleteEvent = true;
                        break;
        }
        return deleteEvent;
}




//=============================================================================
// This handles a request to read a disk sector where there are tracks and
// sectors.  The data structure contains a standard header with five bytes of
// data: (1) Drive number, 0 to 3, (2) sector size (coded), (3) track number,
// zero based, (3) sector number, 0 based, and sectors per track (actual value,
// one based).

static void readSector(Event *ep)
{
        byte *bptr = ep->getData();  // start of arguments
        byte drive = *bptr++;
        byte sectorSize = *bptr++;    // note used for now
        unsigned long track = (unsigned long)(*bptr++);
        //Serial.println(*bptr);
        unsigned long sector = (unsigned long)(*bptr++);
        unsigned long sectorsPerTrack = (unsigned long)(*bptr++);
        
        // NOTE: Need to add checks for valid drive, track, sector, etc
        
        // Compute the offset.  Very simple offset calculation.
        
        unsigned long offset = ((track * sectorsPerTrack) + sector) * SECTOR_SIZE;        
        
        // Now prepare the event for sending back the data.  Same event type,
        // but get rid of the old data and replace with exactly SECTOR_SIZE
        // number of bytes.

#ifdef DEBUG_SECTOR_READ
        Serial.print("readSector drive ");
        Serial.print(drive);
        Serial.print(", track 0x");
        Serial.print(track, HEX);
        Serial.print(", sector 0x");
        Serial.print(sector, HEX);
        Serial.print(", sec/track 0x");
        Serial.print(sectorsPerTrack, HEX);
        Serial.print(", offset 0x");
        Serial.print(offset >> 16, HEX);
        Serial.print(offset & 0xffff, HEX);
        Serial.print(" first two: ");
#endif  // DEBUG_SECTOR_READ

        byte *ptr = ep->getData();
        *ptr++ = 2;      // sector size 256 bytes
        ep->clean(EVT_READ_SECTOR);  // same event type but clear all other data
        if (disks->read(drive, offset, ptr) == false)
        {
                ep->clean(EVT_NAK);  // send error status
                ep->addByte(disks->getErrorCode());
        }
#ifdef DEBUG_SECTOR_READ
        else
        {
                Serial.print(ptr[0], HEX);
                Serial.print(" ");
                Serial.println(ptr[1], HEX);
        }
#endif

        // send back something
        link->sendEvent(ep);
}




//=============================================================================
// This handles a request to read a disk sector where only a large sector
// number is provided.  The data structure contains a standard header with six
// bytes of data: (1) Drive number, 0 to X, (2) sector size (coded), then a
// four byte sector number with the MSB first.

static void readSectorLong(Event *ep)
{
        byte *bptr = ep->getData();  // start of arguments
        byte drive = *bptr++;
        byte sectorSize = *bptr++;    // note used for now
        unsigned long sector = (unsigned long)(*bptr++);
        sector <<= 8;
        sector |= (unsigned long)(*bptr++);
        sector <<= 8;
        sector |= (unsigned long)(*bptr++);
        sector <<= 8;
        sector |= (unsigned long)(*bptr++);
        
        // NOTE: Need to add checks for valid drive, sector, etc
        
        // Compute the offset.  Very simple offset calculation.
        
        unsigned long offset = sector * SECTOR_SIZE;        
        
        // Now prepare the event for sending back the data.  Same event type,
        // but get rid of the old data and replace with exactly SECTOR_SIZE
        // number of bytes.

#ifdef DEBUG_SECTOR_READ
        Serial.print("readSectorLong drive ");
        Serial.print(drive);
        Serial.print(", sector 0x");
        Serial.print(sector, HEX);
        Serial.print(", offset 0x");
        //Serial.print(offset >> 16, HEX);
        Serial.print(offset, HEX);
        Serial.print(" first two: ");
#endif  // DEBUG_SECTOR_READ

        byte *ptr = ep->getData();
        *ptr++ = 2;      // sector size 256 bytes
        ep->clean(EVT_READ_SECTOR);  // same event type but clear all other data
        if (disks->read(drive, offset, ptr) == false)
        {
                ep->clean(EVT_NAK);  // send error status
                ep->addByte(disks->getErrorCode());
        }
#ifdef DEBUG_SECTOR_READ
        else
        {
                Serial.print(ptr[0], HEX);
                Serial.print(" ");
                Serial.println(ptr[1], HEX);
        }
#endif

        // send back something
        link->sendEvent(ep);
}




//=============================================================================
// This handles a request to write a disk sector using only a long sector
// number.  The data structure contains a standard header with six bytes of 
// data: (1) Drive number, 0 to 3, (2) sector size (coded), bytes 3 to 6
// contain the long sector number, MSB first, zero based.
//
// That is followed by one sector's worth of data to be written.

static void writeSectorLong(Event *ep)
{
        // Pull off the arguments from the start of the message
        
        byte *bptr = ep->getData();  // start of arguments
        byte drive = *bptr++;
        byte sectorSize = *bptr++;    // note used for now
        unsigned long sector = (unsigned long)(*bptr++);
        sector <<= 8;
        sector |= (unsigned long)(*bptr++);
        sector <<= 8;
        sector |= (unsigned long)(*bptr++);
        sector <<= 8;
        sector |= (unsigned long)(*bptr++);
        
        // NOTE: Need to add checks for valid drive, sector, etc
        
        // Compute the offset.  Very simple offset calculation.
        
        unsigned long offset = sector * SECTOR_SIZE;        
        
        // Dump the data

#ifdef LOG_WRITE
        Serial.print("writeSectorLong drive ");
        Serial.print(drive);
        Serial.print(", offset 0x");
        Serial.print(offset >> 16, HEX);
        Serial.println(offset & 0xffff, HEX);
#endif  // LOG_WRITE

        if (disks->write(drive, offset, bptr))
        {
                ep->clean(EVT_ACK);
        }
        else
        {
                ep->clean(EVT_NAK);  // send error status
                ep->addByte(disks->getErrorCode());
                Serial.print("got write error: ");
                Serial.println(disks->getErrorCode());
        }
        
        // no error handling at all!
        link->sendEvent(ep);
}




//=============================================================================
// This handles a request to write a disk sector using tracks and sectors.  The
// data structure contains a standard header with five bytes of data: (1) Drive
// number, 0 to 3, (2) sector size (coded), (3) track number, zero based, (3)
// sector number, 0 based, and sectors per track (actual value, one based).
//
// That is followed by one sector's worth of data to be written.

static void writeSector(Event *ep)
{
        // Pull off the arguments from the start of the message
        
        byte *bptr = ep->getData();  // start of arguments
        byte drive = *bptr++;
        byte sectorSize = *bptr++;    // note used for now
        unsigned long track = (unsigned long)(*bptr++);
        unsigned long sector = (unsigned long)(*bptr++);
        unsigned long sectorsPerTrack = (unsigned long)(*bptr++);

        // bptr is now pointing to the start of the user data to be written.
        
        // NOTE: Need to add checks for valid drive, sector, etc
        
        // Compute the offset.  Very simple offset calculation.
        
        unsigned long offset = ((track * sectorsPerTrack) + sector) * SECTOR_SIZE;
        
        // Dump the data

#ifdef LOG_WRITE
        Serial.print("writeSector drive ");
        Serial.print(drive);
        Serial.print(", track ");
        Serial.print(track);
        Serial.print(", sector ");
        Serial.print(sector);
        Serial.print(", sec/track ");
        Serial.print(sectorsPerTrack);
        Serial.print(", offset 0x");
        Serial.print(offset >> 16, HEX);
        Serial.println(offset & 0xffff, HEX);
#endif  // LOG_WRITE

        if (disks->write(drive, offset, bptr))
        {
                ep->clean(EVT_ACK);
        }
        else
        {
                ep->clean(EVT_NAK);  // send error status
                ep->addByte(disks->getErrorCode());
                Serial.print("got write error: ");
                Serial.println(disks->getErrorCode());
        }
        
        // no error handling at all!
        link->sendEvent(ep);
}




//=============================================================================
// This gets a disk drive's status, such as whether it's available or not,
// read-only, etc.

static void getDriveStatus(Event *ep)
{
        byte *bptr = ep->getData();  // start of arguments
        byte drive = *bptr++;
        byte result = disks->getStatus(drive);
        
        ep->clean(EVT_DISK_STATUS);
        ep->addByte(result);
        link->sendEvent(ep);
}



//=============================================================================
// Send a list of all mounted drives

void sendMounted(void)
{
        Event *eptr;
        char *cptr;
        
        for (int i = 0; i < MAX_DISKS; i++)
        {
                eptr = link->getAnEvent();
                eptr->clean(EVT_MOUNTED);

                // add the info about this mounted drive
                eptr->addByte((byte)i);   // drive number

                // if the drive is mounted, then fill in all the details.

                if (disks->isMounted(i))
                {
                        eptr->addByte(disks->isReadOnly(i));

                        cptr = disks->getFilename(i);
                        while (*cptr)
                        {
                                eptr->addByte(*cptr++);
                        }
                        eptr->addByte(0);
                }
                else  // not mounted so indicate so
                {
                        eptr->addByte(0);
                        eptr->addByte(0);
                }
                        
                link->sendEvent(eptr);
        }

        // Indicate all the drives were sent
        
        eptr = link->getAnEvent();
        eptr->clean(EVT_DIR_END);
        link->sendEvent(eptr);
}




//=============================================================================
// This does a simple hex dump to the serial port.  On entry, this is given a
// pointer to the data and the number of bytes to dump.  Very simplistic
// function that has no features.

void hexdump(unsigned char *ptr, unsigned int size)
{
        unsigned int offset = 0;
        
        while (size)
        {
                Serial.print(*ptr++, HEX);
                Serial.print(" ");
                size--;
        }
        Serial.println("");
}



//=============================================================================
// Given a standard disk sector size code, convert to the actual number and
// return it.  If no valid value is provided, this always returns 256.

unsigned int getSectorSize(byte code)
{
        unsigned int size = 256;    // default size
        
        switch (code)
        {
                case 1:
                        size = 128;
                        break;
                      
                case 2:
                        size = 256;
                        break;
                      
                case 3:
                        size = 512;
                        break;
                      
                case 4:
                        size = 1024;
                        break;

                default:
                        Serial.print("Got bad sector size value: ");
                        Serial.println(code);
        }
        
        return size;
}



//=============================================================================
// This returns non-zero for the newer shield, zero for the older version.

bool isNewBoard(void)
{
        // This pin is pulled high for the older boards but is pulled low
        // for new boards.
#if defined(ARDUINO_AVR_MEGA2560)
        return !debounceInputPin(NEW_SHIELD_PIN);
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
	    return true;
#endif
}



//=============================================================================
// Sets a timer.  On entry, the interval specifies the time:
//
// 0 = disable timer
// 1 = 10 ms
// 2 = 20 ms
// 3 = 30 ms
// 4 = 40 ms
// 5 = 50 ms
// 6 = 100 ms
// 7 = 250 ms
// 8 = 500 ms
// 9 = 1 second
//
// Any other value will be ignored.
//
// This sets the timerValue and also resets the timerCount value.  Before
// calling, the user should verify this shield supports the timer.

void setTimerValue(unsigned char interval)
{
        unsigned int values[] = { 0, 1, 2, 3, 4, 5, 10, 25, 50, 100 };

        if (interval < (sizeof(values) / sizeof(unsigned int)))
        {
                timerValue = values[interval];

#ifdef DEBUG_SET_TIMER
                Serial.print("setTimerValue(");
                Serial.print(interval);
                Serial.print(") set timerValue to ");
                Serial.println(timerValue);
#endif
        }
#ifdef DEBUG_SET_TIMER
        else
        {
                Serial.print("setTimerValue got illegal value: ");
                Serial.println(interval);
        }
#endif
}




//=============================================================================
// Given an input pin number, read and debounce it.  Returns the final
// debounced value.  It must be the same value for DEBOUNCE_COUNT times.

bool debounceInputPin(int pin)
{
        bool val, last;     // it's okay not to initialize them
        int goodCount = 0;

        do
        {
                if ((last = digitalRead(pin)) == val)
                {
                        goodCount++;
                }
                else
                {
                        val = last;     // new value
                        goodCount = 0;  // start counting again
                }
        }
        while (goodCount < DEBOUNCE_COUNT);

        return val;
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
