#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
/*
Migrating to the Beaglebone Black.  This is for the purpose of making the
project compatible with future development of the platform and to remove
some of the bugs with the white.  This currently removes the functionality
of the PWM input.  This will remain disabled until a suitible solution can
be found, which will likely have to happen by leveraging the on board PRUs
*/

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <iostream>
#include <termios.h>
#include <time.h>
#include <string>

#include "devices/Gpio.h"
#include "devices/Uart.h"
#include "devices/GPSDecoder.h"
#include "devices/IMUDecoder.h"
#include "devices/ADCSensor3008.h"
#include "devices/CellDriver.h"
//#include "devices/PWMSensor.h"  //<<This guy has to be gone for now
#include "devices/ServoDriver.h"
//#include "devices/HumiditySensor.h" //<<This guy doesn't actually exist?
#include "devices/TemperatureSensor.h"
//#include "devices/GpioOutput.h" //<<This needs to be fitted with the new gpio library

#include "akp/cAkpParser/crc8.h"
#include "akp/cAkpParser/cAkpParser.h"

#include "CommandCenterPinConfig.h"
#include "devices/muxConfig.h"

//These really should be fixed...
#define STAY_ALIVE_PIN DIG_I_O_1
#define CELL_MAX_TAGS 6

//Git will see this

/*
Pin defines based on the CPB.  These however create a problem for the BBB
due to boot problems.  If some of the pins on the BBB are grounded during boot,
boot will fail. The BBB YUAA documentation notes these pins.
*/
/*
#define STATUS_LED_2_PIN 33
#define STATUS_LED_3_PIN 37
#define STATUS_LED_4_PIN 63
#define STATUS_LED_5_PIN 46
#define STATUS_LED_6_PIN 44
#define STATUS_LED_7_PIN 34
#define STATUS_LED_8_PIN 35
#define STATUS_LED_9_PIN 39

#define PWM_INPUT_1_PIN 26
#define PWM_INPUT_2_PIN 27
#define PWM_INPUT_3_PIN 38
#define PWM_INPUT_4_PIN 61
#define PWM_INPUT_5_PIN 65
#define PWM_INPUT_6_PIN 86
#define PWM_INPUT_7_PIN 117
#define PWM_INPUT_8_PIN 115
*/

/*
True pin defines are located in a separte file: CommandCenterPinConfig.h

#define LED_7 P8_9
#define LED_8 P8_10
#define LED_5 P8_11
#define LED_6 P8_12
#define LED_4 P8_13
#define LED_3 P8_14
#define LED_1 P8_15
#define LED_2 P8_16

*/




struct termios oldTerminalSettings;

//These should be ok.  The mux configuration is taken care of by the uart handle
Uart transceiverUart(1, 9600);
Uart imuUart(2, 115200);
Uart servoDriverUart(3, 9600);
Uart cellUart(4, 115200);
Uart gpsUart(5, 9600);


IMUDecoder imuDecoder;  //Good
GPSDecoder gpsDecoder;  //Good
CellDriver cellDriver(&cellUart);  //Good
ADCSensor3008 temperatureAdc(7); //Good
TemperatureSensor temperatureSensor(&temperatureAdc); //Good

// Hi, I'm adding a comment!
//PWMSensor throttleIn(43);  //Functionality gone due to unreliability

//Update to code would be better for the servo controller
ServoDriver servoController(&servoDriverUart);


//Keep track of the last of these critical values
int32_t lastLatitude;
int32_t lastLongitude;
int32_t lastAltitude;
int32_t lastSatelliteCount;

int32_t lastCellMmc;
int32_t lastCellMnc;
int32_t lastCellLac;
int32_t lastCellCid;

//Killswitch timeout, initialized to 10 minutes at startup
long secondsToTimeout = 600;
bool hasKickedBucket = false;

//Tags to be forwarded from cell-shield
//Tag data can be 9 chars long, tags are always 2 chars.
char cellStoredTags[CELL_MAX_TAGS][3];
char cellStoredData[CELL_MAX_TAGS][10];
int cellStoredTagOn = 0;

//Uncomment the following line for IMU data logging to a .txt file
//#define IMULogData
#ifdef IMULogData
bool IMUFileOpened;
#endif

//Keep track of time, second by second
uint64_t secondStartTime;
uint64_t startTimer;
uint64_t permanentStartTimer;
uint64_t towerTimer;

// What to current echo/output to the console - with flags!
// 1 = console
// 2 = transceiverUart
// 4 = GPS
// 8 = IMU
// 16 = CELL_SHIELD
// 32 = tag data
// Default to everything!
// The characters that manipulate this from the console are
// ) to turn everything off
// ! for console
// @ for transceiver
// # for gps
// $ for imu
// % for cell shield
// ^ for tag data
// These correspond to the characters that are Shift+(0-5)
int debugEchoMode = 32;

uint64_t millis()
{
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return currentTime.tv_sec * 1000 + currentTime.tv_usec / 1000;
}

//Recently added, untested...
uint64_t micros()
{
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return currentTime.tv_sec * 1000000 + currentTime.tv_usec;
}

void setup()
{
    //Configure stay-alive pin, start low
    //Preferably, this pin will have a resistor
    //between it and the killswitch circuit


    //Start keeping track of time
    secondStartTime = millis();
    startTimer = millis();
    permanentStartTimer = millis();
    towerTimer = millis();

    muxConfig("BB-CommandCenter");
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    pinMode(LED_4, OUTPUT);
    pinMode(LED_5, OUTPUT);
    pinMode(LED_6, OUTPUT);
    pinMode(LED_7, OUTPUT);
    pinMode(LED_8, OUTPUT);

    pinMode(STAY_ALIVE_PIN, OUTPUT);
}

//Handles general-from-anywhere things
void baseHandleTag(const char *tag, const char *data)
{
    if (strcmp(tag, "KL") == 0)
    {
        //Immediate kill
        hasKickedBucket = true;
    }
    else if (strcmp(tag, "ST") == 0)
    {
        //Try to parse time value
        char *endPtr;
        long seconds = strtol(data, &endPtr, 10);
        //We have parsed the time value correcly if
        //endPtr points to the null-terminator of the string.
        if (*endPtr == '\0')
        {
            secondsToTimeout = seconds;
        }
    }
}

// Only does it if there is space...
void forwardTag(const char *tag, const char *data)
{
    //Set to be forwarded if there is space
    if (cellStoredTagOn < CELL_MAX_TAGS)
    {
        strncpy(cellStoredTags[cellStoredTagOn], tag, sizeof(*cellStoredTags));
        strncpy(cellStoredData[cellStoredTagOn], data, sizeof(*cellStoredData));
        cellStoredTagOn++;
    }
    else
    {
        std::cout << "Could not forward tag: " << tag << " with data: " << data << " due to lack of space.\n";
    }
}

//Handles tags from cell shield
void cellShieldHandleTag(const char *tag, const char *data)
{
    //Specifically, we would like to forward all non-base tags
    //Keep special track of these two...
    //Forward everything else...
    if (strcmp(tag, "KL") != 0 &&
            strcmp(tag, "ST") != 0)
    {
        forwardTag(tag, data);
    }
    else
    {
        //Handle base tags
        baseHandleTag(tag, data);
    }
}

char getHexOfNibble(char c)
{
    c = c & 0x0f;
    if (c < 10)
    {
        return '0' + c;
    }
    else
    {
        return 'a' + c - 10;
    }
}

void sendTag(const char *tag, const char *data, Uart &uart)
{
    if (data && *data)
    {
        unsigned char checksum = crc8(tag, 0);
        checksum = crc8(data, checksum);
        //Get hex of checksum
        char hex1 = getHexOfNibble(checksum >> 4);
        char hex2 = getHexOfNibble(checksum);

        uart << tag << '^' << data << ':' << hex1 << hex2;
    }
}

void sendTag(const char *tag, const char *data, std::ostream &stream)
{
    if (data && *data)
    {
        unsigned char checksum = crc8(tag, 0);
        checksum = crc8(data, checksum);
        //Get hex of checksum
        char hex1 = getHexOfNibble(checksum >> 4);
        char hex2 = getHexOfNibble(checksum);

        stream << tag << '^' << data << ':' << hex1 << hex2;
    }
}

template<class T>
void sendTag(const char *tag, T data, std::ostream &stream)
{
    std::stringstream convertOutput;
    convertOutput << data;
    sendTag(tag, convertOutput.str().c_str(), stream);
}

template<class T>
void sendTag(const char *tag, T data, Uart &uart)
{
    std::stringstream convertOutput;
    convertOutput << data;
    sendTag(tag, convertOutput.str().c_str(), uart);
}

//Sends tag with data to the transceiver and possible std::cout for debugging
//Avoids sending tags with NULL or empty data (for convenience)
template<class T>
void mainSendTag(const char *tag, T data)
{
    sendTag(tag, data, transceiverUart);
    if (debugEchoMode & 32)
    {
        sendTag(tag, data, std::cout);
    }
}

void cellShieldSendInformation()
{
    std::stringstream completeText;

    static int sendCounter = 0;

    // Send every several minutes...
    sendCounter++;
    if (sendCounter >= 600)
    {
        sendCounter = 0;

        sendTag("LA", lastLatitude, completeText);
        sendTag("LO", lastLongitude, completeText);
        sendTag("GS", lastSatelliteCount, completeText);
        sendTag("DT", secondsToTimeout, completeText);
        sendTag("LV", hasKickedBucket ? "0" : "1", completeText);

        //This is Jason's phone number.  Please change it to your own when testing!
        cellDriver.queueTextMessage("12537408798", completeText.str().c_str());
    }


    //Periodically Send update messages (every 5 minutes)
    if (startTimer + 600000 < millis())
    {
        printf("Sending a Text Message\n\r");
        /*This needs to be changed for whatever information should be sent via text*/
        int hours = floor((millis() - permanentStartTimer) / 3600000);
        int minutes = floor(((millis() - permanentStartTimer) / 1000 - hours * 3600) / 60);
        int seconds = (millis() - permanentStartTimer) / 1000 - hours * 3600 - minutes * 60;

        std::stringstream messageTextString;
        messageTextString << "Balloon says hello at " << hours << ":" << minutes << ":" << seconds << " from boot.";
        std::string messageAsString;

        messageAsString = messageTextString.str();
        //This is Jason's phone number.  Please change it to your own when testing!
        cellDriver.queueTextMessage("12537408798", messageAsString.c_str());
        startTimer = millis();

    }

    //Periodically Send Location Requests (every 120 seconds)
    if (towerTimer + 120000 < millis())
    {
        printf("Asking for towers\n\r");
        if (cellDriver.newTowerInfo)
        {
            std::string lastTowerInfo;
            lastTowerInfo = cellDriver.getLastTowerInformation();

            std::cout << lastTowerInfo << std::endl;
            mainSendTag("CT", lastTowerInfo); //untested
            towerTimer = millis();
            cellDriver.newTowerInfo = false;
        }

    }

}

void loop()
{
    //    char insideTemperature[10];
    //    char outsideTemperature[10];
    //    bool gottenInsideTemp = false;
    //    bool gottenOutsideTemp = false;

    //    int32_t temperature = temperatureSensor.readTemperature();

    //Keep track of what new data we have gotten
    bool gottenGps = false;
    bool gottenImu = false;

    //We will loop around getting data from all sources until a second has passed.
    while (secondStartTime + 1000 > millis())
    {
        //Check for data from all sources...
        static TagParseData debuggingData;
        int c = -1;
        while ((c = getchar()) != -1)
        {
            // See the comments of debugEchoMode for details...
            switch (c)
            {
            case ')':
                debugEchoMode = 0;
                break;
            case '!':
                debugEchoMode ^= 1;
                break;
            case '@':
                debugEchoMode ^= 2;
                break;
            case '#':
                debugEchoMode ^= 4;
                break;
            case '$':
                debugEchoMode ^= 8;
                break;
            case '%':
                debugEchoMode ^= 16;
                cellDriver.shouldEchoUartToStdout = (debugEchoMode & 16);
                break;
            case '^':
                debugEchoMode ^= 32;
                break;
            }
            if (debugEchoMode & 1)
            {
                std::cout << (char)c;
            }
            if (parseTag(c, &debuggingData))
            {
                baseHandleTag(debuggingData.tag, debuggingData.data);
            }
        }

        static TagParseData transceiverData;
        c = -1;
        while ((c = transceiverUart.readByte()) != -1)
        {
            if (debugEchoMode & 2)
            {
                std::cout << (char)c;
            }

            if (parseTag(c, &transceiverData))
            {
                // Handle the tag we just marvelously got!
                baseHandleTag(transceiverData.tag, transceiverData.data);
            }
        }

        c = -1;
        while ((c = gpsUart.readByte()) != -1)
        {
            if (debugEchoMode & 4)
            {
                std::cout << (char)c;
            }
            if (gpsDecoder.decodeByte(c))
            {
                gottenGps = true;
            }
        }

        c = -1;
        while ((c = imuUart.readByte()) != -1)
        {
            if (debugEchoMode & 8)
            {
                std::cout << (char)c;
            }
            if (imuDecoder.decodeByte(c))
            {
                gottenImu = true;
            }
        }

        static TagParseData cellData;
        if (cellDriver.update())
        {
            TextMessage textMessage = cellDriver.getTextMessage();
            const char *messageData = textMessage.messageData.c_str();
            int length = textMessage.messageData.length();

            printf("\n\r");
            for (int i = 0; i < length; i++)
            {
                //This is where the parsing will take place, but it has to be formatted for the appropriate application
                printf("%c", messageData[i]);
                /*
                if (parseTag(messageData[i], &cellData))
                {
                    cellShieldHandleTag(cellData.tag, cellData.data);
                }
                */
            }
            printf("\n\r");
            // Remove it from the module. it is a gonner now.
            cellDriver.deleteMessage(textMessage);
        }

        // Give up a little time to the system...
        struct timespec sleepTime = {0, 1000};
        nanosleep(&sleepTime, NULL);
    }

    /*
    At some point you have to do something intelligent with all this data.
    Most likely, you don't want to be doing calculations every time throug the
    loop.  That would be ridiculous and wasteful. It would be better to do
    it on some set interval, such as 200 ms. That is up to the designers of the program
    and the control system.
    */

    //This part of the code is only executed every second!!!!
    //Notice the start time of this next second
    secondStartTime += 1000;

    //Actions for the living
    if (!hasKickedBucket)
    {
        //Update and check timeout
        if (--secondsToTimeout <= 0)
        {
            //Time to die!
            hasKickedBucket = true;
        }
        static int secondsSinceToggle = 0;
        static bool stayAliveUp = false;
        //To indicate that the arduino is running correctly,
        //we send out a 5-second high, 5-second low pulse
        if (++secondsSinceToggle >= 5)
        {
            //Do a toggle!
            secondsSinceToggle = 0;
            stayAliveUp = !stayAliveUp;
            digitalWrite(STAY_ALIVE_PIN, stayAliveUp);
        }
    }


    //Send out data -- ALL the data!
    int32_t temperatureInside = temperatureSensor.readTemperature();
    printf("Temperature: %f\n\r", (float) temperatureInside / 10.0);
    mainSendTag("TI", temperatureInside); //untested




    if (gottenGps)
    {
        //mainSendTag("TM", gpsDecoder.getTime());
        //mainSendTag("HD", gpsDecoder.getHDOP());

        mainSendTag("GS", gpsDecoder.getSatelliteCount());
        mainSendTag("LO", gpsDecoder.getLongitude());
        mainSendTag("LA", gpsDecoder.getLatitude());
        mainSendTag("AL", gpsDecoder.getAltitude());

        //mainSendTag("SP", gpsDecoder.getSpeed());
        //mainSendTag("TH", gpsDecoder.getTrueHeading());
        //mainSendTag("MH", gpsDecoder.getMagneticHeading());

        lastLongitude = gpsDecoder.getLongitude();
        lastLatitude = gpsDecoder.getLatitude();
        lastSatelliteCount = gpsDecoder.getSatelliteCount();
        lastAltitude = gpsDecoder.getAltitude();
    }
    else
    {
        //Send the last found ones if we have nothing new
        mainSendTag("LO", lastLongitude);
        mainSendTag("LA", lastLatitude);
        mainSendTag("AL", lastAltitude);
    }
    if (gottenImu)
    {
        mainSendTag("YA", imuDecoder.getYaw());
        mainSendTag("PI", imuDecoder.getPitch());
        mainSendTag("RO", imuDecoder.getRoll());
        mainSendTag("AX", imuDecoder.getAcceleration().coordX);
        mainSendTag("AY", imuDecoder.getAcceleration().coordY);
        mainSendTag("AZ", imuDecoder.getAcceleration().coordZ);

#ifdef IMULogData

        if (!IMUFileOpened)
        {
            imuoutput = fopen ("imuData4.txt", "w");
            IMUFileOpened = true;
        }

        fprintf(imuoutput, "%f %f %f ", imuDecoder.getYaw(), imuDecoder.getPitch(), imuDecoder.getRoll());
        fprintf(imuoutput, "%f %f %f ", imuDecoder.getAcceleration().coordX, imuDecoder.getAcceleration().coordY, imuDecoder.getAcceleration().coordZ);
        fprintf(imuoutput, "%f %f %f ", imuDecoder.getSpaceAcceleration().coordX, imuDecoder.getSpaceAcceleration().coordY, imuDecoder.getSpaceAcceleration().coordZ);
        fprintf(imuoutput, "%f %f %f ", imuDecoder.getIntegratedVelocity().coordX, imuDecoder.getIntegratedVelocity().coordY, imuDecoder.getIntegratedVelocity().coordZ);
        fprintf(imuoutput, "%f %f %f ", imuDecoder.getSpaceIntegratedVelocity().coordX, imuDecoder.getSpaceIntegratedVelocity().coordY, imuDecoder.getSpaceIntegratedVelocity().coordZ);
        fprintf(imuoutput, "%f %f %f ", imuDecoder.getIntegratedPosition().coordX, imuDecoder.getIntegratedPosition().coordY, imuDecoder.getIntegratedPosition().coordZ);
        fprintf(imuoutput, "%f %f %f ", imuDecoder.getSpaceIntegratedPosition().coordX, imuDecoder.getSpaceIntegratedPosition().coordY, imuDecoder.getSpaceIntegratedPosition().coordZ);
        fprintf(imuoutput, "%f \n", imuDecoder.getAcceleration().timestamp);
#endif




    }

    /*
        mainSendTag("MC", lastCellMmc);
        mainSendTag("MN", lastCellMnc);
        mainSendTag("LC", lastCellLac);
        mainSendTag("CD", lastCellCid);
    */

    //Send extra tags passed from the cellular connection
    /*
        while (cellStoredTagOn > 0)
        {
            cellStoredTagOn--;
            mainSendTag(cellStoredTags[cellStoredTagOn], cellStoredData[cellStoredTagOn]);
        }
    */

    //Life left...
    mainSendTag("DT", secondsToTimeout);

    //Liveliness!
    mainSendTag("LV", hasKickedBucket ? "0" : "1");

    // Meant for deliminating lines of tags...
    if (debugEchoMode & 32)
    {
        std::cout << "\n";
    }

    // Information sent to the cell shield arduino must be done separately to avoid overworking him.
    cellShieldSendInformation();




    // Switch a servo back and forth as a demonstration
    static int servoToggle = 0;
    if (servoToggle <= 0)
    {
        servoController.setSpeed(7, 126);
        servoController.setAngle(7, 3500);
        servoToggle = 1;

        digitalWrite(LED_1, LOW);
        digitalWrite(LED_2, LOW);
        digitalWrite(LED_3, LOW);
        digitalWrite(LED_4, LOW);
        digitalWrite(LED_5, LOW);
        digitalWrite(LED_6, LOW);
        digitalWrite(LED_7, LOW);
        digitalWrite(LED_8, LOW);

    }
    else
    {
        servoController.setSpeed(7, 50);
        servoController.setAngle(7, 1500);
        servoToggle = 0;

        digitalWrite(LED_1, HIGH);
        digitalWrite(LED_2, HIGH);
        digitalWrite(LED_3, HIGH);
        digitalWrite(LED_4, HIGH);
        digitalWrite(LED_5, HIGH);
        digitalWrite(LED_6, HIGH);
        digitalWrite(LED_7, HIGH);
        digitalWrite(LED_8, HIGH);

    }
}

void restoreTerminal()
{
    tcsetattr(0, TCSANOW, &oldTerminalSettings);
}

int main(int argc, char *argv[])
{
    //Unbuffered output, so the file can be read in as streamed.
    setvbuf(stdout, NULL, _IONBF, 0);

    //Don't wait for newline to get stdin input
    struct termios terminalSettings;
    if (tcgetattr(0, &terminalSettings) < 0)
    {
        perror("Error getting terminal settings");
    }

    // Save old terminal settings
    oldTerminalSettings = terminalSettings;

    // disable canonical mode processing in the line discipline driver
    // So everything is read in instantly from stdin!
    // Also, don't echo back characters... so we only see what we receive!
    terminalSettings.c_lflag &= ~(ICANON | ECHO);
    // We do not want to block with getchar
    // We have no minimum break in receiving characters (VTIME = 0)
    // and we have no minimum number of characters to receive (VMIN = 0)
    terminalSettings.c_cc[VTIME] = 0;
    terminalSettings.c_cc[VMIN] = 0;

    if (tcsetattr(0, TCSANOW, &terminalSettings) < 0)
    {
        perror("Error setting terminal settings");
    }

    atexit(restoreTerminal);

    // do initialization
    setup();

    // Perform our main loop FOREVER!
    while (1)
    {
        loop();
    }
}
