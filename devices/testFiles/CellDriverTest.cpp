//Cell Driver Tester
//Program for making sure the cell driver works properly

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <iostream>
#include <termios.h>
#include <time.h>
#include <string>
#include <stdio.h>
#include <math.h>
#include "Uart.h"
#include "CellDriver.h"

Uart cellUart(1, 115200);
CellDriver cellDriver(&cellUart);  //Most definitly not good

uint64_t startTimer;
uint64_t permanentStartTimer;
uint64_t towerTimer;

uint64_t millis()
{
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return currentTime.tv_sec * 1000 + currentTime.tv_usec / 1000;
}


void setup()
{

printf("Running Setup\n\r");
permanentStartTimer = millis();
startTimer = millis();
towerTimer = millis();
}

void loop()
{  
        bool status = false;
        //run the cell driver function
        if (cellDriver.update())
        {
            TextMessage textMessage = cellDriver.getTextMessage();
            const char* messageData = textMessage.messageData.c_str();
            int length = textMessage.messageData.length();

            printf("\n\r");
            for (int i = 0; i < length; i++)
            {

                //This is where the parsing will take place, but it has to be formatted for the appropriate application
                printf("%c",messageData[i]);
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

        //Periodically Send update messages (every 2 minutes)
        if (startTimer + 120000 < millis())
        {
            printf("Sending a Text Message\n\r");

        int hours = floor((millis() - permanentStartTimer)/3600000);
        int minutes = floor(((millis() - permanentStartTimer)/1000 - hours*3600)/60);
        int seconds = (millis() - permanentStartTimer)/1000 - hours*3600 - minutes*60;
        
            std::stringstream messageTextString;
            messageTextString << "Balloon says hello at " << hours << ":" << minutes << ":" << seconds << " from boot.";   
            std::string messageAsString;

            messageAsString = messageTextString.str();
            cellDriver.queueTextMessage("12537408798",messageAsString.c_str());
            startTimer = millis();

        }

                //Periodically Send Location Requests (every 30 seconds)
        if (towerTimer + 30000 < millis())
        {
            printf("Asking for towers\n\r");
            std::string lastTowerInfo;
            lastTowerInfo = cellDriver.getLastTowerInformation();

            std::cout << lastTowerInfo << std::endl;
            towerTimer = millis();
        }        
        
        

}


int main(int argc, char* argv[])
{

    setup();

    // Perform our main loop FOREVER!
    while (1)
    {
    loop();
    }

}
