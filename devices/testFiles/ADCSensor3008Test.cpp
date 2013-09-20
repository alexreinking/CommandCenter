#include "ADCSensor3008.h"
#include "TemperatureSensor.h"
#include <stdio.h>

int main(int argc, char** argv)
{
    ADCSensor3008* adc1 = new ADCSensor3008(0);
     
ADCSensor3008 temperatureAdc(7); //Untested
TemperatureSensor temperatureSensor(&temperatureAdc);
   
    for (int i = 0; i < 10; i++)
    {
        int32_t value = adc1->readConversion();
        int32_t temperature = temperatureSensor.readTemperature();
        printf("Value read: %d\n", value);
        printf("Temperature read: %d\n",temperature);
    }
}
