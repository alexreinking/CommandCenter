#include "TemperatureSensor.h"

TemperatureSensor::TemperatureSensor(ADCSensor3008* adc)
{
    this->adc = adc;
}

int32_t TemperatureSensor::getTemperature() const
{
    return lastTemperature;
}

int32_t TemperatureSensor::readTemperature()
{
    int32_t adcValue = adc->readConversion();
    // We will assume 2.98V at 298K, adc is 10-bit
    // T = V * T0 / V0
    // V0 = 2.98 volts * 1023 max value / 5 volts max value
    // return value = T - 24850 so that the result is in celsius, not kelvin
    int32_t voltage = 5000*(adcValue/1023); //mV
    const int32_t v0 = 5000*(610/1023); //mV. A value of 610 maps to 2.98V
    lastTemperature = (voltage * 298.0/v0) - 273.15;
    return lastTemperature;
}
