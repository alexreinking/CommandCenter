#include "Uart.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
//#include "muxConfig.h"

int Uart::configMux(const char *setting1)
{
    int muxFile = open("/sys/devices/bone_capemgr.8/slots", O_WRONLY);
    if (muxFile == -1)
    {
        return -1;
    }
    write(muxFile, setting1, strlen(setting1));
    close(muxFile);

    return 0;
}

Uart::Uart(int uartNumber, int32_t baudRate)
{
    uartHandle = -1;
    isInitialized = false;

    uartNumber = (uartNumber < 1) ? 1 : ((uartNumber > 5) ? 5 : uartNumber);
    char cfgString[] = "uart1pinmux";
    char fileName[] = "/dev/ttyO0";
    cfgString[4] += uartNumber; // Play with the characters in the name.
    fileName[9] += uartNumber;
    if (configMux(cfgString) == 0)
        uartHandle = open(fileName, (uartNumber == 3) ? O_WRONLY : O_RDWR);

    if (uartHandle == -1)
        return;

    speed_t rate;
    switch (baudRate)
    {
    case 50:
        rate = B50;
        break;
    case 75:
        rate = B75;
        break;
    case 110:
        rate = B110;
        break;
    case 134:
        rate = B134;
        break;
    case 150:
        rate = B150;
        break;
    case 200:
        rate = B200;
        break;
    case 300:
        rate = B300;
        break;
    case 600:
        rate = B600;
        break;
    case 1200:
        rate = B1200;
        break;
    case 1800:
        rate = B1800;
        break;
    case 2400:
        rate = B2400;
        break;
    case 4800:
        rate = B4800;
        break;
    case 9600:
        rate = B9600;
        break;
    case 19200:
        rate = B19200;
        break;
    case 38400:
        rate = B38400;
        break;
    case 115200:
        rate = B115200;
        break;
    case 230400:
        rate = B230400;
        break;
    case 460800:
        rate = B460800;
        break;
    case 500000:
        rate = B500000;
        break;
    case 576000:
        rate = B576000;
        break;
    case 921600:
        rate = B921600;
        break;
    case 1000000:
        rate = B1000000;
        break;
    case 1500000:
        rate = B1500000;
        break;
    case 2000000:
        rate = B2000000;
        break;
    case 2500000:
        rate = B2500000;
        break;
    case 3000000:
        rate = B3000000;
        break;
    case 3500000:
        //            rate = B3500000;
        break;
    case 4000000:
        //            rate = B4000000;
        break;
    default:
        return;
    }

    struct termios terminalOptions;
    tcgetattr(uartHandle, &terminalOptions);
    cfsetispeed(&terminalOptions, rate);
    cfsetospeed(&terminalOptions, rate);

    // disable canonical mode processing in the line discipline driver
    // So everything is read in instantly from stdin!
    // Also, don't echo back characters... so we only see what we receive!
    terminalOptions.c_lflag &= ~(ECHO | ICANON);
    // We do not want to block with getchar
    // We have no minimum break in receiving characters (VTIME = 0)
    // and we have no minimum number of characters to receive (VMIN = 0)
    terminalOptions.c_cc[VTIME] = 0;
    terminalOptions.c_cc[VMIN] = 0;

    if (tcsetattr(uartHandle, TCSANOW, &terminalOptions) == -1)
        return;

    isInitialized = true;
}

bool Uart::isReady() const
{
    return isInitialized;
}

int32_t Uart::readByte()
{
    if (!isInitialized)
    {
        return -1;
    }
    uint8_t value = -1;
    int result = read(uartHandle, &value, sizeof(uint8_t));
    // If we didn't read a single byte... we have a problem,
    if (result != sizeof(uint8_t))
    {
        return -1;
    }
    return value;
}

void Uart::writeByte(uint8_t value)
{
    if (isInitialized)
    {
        write(uartHandle, &value, sizeof(uint8_t));
    }
}

void Uart::writeString(const char *str)
{
    if (isInitialized)
    {
        write(uartHandle, str, strlen(str));
    }
}

/*Uart& operator << (Uart& uart, const char* val)
{
    write(uart.uartHandle, val, strlen(val));
    return uart;
}*/
