#include "muxConfig.h"

int muxConfig(const char* capeName)
{

    int muxFile = open("/sys/devices/bone_capemgr.8/slots", O_WRONLY);
    if (muxFile == -1) {
        return -1;
    }
    write(muxFile, capeName, strlen(capeName));
    close(muxFile);

    return 0;
}
