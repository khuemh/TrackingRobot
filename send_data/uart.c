#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <unistd.h>

#define DATA_SIZE           2

int fd;
uint8_t data8[DATA_SIZE];
uint16_t data16;

void serialCheck()
{
    if((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0 )
    {    
         fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
    }
}

void send(const char *data)
{
        serialPuts(fd, data);
        serialFlush(fd);
}

int main()
{

    uint16_t data16 = 1234; // 0x04D2
    
    data8[0] = (data16 >> 8) & 0xFF;
    data8[1] = (data16 >> 0) & 0xFF;
    
    printf("%c \n", data8[0]);


            while(1) 
            {
                    serialCheck();
                
                    write(fd, data8[0], 1u);

                    send("\n");
                    
                    delay(1000);
                    
                    serialClose(fd);

             }

         return 0;
}
