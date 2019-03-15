#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>



int fd;

void send(const char *data)
{
        serialPuts(fd, data);
        serialFlush(fd);
}

void serialCheck()
{
    if((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0 )
    {    
         fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
    }
}

int main()
{
            while(1) 
            {
                    serialCheck();

                    send("Hello");
//                 serialPuts(fd, "hello");
//
//                 serialFlush(fd);
                 delay(1000);
             }
         return 0;
}
