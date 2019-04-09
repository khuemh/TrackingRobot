#include <iostream>
#include <wiringPi.h>
#include "Socket.h"

using namespace std;

//int buff[2] = {111, 222};

int main(int argc, char const *argv[])
{
    if ((argc < 3) || (argc > 3)) 
    { // Test for correct number of arguments
        cerr << "Usage: " << argv[0] << " <Server> <Server Port>\n";
        exit(1);
    }

    string servAddress = argv[1]; // First arg: server address
    unsigned short servPort = Socket::resolveService(argv[2], "udp");
    
        UDPSocket sock;
        while(1)
        {
            int buff[2] = {111, 222};
            sock.sendTo(buff, sizeof(buff), servAddress, servPort);
            cout << "sent" << endl;
            delay(1000);
        }
    return 0;
}
