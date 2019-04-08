#include <iostream>
#include "Socket.h"

using namespace std;

int buff[2];

int main(int argc, char const *argv[])
{
    buff[0] = 222;
    buff[1] = 111;


    if ((argc < 3) || (argc > 3)) 
    { // Test for correct number of arguments
        cerr << "Usage: " << argv[0] << " <Server> <Server Port>\n";
        exit(1);
    }

    string servAddress = argv[1]; // First arg: server address
    unsigned short servPort = Socket::resolveService(argv[2], "udp");

    UDPSocket sock;

    sock.sendTo(buff, 2, servAddress, servPort);

    delay(1000);
    return 0;
}
