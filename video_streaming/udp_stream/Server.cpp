#include "Socket.h"         // For UDPSocket and SocketException
#include <iostream>         // For cout and cerr
#include <cstdlib>          // For atoi()
#include "config.h"
#include "opencv2/opencv.hpp"

using namespace cv;
#define BUF_LEN         65540 // Larger than maximum UDP packet size


int main(int argc, char * argv[]) 
{
    /* RECIEVER  SIDE */
    // Test for correct number of parameters
    if (argc != 2) 
    { 
        cerr << "Usage: " << argv[0] << " <Server Port>" << endl;
        exit(1);
    }

    // First arg:  local PORT
    unsigned short servPort = atoi(argv[1]); 

    namedWindow("Streaming", CV_WINDOW_AUTOSIZE);

    try {
        UDPSocket sock(servPort);

        char buffer[BUF_LEN]; // Buffer for echo string
        int recvMsgSize; // Size of received message
        string sourceAddress; // Address of datagram source
        unsigned short sourcePort; // Port of datagram source

        clock_t last_cycle = clock();

        while (1) 
        {
            // Block until receive message from a client
            do {
                recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
            } while (recvMsgSize > sizeof(int));
            int total_pack = ((int * ) buffer)[0];

            //cout << "expecting length of packs:" << total_pack << endl;
            char * longbuf = new char[PACK_SIZE * total_pack];
            for (int i = 0; i < total_pack; i++) 
            {
                recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
                if (recvMsgSize != PACK_SIZE) 
                {
                    cerr << "Received unexpected size pack:" << recvMsgSize << endl;
                    continue;
                }
                memcpy( & longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
            }

            //cout << "Received packet from " << sourceAddress << ":" << sourcePort << endl;
 
            Mat rawData = Mat(1, PACK_SIZE * total_pack, CV_8UC1, longbuf);
            Mat frame = imdecode(rawData, CV_LOAD_IMAGE_COLOR);
            if (frame.size().width == 0) {
                cerr << "decode failure!" << endl;
                continue;
            }
            imshow("Streaming", frame);
            free(longbuf);

            clock_t next_cycle = clock();
            double duration = (next_cycle - last_cycle) / (double) CLOCKS_PER_SEC;
            //cout << "\teffective FPS:" << (1 / duration) << " \tkbps:" << (PACK_SIZE * total_pack / duration / 1024 * 8) << endl;

            cout << next_cycle - last_cycle;
            last_cycle = next_cycle;
            if (waitKey(10) == 27)
            {
                cout << "END PROGRAM" << endl;
                exit(EXIT_SUCCESS);
            }
        }
    } catch (SocketException & e) {
        cerr << e.what() << endl;
        exit(1);
    }
    return 0;
}