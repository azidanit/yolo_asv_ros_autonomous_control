#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include "comm_stm/connectiontty.h"
#include "comm_stm/connectionudp.h"
#include "comm_stm/connectionardu.h"
short int speed = 0, servo = 0;

bool updated_value = false;
connectionArdu* arducomm;
std::thread* arduthread;
void startArduComm()
{
    arducomm->start();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "comm_stm");
    std::string mode(argv[1]);
    Connection* comm;

    // arducomm = new connectionArdu("/dev/arduino_nano",9600);

    // arducomm->start();

    if (mode == "tty")
    {
        std::string ttyusb(argv[2]);
        int baud_rate;
        baud_rate = std::stoi(argv[3]);
        printf("BAUDRATE %d\n", baud_rate);
        comm = new ConnectionTTY(ttyusb, 9898, baud_rate);
    }
    else if (mode == "udp")
    {
        if (argc < 5)
        {
            printf(":'v\n");
            return 1;
        }
        int stm_port = -1;
        int pc_port = -1;
        stm_port = atoi(argv[2]);
        pc_port = atoi(argv[3]);

        printf("%s %d\n", argv[4], stm_port);
        //(stm_port,pc_port,address)
        comm = new ConnectionUDP(stm_port, pc_port, argv[4]);
    }
    else
    {
        printf(":'v\n");
        return 1;
    }



    // delete comm;
    return 0;
}