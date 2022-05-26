#include <iostream>
#include "comm_stm/serialconnection.h"
#include "cstring"

SerialConnection::SerialConnection(std::string device_t, int baudrate_t)
{
    strcpy(device, device_t.c_str());
    baudrate = baudrate_t;
    connected = 0;
    // open_connection();
}

SerialConnection::~SerialConnection()
{
}

void SerialConnection::write_data(const char tty[256], int lenght_buffer)
{
    if (!connected)
        return;
    char bufferdata[100];

    tcdrain(fd);
    usleep(10);  // required to make flush work, for some reason
//  tcflush(fd, TCOFLUSH);

    // sprintf(bufferdata,tty);
    wlen = write(fd, tty, lenght_buffer);
    usleep(1000);
    if (wlen != strlen(tty))
    {
        std::cout << "Error from write: " << wlen << " " << errno;
    }
    else
    {
//    std::cout << "Success write : " << tty;
//    if (tcdrain(fd) == -1)
//    {
//      std::cout << "error draining";
//    }
//    else
//      std::cout << "success draining";
        usleep(1000);
    }
}

int SerialConnection::read_data(char buf[256])
{
    // //qdebug()<<"read data";
    if (!connected)
    {
        // close_connection();
        // open_connection();
        // qdebug()<<"trying to connect";
        return -1;
    }
    // do {
    int rdlen;
    // usleep(1);
    tcflush(fd, TCIFLUSH);

    rdlen = read(fd, buf, 6);
    // usleep(100);

    if (rdlen > 0)
    {
        buf[rdlen] = 0;
        connected = 1;

//    printf("Read %d: \"%s\"\n", rdlen, buf);
    }
    else if (rdlen < 0)
    {
        connected = 0;
        //  printf("Error from read: %d: %s\n", rdlen, strerror(errno));
    }
    else
    { /* rdlen == 0 */
        connected = 0;
        //  printf("Timeout from read\n");
    }
    return rdlen;
    //} while (1);
}

int SerialConnection::open_connection()
{
    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);

//    while(fd < 0){
//
//    }
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", device, strerror(errno));
        connected = 0;
        return -1;
    }
    // //qdebug()<<"Connected to STM";
    connected = 1;
    set_interface_attribs();
    tcflush(fd, TCIOFLUSH);
    usleep(10);
    return 0;
}

void SerialConnection::close_connection()
{
    usleep(10);  // required to make flush work, for some reason
    tcflush(fd, TCIOFLUSH);
    connected = 0;
    close(fd);
}

bool SerialConnection::is_device_connected()
{
    struct stat buffer;
    return (stat(device, &buffer) == 0);
}