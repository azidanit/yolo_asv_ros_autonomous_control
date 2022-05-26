#ifndef SERIALCONNECTION_H
#define SERIALCONNECTION_H

//#include <QThread>
//#include "tele_kapal.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <string>

#include <sys/stat.h>
#include <fstream>

class SerialConnection
{
public:
  SerialConnection(std::string, int);
  ~SerialConnection();

  int open_connection();
  void close_connection();
  void write_data(const char *, int);
  int read_data(char *buf);
  bool is_device_connected();

private:
  int fd;
  int connected;
  char ttyType[30];
  std::string device_name;
  char *device = new char[21];
  int baudrate;
  int wlen;
  struct termios tty;

  int set_interface_attribs()
  {
    if (tcgetattr(fd, &tty) < 0)
    {
      printf("Error from tcgetattr: %s\n", strerror(errno));
      return -1;
    }

    cfsetospeed(&tty, (speed_t)baudrate);
    cfsetispeed(&tty, (speed_t)baudrate);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSAFLUSH, &tty) != 0)
    {
      printf("Error from tcsetattr: %s\n", strerror(errno));
      return -1;
    }
    return 0;
  }
};

#endif  // SERIALCONNECTION_H