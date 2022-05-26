#ifndef CONNECTIONTTY_H
#define CONNECTIONTTY_H

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>


#include "comm_stm/connection.h"
#include "comm_stm/serialconnection.h"

class ConnectionTTY : public Connection
{
private:
  enum VEL_SCALE
  {
    SPEED_RANGE = 500,
    SERVO_RANGE = 500,
  };


  std::mutex data_mtx;
  std::thread read_stm_thread, run_thread;

  // bool updated_value;
  SerialConnection* stm;
  // int mode;
  int port;
  // char serial_tx[7];
  // bool connected;
  // struct sockaddr_in server_addr;

  // struct sockaddr_in dummy_addr;
  // socklen_t dummy_socklen;
  // const int BUFFER_LENGTH = 50;
  // char rx_len, udp_rx[50];     // buffer for received data
  // int sockfd_rx;
  // struct timeval tv;  // time interval for timeout
  // int timeout_count;
  const int MAX_TIMEOUT_COUNT = 100;
  std::string ttyusb;
  std::string STMdata;

  void startRobotListener();
  void stopRobotListener();
  void send_to_stm(int, int, int, int,int,int);
  void controlCallback(const comm_stm::drive_system& msg);
  void run();
  void readDataFromSTM();

public:
  ConnectionTTY();
  ConnectionTTY(std::string, int, int);
  ~ConnectionTTY();
  void start();
};

#endif
