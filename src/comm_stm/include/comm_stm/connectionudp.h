#ifndef CONNECTIONUDP_H
#define CONNECTIONUDP_H

#include <arpa/inet.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <mutex>
#include <thread>
#include "comm_stm/connection.h"
#include <geometry_msgs/Twist.h>

class ConnectionUDP : public Connection
{
  //    static short int BUFFER_LENGTH = 32;

  int port_stm;
  int port_pc;
  char ip_stm[25];
  char serial_header[3];

  struct sockaddr_in server_addr;
  struct sockaddr_in client_addr;
  struct sockaddr_in dummy_addr;
  socklen_t dummy_socklen;

  char rx_len, serial_rx[32];
  char tx_len, serial_tx[50];

  int sockfd_rx, sockfd_tx;

  int trim_steer_from_remote, trim_thrust_from_remote;
  std::mutex all_mtx;

  std::thread listen_thread;

  void controlCallback(const geometry_msgs::Twist::ConstPtr& msg);
  std::string UDPdata;

public:
  ConnectionUDP(int _port_stm, int _port_pc, const char* _ip_stm);
  void openPORT();
  void startUDPListener();
  void UDPListenerThread();

  void stopUDPListener();
  void write_to_udp(int motor_L, int motor_R, int servo_L, int servo_R,int,int);
  //    void get_sensor_datas();
  void process_received_msg(const char* msg);
  void publishOffset();
  void start();
};

#endif