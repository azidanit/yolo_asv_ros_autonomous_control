#include "comm_stm/connectionudp.h"


ConnectionUDP::ConnectionUDP(int _port_stm, int _port_pc, const char* _ip)
{
  trim_steer_from_remote = 1500;
  trim_thrust_from_remote = 1500;

  this->port_stm = _port_stm;
  this->port_pc = _port_pc;
  strcpy(this->ip_stm, _ip);
  openPORT();
  sub_drive_system = node.subscribe("/data_to_stm", 100, &ConnectionUDP::controlCallback, this);
  pub_stm_state = node.advertise<comm_stm::stm_status>("/data_from_stm", 100);
    pub_stm_conn = node.advertise<std_msgs::Bool> ("/stm_connected",10);
    pub_remote_auto_state = node.advertise<std_msgs::Bool>("/remote_auto_state",10);


    start();
 startUDPListener();
}
void ConnectionUDP::openPORT()
{
  sockfd_rx = socket(AF_INET, SOCK_DGRAM, 0);
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(this->port_pc);
  bind(sockfd_rx, (struct sockaddr*)&server_addr, sizeof(server_addr));
  dummy_socklen = sizeof(dummy_addr);

  sockfd_tx = socket(AF_INET, SOCK_DGRAM, 0);
  client_addr.sin_family = AF_INET;
  client_addr.sin_addr.s_addr = inet_addr(this->ip_stm);
  client_addr.sin_port = htons(this->port_stm);
}
void ConnectionUDP::startUDPListener()
{
  listen_thread = std::thread(&ConnectionUDP::UDPListenerThread, this);
}
void ConnectionUDP::UDPListenerThread()
{
  ros::Rate rr_(100);

  while (ros::ok())
  {
    rx_len = recvfrom(sockfd_rx, serial_rx, sizeof(serial_rx), MSG_DONTWAIT, (struct sockaddr*)&dummy_addr, &dummy_socklen);
//    std::cout<<"LENGTH: "<<strlen(serial_rx)<<std::endl;
    if (rx_len > 0)
    {
      memcpy(serial_header, serial_rx, 3);
       std::cout << "RECEIVE " << serial_rx << " LEN " << strlen(serial_rx) << std::endl;
      process_received_msg(serial_rx);
    }
    rr_.sleep();
    memset(serial_rx,0,32);
  }
}

// state remOTE
// auto 0
// manual 2
// qc 3

// speed mode
// 1 HIGH 
// 2 MED 
// 3 LOW

// TRIM
// ambil langsung
void ConnectionUDP::process_received_msg(const char* msg)
{
    std_msgs::Bool data_conn, data_auto;
    if(msg[0]=='i'&&msg[1]=='t'&&msg[2]=='s')
    {
        short int state_remote,
                mode_speed,
                is_tekin_available,
                servo_remote,
                trim,
                motor_remote;//, is_tekin_available,servo_remote;

        short srf_data[4];
        memcpy(&mode_speed,msg+3,2);
        memcpy(&state_remote,msg+5,2);
        memcpy(&is_tekin_available,msg+7,2);
//        memcpy(&is_tekin_available,msg+9,2);
        memcpy(&servo_remote,msg+9,2);
        memcpy(&motor_remote,msg+11,2);
        memcpy(&trim,msg+13,2);
        memcpy(srf_data,msg+15,8);
//        memcpy(&srf_data[0],msg+15,2);
//        memcpy(&srf_data[1],msg+17,2);
//        memcpy(&srf_data[2],msg+19,2);
//        memcpy(&srf_data[3],msg+21,2);
        std::cout<<"STATE REMOTE    : "<<state_remote<<std::endl
                 <<"SPEED MODE      : "<<mode_speed<<std::endl
                 <<"TEKIN AVAILABLE : "<<is_tekin_available<<std::endl
                 //                 <<"TEKIN AVAILABLE : "<<is_tekin_available<<std::endl
                 <<"SERVO REMOTE PWM: "<<servo_remote<<std::endl
                 <<"MOTOR REMOTE PWM: "<<motor_remote<<std::endl
                 <<"TRIM REMOTE PWM : "<<trim<<std::endl
                 <<"SRF1            : " << srf_data[0] << std::endl
                 <<"SRF2            : " << srf_data[1] << std::endl
                 <<"SRF3            : " << srf_data[2] << std::endl
                 <<"SRF4            : " << srf_data[3] << std::endl
                 ;
        // short int rc_steer, rc_thrust, checksum;
        // memcpy(&rc_steer,msg+3,2);
        // memcpy(&rc_thrust,msg+3,2);
        // memcpy(&checksum,msg+3,2);
        // std::cout<<"RC STEER   :"<<rc_steer;
        // std::cout<<"RC THRUST  :"<<rc_thrust;
        comm_stm::stm_status to_send;
        to_send.state_remote = state_remote;
        to_send.speed_mode = mode_speed;
        to_send.is_tekin_available = is_tekin_available;
        to_send.remote_servo = servo_remote;
        to_send.remote_motor = motor_remote;
        if(servo_remote == 0)
            to_send.trim = -1;
        else
            to_send.trim = servo_remote;
        pub_stm_state.publish(to_send);
        // mode speed 1 2 3: high medium low
        // state remote : 0 auto 2 manual
        //
//        memcpy(&state_remote,msg+3,2);
//        memcpy(&state_remote,msg+3,2);
        if (state_remote == 0){
            data_auto.data = true;
        }else{
            data_auto.data = false;
        }
        data_conn.data = true;
    }else{
        data_conn.data = false;
    }
    pub_stm_conn.publish(data_conn);
    pub_remote_auto_state.publish(data_auto);
//    UDPdata = std::string(msg);
}

void ConnectionUDP::start()
{
  ros::Rate rate(50);
  // int counter(0);
  // startRobotListener();
  startUDPListener();

  while (ros::ok())
  {
    ros::spinOnce();
    // write_to_udp(motorL, motorR, servoL, servoR);
    std_msgs::String tmp;
    tmp.data = UDPdata;
//    pub_stm_data.publish(tmp);
    rate.sleep();
  }
}
void ConnectionUDP::controlCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    motorL = motorR = msg->linear.x + 1500;
    servoL = servoR = msg->angular.z + 1500;

    motorL += msg->angular.z;
    motorR -= msg->angular.z;

    printf("MOTOR L:%d R:%d, SERVO L:%d R:%d, TEKIN L:%d R:%d\n", motorL, motorR, servoL, servoR,tekinL,tekinR);
    write_to_udp(motorL, motorR, servoL, servoR,tekinL,tekinR);
}

void ConnectionUDP::write_to_udp(int motor_L, int motor_R, int servo_L, int servo_R, int tekin_L, int tekin_R)
{
    motorL = motor_L;
    motorR = motor_R;
    servoL = servo_L;
    servoR = servo_R;
    tekinL = tekin_L;
    tekinR = tekin_R;

    short int motorL_ = motor_L;
    short int motorR_ = motor_R;
    short int servoL_ = servo_L;
    short int servoR_ = servo_R;
    short int tekinL_ = tekin_L;
    short int tekinR_ = tekin_R;

    // short int drive_train[4];

    short int checksum_kirim;
    checksum_kirim = motorR_ ^ motorL_ ^ servoR_ ^ servoL_;

    // memcpy(serial_tx + 3, drive_train, 2 * 4);

    serial_tx[0] = 'i';
    serial_tx[1] = 't';
    serial_tx[2] = 's';
    memcpy(serial_tx + 3, &motorL_, 2);
    memcpy(serial_tx + 5, & motorR_, 2);
    memcpy(serial_tx + 7, &servoR_, 2);
    memcpy(serial_tx + 9, &servoL_, 2);
    memcpy(serial_tx + 11, &tekinR_, 2);
    memcpy(serial_tx + 13, &tekinL_, 2);
    // memcpy(serial_tx + 11, &tekinR_, 2);
    // memcpy(serial_tx + 13, &tekinL_, 2);

//  sprintf(serial_tx, "A%04d,%04d,%04d,%04,%04d,%04dB", motorL, motorR, servoL, servoR,tekinL,tekinR);
  tx_len = sendto(sockfd_rx, serial_tx, 24, 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
  // std::cout << serial_tx << std::endl;  
}