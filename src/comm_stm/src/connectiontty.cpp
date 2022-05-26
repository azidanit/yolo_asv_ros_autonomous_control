#include "comm_stm/connectiontty.h"

ConnectionTTY::ConnectionTTY()
{
}
ConnectionTTY::ConnectionTTY(std::string ttyusb, int port, int baud_rate_)
{
  this->port = port;
  this->ttyusb = ttyusb;
  stm = new SerialConnection(this->ttyusb, baud_rate_);
  stm->close_connection();
  stm->open_connection();
}

ConnectionTTY::~ConnectionTTY()
{
  stm->close_connection();
}

void ConnectionTTY::start()
{
  sub_drive_system = node.subscribe("/data_to_stm", 10, &ConnectionTTY::controlCallback, this);
  pub_stm_data = node.advertise<std_msgs::String>("/data_from_stm", 10);
  pub_stm_state = node.advertise<std_msgs::Bool> ("/stm_connected",10);
  pub_remote_auto_state = node.advertise<std_msgs::Bool>("/remote_auto_state",10);
  read_stm_thread = std::thread(&ConnectionTTY::readDataFromSTM, this);

//  run_thread = std::thread(&ConnectionTTY::run, this);
//  read_stm_thread.join();
//  run_thread.join();
    run();
}
void ConnectionTTY::run()
{
  ros::Rate rate(50);
  int counter(0);
  while (ros::ok())
  {
	// else
	{
	  ros::spinOnce();
        std::cout << "SPINONCE\n";
	  rate.sleep();
	}
	// try
	//{
	// rx_len = recvfrom(sockfd_rx, udp_rx, sizeof(udp_rx), MSG_DONTWAIT, (struct sockaddr *)&dummy_addr,
	// &dummy_socklen); if(rx_len>0)
	//{
	//	;
	//}
	// else
	// if (timeout_count >= MAX_TIMEOUT_COUNT)
	// {
	//   motorL = 1500;
	//   motorR = 1500;
	//   servoL = 1500;
	//   servoR = 1500;
	//   startRobotListener();
	// }
	// else
	//   timeout_count++;
	//}
	// if (counter % 50 == 0)
	//{
	//  std::cout << "EXIST " << stm->is_device_connected() << std::endl;
	//}
	// counter++;
  }
}

void ConnectionTTY::send_to_stm(int motorL, int motorR, int servoL, int servoR, int tekinL,int tekinR)
{
  char kirim_stm[100];
  sprintf(kirim_stm, "A%04d,%04d,%04d,%04d,%04d,%04dB", motorL, motorR, servoL, servoR,tekinL,tekinR);
  std::cout << kirim_stm << std::endl;
  stm->write_data(kirim_stm, strlen(kirim_stm));
}

void ConnectionTTY::stopRobotListener()
{
  sub_drive_system.shutdown();
}

void ConnectionTTY::controlCallback(const comm_stm::drive_system& msg)
{
//    data_mtx.lock();
  motorL = msg.motor_kiri;
  motorR = msg.motor_kanan;
  servoL = 1500-(msg.servo_kiri-1500);
  servoR = 1500-(msg.servo_kanan-1500);
  tekinL = msg.tekin_kiri;
  tekinR = msg.tekin_kanan;
//  printf("MOTOR L:%d R:%d, SERVO L:%d R:%d\n", motorL, motorR, servoL, servoR);
    printf("MOTOR L:%d R:%d, SERVO L:%d R:%d, TEKIN L:%d R:%d\n", motorL, motorR, servoL, servoR,tekinL,tekinR);

    send_to_stm(motorL, motorR, servoL, servoR,tekinL,tekinR);
}

void ConnectionTTY::readDataFromSTM()
{
    ros::Rate read_rate(50);
    int counter = 0;
    std_msgs::Bool boolmsg;
    while (ros::ok()){
        if(!stm->is_device_connected())
        {
            boolmsg.data = false;
            pub_stm_state.publish(boolmsg);
            if (counter%150 == 0)
            {
                stm->close_connection();
                stm->open_connection();
                counter = 0;
            }
        }
        else{
            boolmsg.data = true;
            pub_stm_state.publish(boolmsg);
        }
        read_rate.sleep();
        std::cout << "READ\n";
        char buf[256];
        if (stm->read_data(buf) <= 0)
            continue;
        STMdata = std::string(buf);
//        bool stateRemote;
//        sscanf(STMdata,)
//        if(STMdata.find('0')!=std::string::npos)
//            boolmsg.data = true;
//        else
//            boolmsg.data = false;

        pub_remote_auto_state.publish(boolmsg);

        std_msgs::String tmp;
        tmp.data = STMdata;
        pub_stm_data.publish(tmp);

        counter++;
    }
}
// void Misi::readDataFromSTM(){
//     char buf[3*MSGLEN + 1];
//     char* px = NULL, *py = NULL;
//     int page;
//     char manat, startstop, param, change;
//     while(!forceCloseSTM){
//         int ln = stm->read_data(buf);
//         if(ln <= 0){
//             emit statusSTMNow(false);
//             stm->close_ConnectionTTY();
//             stm->open_ConnectionTTY();
//         }
//         else{   // bisa ngebaca -> connected
//             emit statusSTMNow(true);
//             mtx_read.lock();

//             if(ln>2)
//             {
//                 px = strchr(buf, 'x');
//                 if(px!=NULL){
//                     py = strchr(px, 'y');
//                     if(py!=NULL){
//                         paramPointer = -1;
//                         *py = '\0';
//                         sscanf(px+1, "%c,%c,%c,%d,%c,%c", &manat, &startstop,&mode_speed, &page, &param, &change);
//                         // param += '0';
//                         // qDebug() << "baca: " << startstop << page, param, change;
//                         changeParams(manat, startstop, page, param, change);
//                         if(paramPointer != -1){
//                             sprintf(strResponse, "C%d,%c,%04dD", page, param, param=='m'? (int)(paramPointer) :
//                             (int)(paramPointer * 100));
//                             // sprintf(strResponse, "%s%s", strResponse, strResponse);
//                             qDebug()<<"-------------";
//                             printf("%f %f %f %d\n%f %f %f %d\n-----------\n", pid[1].p, pid[1].i, pid[1].d, speed[1],
//                             pid[2].p, pid[2].i, pid[2].d, speed[2]); stm->write_data(strResponse);
//                                 emit paramChanged(page, param, paramPointer);
//                         }
//                     }
//                 }
//                 else{
//                     px = strchr(buf, 'v');
//                     if(px!=NULL){
//                         py = strchr(px+1, 'w');
//                         if(py!=NULL){
//                             *py = '\0';
//                             // ubah mode
//                         }
//                     }
//                 }
//             }

//             // qDebug()<<"-----------------------";
//             // qDebug()<<pid[1].p<<pid[1].i<<pid[1].d<<speed[1];
//             // qDebug()<<pid[2].p<<pid[2].i<<pid[2].d<<speed[2];
//             // qDebug()<<"----------------";
//             /////// BELUM NRIMA TOMBOL
//             // if((ln>2 && buf[1]=='0' || buf[1]=='2' )&& buf[1]!=manauto){
//             //     mtx_read.lock();
//             //     manauto = buf[1];
//             //     mtx_read.unlock();
//             //     emit manautoChanged(manauto);
//             // }
//             mtx_read.unlock();
//         }
//     }
//     stm->close_ConnectionTTY();
//     qDebug()<<"STM Closed";
//     quit();
// }