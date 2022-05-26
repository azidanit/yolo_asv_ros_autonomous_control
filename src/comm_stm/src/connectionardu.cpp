//
// Created by dion on 27/10/2020.
//

#include <std_msgs/Bool.h>
#include "../include/comm_stm/connectionardu.h"

connectionArdu::connectionArdu(std::string device,int baud_rate)
{
    ardu = new SerialConnection(device,baud_rate);
    ardu->close_connection();
    ardu->open_connection();
    pub_mode = node.advertise<std_msgs::Bool>("/start_from_ardu",10);
}

connectionArdu::~connectionArdu(){
    thread_read.join();
    ardu->close_connection();
}

void connectionArdu::start()
{
    thread_read = std::thread(&connectionArdu::threadRead, this);
//    thread_read.join();
}

void connectionArdu::threadRead() {
    printf("STARTED\n");
    ros::Rate rate(50);
    int rec_error_counter = 0;
    std_msgs::Bool state_to_send;
    state_to_send.data = false;
    while(ros::ok())
    {

//        printf("ARDU_LOOP\n");
        if(!ardu->is_device_connected())
        {
            printf("ardu is not connected.. retrying..");
            ardu->close_connection();
            sleep(1);
            ardu->open_connection();
            continue;
        }

        char buff[7];
        ardu->read_data(buff);

        if(strcmp(buff, "ASTOPB")==0){
            std::cout << "AUTO\n";
            rec_error_counter = 0;
            state_to_send.data = true;
        }else if(strcmp(buff, "ASTRTB")==0){
            std::cout << "MANUAL\n";
            rec_error_counter = 0;
            state_to_send.data = false;
        }else{
            rec_error_counter++;
            if(rec_error_counter>=80){
                rec_error_counter = 0;
                std::cout << "MANUAL\n";
                state_to_send.data = false;
            }
        }
        pub_mode.publish(state_to_send);
        rate.sleep();
    }

}
