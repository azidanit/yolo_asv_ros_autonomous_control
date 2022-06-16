
//
// Created by azidanit on 4/8/22.
//

#include "sub_control/WaypointControl.h"

WaypointControl::WaypointControl(Control *ct,  Misi* ms, PIDController* pid_d, PIDController* pid_a) {
   this->ct_ = ct;
   this->ms_ = ms;
   this->pid_angle = pid_a;
   this->pid_distance = pid_d;

   path_error_dist = path_error_angle = 0;
   path_error_dist_before = path_error_angle_before = 0;

   path_idx = 1;
   path_idx_before = 0;

   path_keep_distance = 7;
   path_angle_limit = 50;
   path_length_trace = 1;

//    main_path = ct_->getPathMain();

   std::cout << "CREATED WaypoinT Control From " << ms_->mission_name << std::endl;

   targetMarker_pub = ct_->nh.advertise<visualization_msgs::Marker>("/waypoint_control/target_marker", 1);
   error_angle_pub = ct_->nh.advertise<std_msgs::Float64>("/waypoint_control/error/angle", 1);
   error_distance_pub = ct_->nh.advertise<std_msgs::Float64>("/waypoint_control/error/distance", 1);
}

WaypointControl::~WaypointControl() {

}

void WaypointControl::setPath(nav_msgs::Path path_){
   std::cout << "get Path wp in " << ms_->mission_name << std::endl;
   main_path = path_;
}

void WaypointControl::stopAndReset(){
   path_idx = 1;
   path_idx_before = 0;

   publishMarker(ct_->getRobotTf().pos ,ct_->getRobotTf().pos, ct_->getRobotTf().pos);
}


geometry_msgs::Twist WaypointControl::calculateOut() {
   geometry_msgs::Twist output;

   //{TODO} Check using which path
   if (main_path.poses.size() < (path_idx+1) ){
       output.linear.x = 0;
       output.angular.z = 0;
       output.angular.x = 0;

       std::cout << "WP CONTROL, NO WP\n";

       return output;
   }
   if (isArrivedPath()){
       path_idx++;
       path_idx_before = path_idx - path_length_trace;
       if(path_idx_before < 0) {
           path_idx_before = 0;
       }
//        std::cout << "SAMPAI PATH NEXT PATH\n";
   }

//    std::cout << "debug crash sampai sini 1\n";

   double angle_path = utils::getAngleInvert(main_path.poses[path_idx_before].pose.position,
                                             main_path.poses[path_idx].pose.position);

   std::cout << "ANGGLE PATH BEFORE " << angle_path << "\n";
//    angle_path += M_PI/2;
//    if (angle_path > 0){
//        angle_path = (-1 * M_PI) + angle_path;
//    }else {
//        angle_path = M_PI + angle_path;
//    }
   path_error_angle = utils::radToDeg(angle_path - ct_->getRobotTf().yaw);

   if ( path_error_angle < -180 ) path_error_angle += 360; //waypoint behind robot heading
   else if (path_error_angle > 180) path_error_angle -= 360;

//    std::cout << "BOAT TF YAW " << utils::radToDeg(boatTF.yaw) << " AP "<< utils::radToDeg(angle_path) << std::endl;
//    std::cout <<"ERROR ANGLE PATH " << path_error_angle << std::endl;

   path_error_dist = distanceLineToPoint(getInitPathPoint(), getNextPathPoint(), ct_->getRobotTf().pos);
//    std::cout << "ERROR DIST PATH: " << path_error_dist <<  std::endl;

//    std::cout << "debug crash sampai sini 2\n";
   error_acc_angle_path = path_error_angle + path_error_angle_before;
   error_acc_dist_path = path_error_dist + path_error_dist_before;
   std::cout << "YAW A " << angle_path << " RB " << ct_->getRobotTf().yaw << " ERROR "  << path_error_angle << "\n";
   std::cout << "ERROR D " << path_error_dist << "\n";

   path_controlDistanceOut = pid_distance->updateError(path_error_dist);
   path_controlAngleOut = pid_angle->updateError(path_error_angle);

   std_msgs::Float64 msg_err;
   msg_err.data = path_error_angle;
   error_angle_pub.publish(msg_err);
   msg_err.data = path_error_dist;
   error_distance_pub.publish(msg_err);

   double result = path_controlAngleOut + path_controlDistanceOut;

   output.angular.z = result;

   // //publish marker
   publishMarker(ct_->getRobotTf().pos ,getInitPathPoint(), getNextPathPoint());

   return output;
}

bool WaypointControl::isArrivedPath() {
   geometry_msgs::Point controlled_path_point = main_path.poses[path_idx].pose.position;
   geometry_msgs::Point controlled_path_point_before = main_path.poses[path_idx_before].pose.position;

   double delta_x = ct_->getRobotTf().pos.x - controlled_path_point.x;
   double delta_y = ct_->getRobotTf().pos.y - controlled_path_point.y;
   double angle = utils::radToDeg(
           utils::getAngle(ct_->getRobotTf().pos, controlled_path_point)
           - utils::getAngle(controlled_path_point_before, controlled_path_point)
   );

   if (fabs(angle) >= 180){
       angle = 360 - fabs(angle);
   }

   return !(fabs(angle) < path_angle_limit and pow(delta_x, 2) + pow(delta_y, 2) > pow(path_keep_distance,2));

}

inline double WaypointControl::distanceLineToPoint(const geometry_msgs::Point& init, const geometry_msgs::Point& dest, const geometry_msgs::Point& curr){
   double y2Miny1 = dest.y-init.y;
   double x2Minx1 = dest.x-init.x;
   std::cout << init << dest << curr << "\n";
   return 10*((y2Miny1*curr.x - x2Minx1*curr.y + dest.x*init.y - dest.y*init.x)
           / sqrt(y2Miny1*y2Miny1 + x2Minx1*x2Minx1));
}

geometry_msgs::Point WaypointControl::getNextPathPoint() {
   return main_path.poses[path_idx].pose.position;
}

geometry_msgs::Point WaypointControl::getInitPathPoint() {
   return main_path.poses[path_idx_before].pose.position;
}

void WaypointControl::publishMarker(geometry_msgs::Point p0, geometry_msgs::Point p2, geometry_msgs::Point p1){
   visualization_msgs::Marker targetLine;

   targetLine.type = visualization_msgs::Marker::LINE_STRIP;
    targetLine.header.frame_id = "map";
    targetLine.header.stamp = ros::Time::now();
    targetLine.ns = "lines";
    targetLine.action = visualization_msgs::Marker::ADD;
    targetLine.scale.x = 0.05;
    targetLine.color.g = 1.0;
    targetLine.color.a = 1.0;

   targetLine.pose.orientation.x = 0;
   targetLine.pose.orientation.y = 0;
   targetLine.pose.orientation.z = 0;
   targetLine.pose.orientation.w = 1;

//    std::cout << p1 << p2 << std::endl;
    targetLine.points.clear();
    targetLine.color.a = 1.0;
//
    targetLine.header.stamp = ros::Time::now();
//
////    p1 = misi[currentMission]->getNextPathPoint();
////    p2 = misi[currentMission]->getInitPathPoint();
    targetLine.points.push_back(p0);
    targetLine.points.push_back(p1);
    targetLine.points.push_back(p2);
    targetMarker_pub.publish(targetLine);
}

void WaypointControl::setWpPIDAngle(PIDController pid_new){
   pid_angle->setP(pid_new.getP());
   pid_angle->setI(pid_new.getI());
   pid_angle->setD(pid_new.getD());
}

void WaypointControl::setWpPIDDistance(PIDController pid_new){
   pid_distance->setP(pid_new.getP());
   pid_distance->setI(pid_new.getI());
   pid_distance->setD(pid_new.getD());
}
