////
//// Created by azidanit on 5/30/22.
////
//
////
//// Created by azidanit on 4/8/22.
////
//
//#include "sub_control/WaypointControl.h"
//
//WaypointControl::WaypointControl(MissionWrapper *ct) {
//    this->ct_ = ct;
//
//    path_error_dist = path_error_angle = 0;
//    path_error_dist_before = path_error_angle_before = 0;
//
//    is_using_left_path = true;
//
//    left_path_idx = 20;
//    right_path_idx = 20;
//    left_path_idx_before = 0;
//    right_path_idx_before = 0;
//
//    path_idx = 10;
//    path_idx_before = 0;
//
//    path_keep_distance = 7;
//    path_angle_limit = 50;
//    path_length_trace = 10;
//
//    main_path = ct_->getPathMain();
////    left_path = ct_->getPathMain();
//
////    std::cout << "debug crash sampai sini 0\n";
//
//}
//
//WaypointControl::~WaypointControl() {
//
//}
//
//geometry_msgs::Twist WaypointControl::calculateOut() {
//    geometry_msgs::Twist output;
//
//    //{TODO} Check using which path
//    if (main_path.poses.size() < (path_idx+10) ){
//        output.linear.x = 0;
//        output.angular.z = 0;
//        output.angular.x = 0;
//
//        return output;
//    }
//    if (isArrivedPath()){
//        path_idx++;
//        path_idx_before = path_idx - path_length_trace;
//        if(path_idx_before < 0) {
//            path_idx_before = 0;
//        }
////        std::cout << "SAMPAI PATH NEXT PATH\n";
//    }
//
////    std::cout << "debug crash sampai sini 1\n";
//
//    double angle_path = utils::getAngleInvert(main_path.poses[path_idx_before].pose.position,
//                                              main_path.poses[path_idx].pose.position);
//
//    std::cout << "ANGGLE PATH BEFORE " << angle_path << "\n";
////    angle_path += M_PI/2;
////    if (angle_path > 0){
////        angle_path = (-1 * M_PI) + angle_path;
////    }else {
////        angle_path = M_PI + angle_path;
////    }
//    path_error_angle = utils::radToDeg(angle_path - ct_->getRobotTf().yaw);
//
//    if ( path_error_angle < -180 ) path_error_angle += 360; //waypoint behind robot heading
//    else if (path_error_angle > 180) path_error_angle -= 360;
//
////    std::cout << "BOAT TF YAW " << utils::radToDeg(boatTF.yaw) << " AP "<< utils::radToDeg(angle_path) << std::endl;
////    std::cout <<"ERROR ANGLE PATH " << path_error_angle << std::endl;
//
//    path_error_dist = distanceLineToPoint(getInitPathPoint(), getNextPathPoint(), ct_->getRobotTf().pos);
////    std::cout << "ERROR DIST PATH: " << path_error_dist <<  std::endl;
//
////    std::cout << "debug crash sampai sini 2\n";
//    error_acc_angle_path = path_error_angle + path_error_angle_before;
//    error_acc_dist_path = path_error_dist + path_error_dist_before;
//    std::cout << "YAW A " << angle_path << " RB " << ct_->getRobotTf().yaw << " ERROR "  << path_error_angle << "\n";
//    std::cout << "ERROR D " << path_error_dist << "\n";
//    path_controlDistanceOut = ct_->calculateErrorPID(ct_->getPidDistance(), path_error_dist,
//                                                     path_error_dist_before, error_acc_dist_path);
//    path_controlAngleOut = ct_->calculateErrorPID(ct_->getPidAngle(), path_error_angle,
//                                                  path_error_angle_before, error_acc_angle_path);
//
//    path_error_dist_before = path_error_dist;
//    path_error_angle_before = path_error_angle;
//
//    double result = path_controlAngleOut + path_controlDistanceOut;
//
//    output.angular.z = result;
//
//    //publish marker
//    ct_->publishMarker(ct_->getRobotTf().pos ,getInitPathPoint(), getNextPathPoint());
//
//    return output;
//}
//
//bool WaypointControl::isArrivedLeftPath() {
//
//    geometry_msgs::Point controlled_path_point = left_path.poses[left_path_idx].pose.position;
//    geometry_msgs::Point controlled_path_point_before = left_path.poses[left_path_idx_before].pose.position;
//
//    double delta_x = ct_->getRobotTf().pos.x - controlled_path_point.x;
//    double delta_y = ct_->getRobotTf().pos.y - controlled_path_point.y;
//    double angle = utils::radToDeg(
//            utils::getAngle(ct_->getRobotTf().pos, controlled_path_point)
//            - utils::getAngle(controlled_path_point_before, controlled_path_point)
//    );
//
//    if (fabs(angle) >= 180){
//        angle = 360 - fabs(angle);
//    }
////    std::cout << "IS ARRIVED PATH ANGLE" << angle << " KEEP DIST " << (float)path_keep_distance << " trace " << path_length_trace <<"\n";
////    std::cout << "PATH IDX" << path_points_idx << "\n";
//
////    if(!(fabs(angle) < path_angle_limit and pow(delta_x, 2) + pow(delta_y, 2) > pow(path_keep_distance,2)))
////        std::cout << "Angle of init, boat, dest : " << angle << " IDX: " << path_points_idx << std::endl;
//    return !(fabs(angle) < path_angle_limit and pow(delta_x, 2) + pow(delta_y, 2) > pow(path_keep_distance,2));
//
//}
//
//bool WaypointControl::isArrivedPath() {
////    return isArrivedLeftPath();
//    geometry_msgs::Point controlled_path_point = main_path.poses[path_idx].pose.position;
//    geometry_msgs::Point controlled_path_point_before = main_path.poses[path_idx_before].pose.position;
//
//    double delta_x = ct_->getRobotTf().pos.x - controlled_path_point.x;
//    double delta_y = ct_->getRobotTf().pos.y - controlled_path_point.y;
//    double angle = utils::radToDeg(
//            utils::getAngle(ct_->getRobotTf().pos, controlled_path_point)
//            - utils::getAngle(controlled_path_point_before, controlled_path_point)
//    );
//
//    if (fabs(angle) >= 180){
//        angle = 360 - fabs(angle);
//    }
//
//    return !(fabs(angle) < path_angle_limit and pow(delta_x, 2) + pow(delta_y, 2) > pow(path_keep_distance,2));
//
//}
//
//inline double WaypointControl::distanceLineToPoint(const geometry_msgs::Point& init, const geometry_msgs::Point& dest, const geometry_msgs::Point& curr){
//    double y2Miny1 = dest.y-init.y;
//    double x2Minx1 = dest.x-init.x;
//    std::cout << init << dest << curr << "\n";
//    return ((y2Miny1*curr.x - x2Minx1*curr.y + dest.x*init.y - dest.y*init.x)
//            / sqrt(y2Miny1*y2Miny1 + x2Minx1*x2Minx1));
//}
//
//geometry_msgs::Point WaypointControl::getNextPathPoint() {
//    return main_path.poses[path_idx].pose.position;
//}
//
//geometry_msgs::Point WaypointControl::getInitPathPoint() {
//    return main_path.poses[path_idx_before].pose.position;
//}