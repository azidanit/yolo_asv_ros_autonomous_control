#ifndef UTILS1_HPP
#define UTILS1_HPP

#include <math.h>

namespace utils{
    inline double radToDeg(double rad){
        return rad * 180 / M_PI;
    }

    inline double degToRad(double deg){
        return deg * M_PI / 180;
    }

    // inline double calculateRadiusSquare(const pcl::PointXYZ& target, const pcl::PointXYZ& init){
    //     return pow(target.x - init.x, 2) + pow(target.y - init.y, 2);
    // }

    inline double calculateResultant(double a, double b){
        return sqrt(pow(a,2) + pow(b,2));
    }

//    inline double calculateRadiusSquare(const pcl::PointXYZ& target){
//        return pow(target.x, 2) + pow(target.y, 2);
//    }

    inline double calculateDistance(double x0, double y0, double x1, double y1){
        return calculateResultant(x1-x0, y1-y0);
    }

    inline double calculateSpeedFromVel(double vx, double vy){
        return  calculateResultant(vx, vy);
    }

    inline double findXFromLine(double angle_rad, double y){
        return tan(angle_rad) * y;
    }

    inline double findYFromLine(double angle_rad, double x){
        return x / tan(angle_rad);
    }

    inline geometry_msgs::Point findProjection(geometry_msgs::Point pangkal, geometry_msgs::Point ujung, geometry_msgs::Point pt){
        geometry_msgs::Point res;
        double x_l = pangkal.x - ujung.x;
        double y_l = pangkal.y - ujung.x;
        double x_p = pt.x - ujung.x;
        double y_p = pt.y - ujung.y;

        double norm_2 = pow(x_l,2) + pow(y_l, 2);

        res.x = ((x_l * x_p) + (y_l * y_p)) / norm_2 * x_l;
        res.y = ((x_l * x_p) + (y_l * y_p)) / norm_2 * y_l;
        return res;
    }

    inline double getAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
        return atan2( -p1.x + p2.x, -p1.y + p2.y );
    }

    
    /** @brief
    * Check whether a line crossing a point /pos/ with gradient = 0
    * intersects a line segment of (p1, p2)
    * 
    * @params:
    *   - pos : a point in the line to be checked
    *   - p1, p2 : start and end point of the line segment.
    * 
    * @return : true if intersecting, false otherwise.
    *
    */
    inline int isIntersecting( const geometry_msgs::Point& pos,
                                const geometry_msgs::Point& p1,
                                const geometry_msgs::Point& p2){
        

        double x_intersection;
        double y_intersection = pos.y;

        //////////////////////////////////////
        //      Finding the intersection    //
        //////////////////////////////////////

        if (p2.y - p1.y == 0){ // parallel lines
            return 0;
        }

        /*
        * (x-x1) / (y-y1) = (x2-x1) / (y2-y1)
        * x = x1 + [ (x2-x1)(y-y1) / (y2-y1)]
        */
        x_intersection = p1.x + ( (p2.x - p1.x) * (y_intersection - p1.y) / (p2.y - p1.y));

        // if the intersection is on the right, return true.
        if(x_intersection >= pos.x && 
            x_intersection <= fmax(p1.x, p2.x) &&
            x_intersection >= fmin(p1.x, p2.x)
        ){
            return 1;
        }
        
        return 0;
    }

    /**
    * Check whether a point is inside a polygon.
    * 
    * @params:
    *   - pos : the point to be checked
    *   - p1, p2, p3, p4 : vertices of the polygon
    * 
    * @return : true if the pos inside the polygon, false otherwise
    * 
    * Note: a point is inside a polygon iff any line segment started at that point
    *       intersects exactly a vertex of the polygon.
    *       Here, we use grad = 0 and right side of the line crossing point /pos/ as the line segment.
    */
    inline bool isInsidePolygon(const geometry_msgs::Point& pos,
                                const geometry_msgs::Point& p1,
                                const geometry_msgs::Point& p2,
                                const geometry_msgs::Point& p3,
                                const geometry_msgs::Point& p4){
        int n_intersection =  isIntersecting(pos, p1, p2)
                            + isIntersecting(pos, p2, p3)
                            + isIntersecting(pos, p3, p4)
                            + isIntersecting(pos, p4, p1);
        
        if(n_intersection==1){
            return true;
        }
        else{
            return false;
        }
    }

    inline double distanceLineToPoint(const geometry_msgs::Point& pos, const geometry_msgs::Point& sample_point, double angle){
       double m = tanf64(angle);
       double d;

       // calculate distance
       double dividend = fabs( m * pos.x - pos.y + (sample_point.y - m * sample_point.x));
       double divisor = sqrtf64( m*m + 1 );
       d = dividend / divisor;

    //    std::cout << "POINT TO LINE " << d << " angle " << radToDeg(angle) << " m " << m << std::endl;
       return d;
    }

    inline bool isGoodTrotoarDetection(double x_r, double x_l, double angle_r, double angle_l){
        return  (x_r >= 0 && x_l <= 0 ) &&
                (radToDeg(fabs(angle_r - angle_l)) <= 30);
    }
}

#endif // UTILS1_HPP