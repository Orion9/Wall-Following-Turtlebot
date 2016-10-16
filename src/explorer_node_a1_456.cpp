/*
* explorer_node_a1_456.cpp
*
* Name  : Oğuz Kerem Tural
* ID        : 150130125
*
*/

//Common ROS headers.
#include "ros/ros.h"

//This is needed for the data structure containing the motor command.
#include "geometry_msgs/Twist.h"
//This is needed for the data structure containing the laser scan.
#include "sensor_msgs/LaserScan.h"
//This is needed for the data structure containing the map (which you may not use).
#include "nav_msgs/OccupancyGrid.h"
//This is for easy printing to console.

#include "tf/transform_listener.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

#define ROBOT_SPEED 0.25

// For information on what publishing and subscribing is in ROS, look up the tutorials.
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
ros::Subscriber map_subscriber;

// For information on what a "message" is in ROS, look up the tutorials.
sensor_msgs::LaserScan laser_msg;
nav_msgs::OccupancyGrid map_msg;
geometry_msgs::Twist motor_command;


//The following function is a "callback" function that is called back whenever a new laser scan is available.
//That is, this function will be called for every new laser scan.
//
// --------------------------------------------------------------------------------------------
//CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY.
// --------------------------------------------------------------------------------------------
//

typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    GO_RIGHT,
    GO_LEFT

} ROBOT_MOVEMENT;

bool robot_move ( const ROBOT_MOVEMENT move_type ) {
    if ( move_type == STOP ) {
        ROS_INFO ( "[ROBOT] HALT! \n" );
        
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
    }

    else if ( move_type == FORWARD ) {
        ROS_INFO ( "[ROBOT] Always FORWARD! \n" );
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.5;
    }

    else if ( move_type == BACKWARD ) {
        ROS_INFO ( "[ROBOT] I'm going back! \n" );
        motor_command.linear.x = -0.75;
        motor_command.angular.z = 0.0;
    }

    else if ( move_type == TURN_LEFT ) {
        ROS_INFO ( "[ROBOT] I'm turning left! \n" );
        motor_command.linear.x = 0.0;
        motor_command.angular.z = 1.0;
    }

    else if ( move_type == TURN_RIGHT ) {
        ROS_INFO ( "[ROBOT] I'm turning right! \n" );
        motor_command.linear.x = 0.0;
        motor_command.angular.z = -1.0;
    } 
    else if ( move_type == GO_RIGHT ) {
        ROS_INFO ( "[ROBOT] I'm goin right! \n" );
        motor_command.linear.x = 0.25;
        motor_command.angular.z = -0.25;
    }
    else if ( move_type == GO_LEFT ) {
        ROS_INFO ( "[ROBOT] I'm goin left! \n" );
        motor_command.linear.x = 0.25;
        motor_command.angular.z = 0.25;
    }
    else {
        ROS_INFO ( "[ROBOT_MOVE] Move type wrong! \n" );
        return false;
    }

    //let's publish that command so that the robot follows it
    motor_command_publisher.publish ( motor_command );
    usleep(10);
    return true;
}

bool following_wall = false;
bool thats_a_door = false;
bool crashed = false;

void laser_callback ( const sensor_msgs::LaserScan::ConstPtr &scan_msg ) {
    
    laser_msg = *scan_msg;
    //data structure containing the command to drive the robot

    //Alternatively we could have looked at the laser scan BEFORE we made this decision.
    //Well, let's see how we might use a laser scan.
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();    
    float left_side = 0.0, right_side = 0.0;
    float range_min = laser_msg.range_max, range_max = laser_msg.range_min;
    int nan_count = 0;
    for(size_t i = 0; i < range_size; i++){
        if (laser_ranges[i] < range_min){
            range_min = laser_ranges[i];
        }
        
        if (std::isnan(laser_ranges[i])){
            nan_count++;
        }
        if (i < range_size / 4 ) {
            if (laser_ranges[i] > range_max){
                range_max = laser_ranges[i];
            }
        }
        
        if (i > range_size / 2){
            left_side += laser_ranges[i];
        }
        else {
            right_side += laser_ranges[i];
        }
    }
    
    if (nan_count > (range_size * 0.9) || laser_ranges[range_size / 2] < 0.25) {
        crashed = true;
    }
    else {
        crashed = false;
    }
    if (!crashed) {
        
        if (range_min <= 0.5 && !thats_a_door){
            following_wall = true;
            crashed = false;
            robot_move(STOP);
            
            if (left_side >= right_side) {
                robot_move(TURN_RIGHT);
            }
            else {
                robot_move(TURN_LEFT);  
            }
        }
        else {
            ROS_INFO("[ROBOT] Dam son: %f , %d \n", range_max, following_wall );
            robot_move(STOP);
            if ( following_wall ) {
                if (range_max >= 2.0){
                    thats_a_door = true;
                    following_wall = false;
                    //robot_move(TURN_RIGHT);
                    ROS_INFO("[ROBOT] I am following wall and my max range > 2.0 Range Max: %f \n",range_max );
                }
            } 
            if (thats_a_door) {
                if (laser_ranges[0] <= 0.5){
                    thats_a_door = false;
                }
                else {
                    robot_move(GO_RIGHT);  
                }
                ROS_INFO("[ROBOT] I am goin' right!: %d \n", thats_a_door );
                
            }
            else {
                robot_move(FORWARD);  
            }
        }
    }
    else {
        robot_move(BACKWARD);
    }
    //the laser scan is an array (vector) of distances.
    std::cout<<"Number of points in laser scan is: "<<laser_ranges.size()<<std::endl;
    std::cout<<"The distance to the rightmost scanned point is "<<laser_ranges[0]<<std::endl;
    std::cout<<"The distance to the leftmost scanned point is "<<laser_ranges[laser_ranges.size()-1]<<std::endl;
    std::cout<<"The distance to the middle scanned point is "<<laser_ranges[laser_ranges.size()/2]<<std::endl;

    //You can use basic trignometry with the above scan array and the following information to find out exactly where the laser scan found something:
    std::cout<<"The minimum angle scanned by the laser is "<<laser_msg.angle_min<<std::endl;
    std::cout<<"The maximum angle scanned by the laser is "<<laser_msg.angle_max<<std::endl;
    std::cout<<"The increment in angles scanned by the laser is "<<laser_msg.angle_increment<<std::endl; //should be that angle_min+angle_increment*laser_ranges.size() is about angle_max
    std::cout<<"The minimum range (distance) the laser can perceive is "<<laser_msg.range_min<<std::endl;
    std::cout<<"The maximum range (distance) the laser can perceive is "<<laser_msg.range_max<<std::endl;

}
//
// --------------------------------------------------------------------------------------------
//

//You can also make use of the map which is being built by the "gslam_mapping" subsystem
//There is some code here to help but you can understand the API also
// by looking up the OccupancyGrid message and its members (this is the API for the message).
//If you want me to explain the data structure I will - just ask me in advance of class.
void map_callback ( const nav_msgs::OccupancyGrid::ConstPtr &msg ) {

    const bool chatty_map = true;

    map_msg = *msg;

    double map_width = map_msg.info.width;
    double map_height = map_msg.info.width;

    double map_origin_x = map_msg.info.origin.position.x;
    double map_origin_y = map_msg.info.origin.position.y;
    double map_orientation = acos ( map_msg.info.origin.orientation.z );

    std::vector<signed char > map = map_msg.data;
    
    

//     if ( chatty_map ) std::cout << "------MAP:------" << std::endl;
// 
//     for ( unsigned int x = 0; x < map_width; x++ ) {
//         for ( unsigned int y = 0; y < map_height; y++ ) {
// 
//             unsigned int index = x + y * map_width;
//             if (index > 0 && index < map.size()) {
//                 if ( map[index] > 50 ) { // 0 – 100 represents how occupied
//                     //this square is occupied
//                 } else if ( map[index] >= 0 ) {
//                     //this square is unoccupied
//                     if (x > map_width / 2){
//                    
//                 } else {
//                     //this square is unknown
//                 }
//             }
//         }
//         if ( chatty_map ) std::cout << std::endl;
//     }
//     if ( chatty_map ) std::cout << "----------------" << std::endl;
//     }

}

int main ( int argc, char **argv ) {
    // must always do this when starting a ROS node - and it should be the first thing to happen
    ros::init ( argc, argv, "amble" );
    // the NodeHandle object is our access point to ROS
    ros::NodeHandle n;

    // Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi
    motor_command_publisher = n.advertise<geometry_msgs::Twist> ( "/cmd_vel_mux/input/navi", 100 );

    // Here we set the function laser_callback to receive new laser messages when they arrive
    laser_subscriber = n.subscribe ( "/scan", 1000, laser_callback );
    // Here we set the function map_callback to receive new map messages when they arrive from the mapping subsystem
    map_subscriber = n.subscribe ( "/map", 1000, map_callback );

    //now enter an infinite loop - the callback functions above will be called when new laser or map messages arrive.
    ros::Duration time_between_ros_wakeups ( 0.001 );
    while ( ros::ok() ) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }
    return 0;
}
