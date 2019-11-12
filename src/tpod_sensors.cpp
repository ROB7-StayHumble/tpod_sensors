#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
//#include "tpod_sensors/IMU.h"
#include "port.h"
#include "mahony_ahrs.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/Imu.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <sstream>
#include <stdint.h>
#include <string>

#define SENSOR_RATE 50
#define BUFSIZE 2048
#define IMU_MESSAGE_SIZE 9

int main(int argc, char **argv){
  ros::init(argc,argv,"imu_node");
  ros::NodeHandle n;
  ros::Rate tpod_sensors(SENSOR_RATE);
  ros::Publisher tpod_imu_pub = n.advertise<sensor_msgs::Imu>("imu_data",1);

  //Used for measuring loop time
  double current_time = ros::Time::now().toSec();
  double start_time =current_time;

  // setup transform broardcast
  //static tf::TransformBroadcaster br;
  //tf::Transform transform;
  //transform.setOrigin(tf::Vector3(0.0,0.0,0.0));

  // prepare ahrs filter object
  MahonyAHRS ahrs_filetr(10,5,20); // magic numbers that i dont know for sure what does yet. to-do

  //udp stuff
  struct sockaddr_in locaddr;  /* our address */
  struct sockaddr_in remaddr; /* remote address */
  socklen_t addrlen = sizeof(remaddr);    /* length of addresses */
  int recvlen;      /* # bytes received */
  int fd;       /* our socket */
  unsigned char buf[BUFSIZE]; /* receive buffer */

  /* create a UDP socket */
  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      perror("cannot create socket\n");
      return 0;
  }

  /* bind the socket to any valid IP address and a specific port */
  memset((char *)&locaddr, 0, sizeof(locaddr));
  locaddr.sin_family = AF_INET;
  locaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  locaddr.sin_port = htons(SERVICE_PORT);
  if (bind(fd, (struct sockaddr *)&locaddr, sizeof(locaddr)) < 0) {
    perror("bind failed");
    return 0;
  }
  ROS_INFO("bind successful \n");

  //////////////////////////////////

  while(ros::ok()){
    ros::spinOnce();//gets the newest values
    recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
    std::string imu_arr[IMU_MESSAGE_SIZE];

    // go thrugh the data if there is something
    if (recvlen > 0) {
      buf[recvlen] = 0;
      //prepare a string message
      std::string msg;
      msg.append(reinterpret_cast<const char*> (buf));

      // if it does not stat with $ because then it is gps data
      // go thrugh the string and seperate it at each comma
      if ( msg.at(0) != '$' ){
        int i = 0;
        //std::cout<<msg<<std::endl;
        for(int j = 0; j < msg.length(); j++){
          if( msg.at(j) != ',' ){
            imu_arr[i].push_back( msg.at(j)) ;
          }
          else{
            ++i;
          }
        }
      }
    } // if(recvlen > 0)

    // convert strings to floats
    float imu_floats[IMU_MESSAGE_SIZE];
    for (int i = 0; i < IMU_MESSAGE_SIZE; i++){
      try{
        imu_floats[i] = stod(imu_arr[i]);
      }
      catch (const std::invalid_argument & ia){ // error handeling
        ROS_INFO("conversion error on data stream. Data ignored");
        break;
      }
    }

    //update the filter with the newest data.
    ahrs_filetr.MahonyAHRSupdate(imu_floats[3]*3.1415/180,imu_floats[4]*3.1415/180,imu_floats[5]*3.1415/180,  // gyry likely need the gyro needs *3.1415/180,
                                 imu_floats[0],imu_floats[1],imu_floats[2],  // lin acceleration
                                 imu_floats[6],imu_floats[7],imu_floats[8]); // magnetrometer

    tf::Quaternion my_orientation(ahrs_filetr.getQuaternions(1),ahrs_filetr.getQuaternions(2),ahrs_filetr.getQuaternions(3),ahrs_filetr.getQuaternions(0));
    //transform.setRotation(my_orientation);
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link","IMU"));


    // create a geometry_twist message
    geometry_msgs::Quaternion q_geometry;
    tf::quaternionTFToMsg(my_orientation,q_geometry);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_link";

    imu_msg.orientation = q_geometry;
    geometry_msgs::Vector3 angular_vel;
    angular_vel.x = imu_floats[3]*3.1415/180;
    angular_vel.y = imu_floats[4]*3.1415/180;
    angular_vel.z = imu_floats[5]*3.1415/180;
    imu_msg.angular_velocity = angular_vel;
    geometry_msgs::Vector3 linear_acc;
    linear_acc.x = imu_floats[0];
    linear_acc.y = imu_floats[1];
    linear_acc.z = imu_floats[2];
    imu_msg.linear_acceleration = linear_acc;

    tpod_imu_pub.publish(imu_msg);


    tpod_sensors.sleep();//Ensures the timing of the loop
  }// if (ros::ok)
  return 0;
}//shutdown;

