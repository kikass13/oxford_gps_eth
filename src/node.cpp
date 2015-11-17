#include <ros/ros.h>

// ROS messages
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

// Tf Quaternions
#include <tf/LinearMath/Quaternion.h>

// Packet structure
#include "dispatch.h"

// Ethernet
#include <arpa/inet.h>

static inline bool openSocket(const std::string &interface, const std::string &ip_addr, uint16_t port, int *fd_ptr, sockaddr_in *sock_ptr)
{
  // Create UDP socket
  int fd;
  fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd != -1) {
    if (interface.length()) {
      if (!setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(), interface.length()) == 0) {
        close(fd);
        return false;
      }
    }
    memset(sock_ptr, 0, sizeof(sockaddr_in));
    sock_ptr->sin_family = AF_INET;
    sock_ptr->sin_port = htons(port);
    if (!inet_aton(ip_addr.c_str(), &sock_ptr->sin_addr)) {
      sock_ptr->sin_addr.s_addr = INADDR_BROADCAST; // Invalid address, use BROADCAST
    }
    if (bind(fd, (sockaddr*)sock_ptr, sizeof(sockaddr)) == 0) {
      *fd_ptr = fd;
      return true;
    }
  }
  return false;
}

static inline int readSocket(int fd, unsigned int timeout, void *data, int size, sockaddr *source_ptr = NULL)
{
  if (fd >= 0) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    // Set up timeout
    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout * 1000) % 1000000;

    if (select(fd + 1, &fds, NULL, NULL, &tv) > 0) {
      socklen_t socklen = source_ptr ? sizeof(*source_ptr) : 0;
      socklen_t *socklen_ptr = source_ptr ? &socklen : NULL;
      return recvfrom(fd, data, size, 0, source_ptr, socklen_ptr);
    }

    // Timeout
    return 0;
  }
  return -1;
}

static inline void handlePacket(const Packet *packet, ros::Publisher &pub_fix, ros::Publisher &pub_vel,
                                ros::Publisher &pub_imu, const std::string &frame_id)
{
  static uint8_t fix_status = sensor_msgs::NavSatStatus::STATUS_FIX;
  static uint8_t position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  static double position_covariance[9];
  if (packet->nav_status == 4) {
    ros::Time stamp = ros::Time::now();

    switch (packet->channel) {
      case 0:
        switch (packet->chan.chan0.position_mode) {
          case MODE_DIFFERENTIAL:
          case MODE_DIFFERENTIAL_PP:
            fix_status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            break;
          case MODE_OMNISTAR_VBS:
          case MODE_OMNISTAR_HP:
          case MODE_OMNISTAR_XP:
          case MODE_WAAS:
          case MODE_CDGPS:
            fix_status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
            break;
          case MODE_SPS:
            fix_status = sensor_msgs::NavSatStatus::STATUS_FIX;
            break;
          case MODE_NONE:
          case MODE_SEARCH:
          case MODE_DOPLER:
          case MODE_RTK_FLOAT:
          case MODE_RTK_INTEGER:
          case MODE_NO_DATA:
          case MODE_BLANKED:
          case MODE_DOPLER_PP:
          case MODE_SPS_PP:
          case MODE_RTK_FLOAT_PP:
          case MODE_RTK_INTEGER_PP:
          case MODE_NOT_RECOGNISED:
          case MODE_UNKNOWN:
          default:
            fix_status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            break;
        }
#if 0
        ROS_INFO("Num Sats: %u, Position mode: %u, Velocity mode: %u, Orientation mode: %u",
                 packet->chan.chan0.num_sats,
                 packet->chan.chan0.position_mode,
                 packet->chan.chan0.velocity_mode,
                 packet->chan.chan0.orientation_mode);
#endif
        break;
      case 3:
        if (packet->chan.chan3.age < 150) {
          position_covariance[0] = (double)packet->chan.chan3.acc_position_east * 1e-3;
          position_covariance[1] = 0;
          position_covariance[2] = 0;
          position_covariance[3] = 0;
          position_covariance[4] = (double)packet->chan.chan3.acc_position_north * 1e-3;
          position_covariance[5] = 0;
          position_covariance[6] = 0;
          position_covariance[7] = 0;
          position_covariance[8] = (double)packet->chan.chan3.acc_position_down * 1e-3;
          position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
#if 0
          ROS_INFO("Position accuracy: North: %umm, East: %umm, Down: %umm",
                   packet->chan.chan3.acc_position_north,
                   packet->chan.chan3.acc_position_east,
                   packet->chan.chan3.acc_position_down);
#endif
        } else {
          position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        }
        break;
      case 4:
#if 0
        if (packet->chan.chan4.age < 150) {
          ROS_INFO("Velocity accuracy: North: %umm/s, East: %umm/s, Down: %umm/s",
                   packet->chan.chan4.acc_velocity_north,
                   packet->chan.chan4.acc_velocity_east,
                   packet->chan.chan4.acc_velocity_down);
        }
#endif
        break;

      case 5:
#if 0
        if (packet->chan.chan5.age < 150) {
          ROS_INFO("Velocity accuracy: Heading: %frad, Pitch: %frad, Roll: %frad",
                   (double)packet->chan.chan5.acc_heading * 1e-5,
                   (double)packet->chan.chan5.acc_pitch * 1e-5,
                   (double)packet->chan.chan5.acc_roll * 1e-5);
        }
#endif
        break;
      case 23:
#if 0
        ROS_INFO("Delay: %ums", packet->chan.chan23.delay_ms);
#endif
        break;
      case 27:
#if 0
        ROS_INFO("Heading quality: %u", packet->chan.chan27.heading_quality);
#endif
        break;
      case 37:
#if 0
        if (packet->chan.chan37.valid) {
          ROS_INFO("Heading Misaligment: Angle: %frad, Accuracy: %frad",
                   (double)packet->chan.chan37.heading_misalignment_angle * 1e-4,
                   (double)packet->chan.chan37.heading_misalignment_angle * 1e-4);
        }
#endif
        break;
      case 48:
#if 0
        ROS_INFO("HDOP: %0.1f, PDOP: %0.1f",
                 (double)packet->chan.chan48.HDOP * 1e-1,
                 (double)packet->chan.chan48.PDOP * 1e-1);
#endif
        break;
    }

    sensor_msgs::NavSatFix msg_fix;
    msg_fix.header.stamp = stamp;
    msg_fix.header.frame_id = frame_id;
    msg_fix.latitude = packet->latitude * (180 / M_PI);
    msg_fix.longitude = packet->longitude * (180 / M_PI);
    msg_fix.altitude = packet->altitude;
    msg_fix.status.status = fix_status;
    msg_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    msg_fix.position_covariance_type = position_covariance_type;
    if (position_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      for (unsigned int i = 0; i < 9; i++) {
        msg_fix.position_covariance[i] = position_covariance[i];
      }
    }
    pub_fix.publish(msg_fix);

    geometry_msgs::TwistStamped msg_vel;
    msg_vel.header.stamp = stamp;
    msg_vel.header.frame_id = frame_id;
    msg_vel.twist.linear.x = (double)packet->vel_east * 1e-4;
    msg_vel.twist.linear.y = (double)packet->vel_north * 1e-4;
    msg_vel.twist.linear.z = (double)packet->vel_down * -1e-4;
    pub_vel.publish(msg_vel);

    tf::Quaternion q;
    q.setRPY((double)packet->roll * 1e-6, (double)packet->pitch * 1e-6, (double)packet->heading * 1e-6);
    sensor_msgs::Imu msg_imu;
    msg_imu.header.stamp = stamp;
    msg_imu.header.frame_id = frame_id;
    msg_imu.linear_acceleration.x = (double)packet->accel_x * 1e-4;
    msg_imu.linear_acceleration.y = (double)packet->accel_y * 1e-4;
    msg_imu.linear_acceleration.z = (double)packet->accel_z * -1e-4;
    msg_imu.linear_acceleration_covariance[0] = -1;
    msg_imu.angular_velocity.x = (double)packet->gyro_x * 1e-5;
    msg_imu.angular_velocity.y = (double)packet->gyro_y * 1e-5;
    msg_imu.angular_velocity.z = (double)packet->gyro_z * -1e-5;
    msg_imu.angular_velocity_covariance[0] = -1;
    msg_imu.orientation.w = q.w();
    msg_imu.orientation.x = q.x();
    msg_imu.orientation.y = q.y();
    msg_imu.orientation.z = q.z();
    msg_imu.orientation_covariance[0] = 0.0174532925;
    msg_imu.orientation_covariance[1] = 0;
    msg_imu.orientation_covariance[2] = 0;
    msg_imu.orientation_covariance[3] = 0;
    msg_imu.orientation_covariance[4] = 0.0174532925;
    msg_imu.orientation_covariance[5] = 0;
    msg_imu.orientation_covariance[6] = 0;
    msg_imu.orientation_covariance[7] = 0;
    msg_imu.orientation_covariance[8] = 0.0174532925;
    msg_imu.angular_velocity_covariance[0] = 0.000436332313;
    msg_imu.angular_velocity_covariance[1] = 0;
    msg_imu.angular_velocity_covariance[2] = 0;
    msg_imu.angular_velocity_covariance[3] = 0;
    msg_imu.angular_velocity_covariance[4] = 0.000436332313;
    msg_imu.angular_velocity_covariance[5] = 0;
    msg_imu.angular_velocity_covariance[6] = 0;
    msg_imu.angular_velocity_covariance[7] = 0;
    msg_imu.angular_velocity_covariance[8] = 0.000436332313;
    msg_imu.linear_acceleration_covariance[0] = 0.0004;
    msg_imu.linear_acceleration_covariance[1] = 0;
    msg_imu.linear_acceleration_covariance[2] = 0;
    msg_imu.linear_acceleration_covariance[3] = 0;
    msg_imu.linear_acceleration_covariance[4] = 0.0004;
    msg_imu.linear_acceleration_covariance[5] = 0;
    msg_imu.linear_acceleration_covariance[6] = 0;
    msg_imu.linear_acceleration_covariance[7] = 0;
    msg_imu.linear_acceleration_covariance[8] = 0.0004;
    pub_imu.publish(msg_imu);
#if 0
  } else {
    ROS_WARN("Nav Status: %u", packet->nav_status);
#endif
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oxford_gps");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  std::string interface = "";
  std::string ip_addr = "";
  int port = 3000;
  std::string frame_id = "gps";
  priv_nh.getParam("interface", interface);
  priv_nh.getParam("ip_address", ip_addr);
  priv_nh.getParam("port", port);
  priv_nh.getParam("frame_id", frame_id);

  if (port > UINT16_MAX) {
    ROS_ERROR("Port %u greater than maximum value of %u", port, UINT16_MAX);
  }

  if (interface.length() && ip_addr.length()) {
    ROS_INFO("Preparing to listen on interface %s port %u for packets from ip %s", interface.c_str(), port, ip_addr.c_str());
  } else if (interface.length()) {
    ROS_INFO("Preparing to listen on interface %s port %u", interface.c_str(), port);
  } else if (ip_addr.length()) {
    ROS_INFO("Preparing to listen on port %u for packets from ip %s", port, ip_addr.c_str());
  } else {
    ROS_INFO("Preparing to listen on port %u", port);
  }

  int fd;
  sockaddr_in sock;
  if (openSocket(interface, ip_addr, port, &fd, &sock)) {
    // Set up Publishers
    ros::Publisher pub_fix = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 2);
    ros::Publisher pub_vel = node.advertise<geometry_msgs::TwistStamped>("gps/vel", 2);
    ros::Publisher pub_imu = node.advertise<sensor_msgs::Imu>("imu/data", 2);

    // Variables
    Packet packet;
    sockaddr source;
    bool first = true;

    // Loop until shutdown
    while (ros::ok()) {
      if (readSocket(fd, 10, &packet, sizeof(packet), &source) >= sizeof(packet)) {
        if (validatePacket(&packet)) {
          if (first) {
            first = false;
            ROS_INFO("Connected to Oxford GPS at %s:%u", inet_ntoa(((sockaddr_in*)&source)->sin_addr), htons(((sockaddr_in*)&source)->sin_port));
          }
          handlePacket(&packet, pub_fix, pub_vel, pub_imu, frame_id);
        }
      }

      // handle callbacks
      ros::spinOnce();
    }

    // Close socket
    close(fd);
  } else {
    ROS_FATAL("Failed to open socket");
    ros::WallDuration(1.0).sleep();
  }

  return 0;
}
