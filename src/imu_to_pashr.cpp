#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nmea_msgs/Sentence.h>
#include <tf2/utils.h>

ros::Publisher nmea_publisher;

void imuCallback(const sensor_msgs::Imu::ConstPtr &message)
{
  std::stringstream nmea;
  nmea << "$PASHR,";

  // timestamp

  auto time_seconds = static_cast<time_t>(message->header.stamp.sec);
  
  tm time_struct;
  gmtime_r(&time_seconds, &time_struct);

  nmea << std::setfill('0') << std::setw(2) << time_struct.tm_hour;
  nmea << std::setfill('0') << std::setw(2) << time_struct.tm_min;
  nmea << std::setfill('0') << std::setw(2) << time_struct.tm_sec;

  int milliseconds = message->header.stamp.nsec / 1000000;

  nmea << "." << std::setfill('0') << std::setw(3) << milliseconds << ",";

  double yaw, pitch, roll;

  tf2::getEulerYPR(message->orientation, yaw, pitch, roll);

  // heading

  auto heading = 90.0 - (180.0*yaw/M_PI);
  if (heading < 0.0)
    heading += 360.0;
  nmea << heading;

  nmea << ",T,";

  // roll

  if (roll >= 0.0)
    nmea << "+";
  nmea << 180.0*roll/M_PI << ",";

  // pitch

  pitch = -pitch;

  if (pitch >= 0.0)
    nmea << "+";
  nmea << 180.0*pitch/M_PI << ",";

  // heave

  nmea << "0.0,";

  // roll accuracy (std dev)

  nmea << 180.0*sqrt(message->orientation_covariance[0])/M_PI << ",";

  // pitch accuracy (std dev)
  
  nmea << 180.0*sqrt(message->orientation_covariance[4])/M_PI << ",";

  // heading accuracy (std dev)
  
  nmea << 180.0*sqrt(message->orientation_covariance[8])/M_PI << ",";

  // GPS Update Quality Flag

  nmea << "1,"; // 1 = non-rtk

  /// INS Status Flag

  nmea << "1"; // 1 = span post alignment

  nmea << "\r\n";

  nmea_msgs::Sentence sentence;
  sentence.header = message->header;
  sentence.sentence = nmea.str();

  nmea_publisher.publish(sentence);

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "imu_to_pashr");

  ros::NodeHandle nh;

  nmea_publisher = nh.advertise<nmea_msgs::Sentence>("nmea", 10);

  ros::Subscriber imu_subscriber = nh.subscribe("input", 10, &imuCallback);

  ros::spin();
  return 0;
}

