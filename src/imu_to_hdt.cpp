#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nmea_msgs/Sentence.h>
#include <tf2/utils.h>

ros::Publisher nmea_publisher;

void imuCallback(const sensor_msgs::Imu::ConstPtr &message)
{
  std::stringstream nmea;
  nmea << "$GPHDT,";

  auto yaw = tf2::getYaw(message->orientation);

  auto heading = 90.0 - (180.0*yaw/M_PI);
  nmea << heading;

  nmea << ",T\r\n";
  
  nmea_msgs::Sentence sentence;
  sentence.header = message->header;
  sentence.sentence = nmea.str();

  nmea_publisher.publish(sentence);

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "imu_to_hdt");

  ros::NodeHandle nh;

  nmea_publisher = nh.advertise<nmea_msgs::Sentence>("nmea", 10);

  ros::Subscriber imu_subscriber = nh.subscribe("input", 10, &imuCallback);

  ros::spin();
  return 0;
}

