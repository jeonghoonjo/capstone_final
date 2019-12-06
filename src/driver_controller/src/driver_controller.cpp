#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

tf::Transform transform;
tf::Quaternion q;

void poseCb(const geometry_msgs::PoseStampedPtr &pose)
{
   static tf::TransformBroadcaster br;
   q.setX(pose->pose.orientation.x);
   q.setY(pose->pose.orientation.y);
   q.setZ(pose->pose.orientation.z);
   q.setW(pose->pose.orientation.w);

   transform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0));
   transform.setRotation(q);

   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_controller");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe("/pose", 1, poseCb);
  ros::Rate loop_rate(100);

  while(ros::ok())
  {
  	ros::spinOnce();
 	loop_rate.sleep();
  }

}
