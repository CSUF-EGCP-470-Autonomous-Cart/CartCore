#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <nav_msgs/Odometry.h>

void print(ros::console::Level level, const std::string& s) {
	ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME, "%s", s.c_str());
}

class DriveSystem {
	public:
		DriveSystem();

	private:
		void teleopCallback(const geometry_msgs::Twist::ConstPtr& twist);
		void safetyCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& status);
		ros::NodeHandle nh_;

		ros::Publisher odom_pub_;
		ros::Subscriber teleop_sub_;
		ros::Subscriber safety_sub_;
};

DriveSystem::DriveSystem() {
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/cart_odom", 1);
	teleop_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &DriveSystem::teleopCallback, this);
	safety_sub_ = nh_.subscribe<>("safety_system", 10, &DriveSystem::safetyCallback, this);
}

void DriveSystem::teleopCallback(const geometry_msgs::Twist::ConstPtr& twist) {

}

void DriveSystem::safetyCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& status) {

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "drive_system");
	DriveSystem drive_system;

	ros::spin();

	return 0;
}
