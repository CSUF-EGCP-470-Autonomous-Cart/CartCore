#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

class BatteryManagementSystem {
	public:
		BatteryManagementSystem();

		void update();
	private:
		ros::Publisher battery_state_pub_;

		ros::ServiceClient safety_system_client_;

};

