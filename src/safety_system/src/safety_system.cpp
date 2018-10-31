#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/console.h>
#include <std_srvs/Trigger.h>

void print(ros::console::Level level, const std::string& s) {
        ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME, "%s", s.c_str());
}

class SafetySystem {
	public:
		SafetySystem();
		void update();

	private:
		bool digitalEStopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
		ros::NodeHandle nh_;

		ros::Publisher status_pub_;

		ros::ServiceServer digital_estop_service_;
};

SafetySystem::SafetySystem() {
	status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/safety_system", 1);

	digital_estop_service_ = nh_.advertiseService("digital_estop", &SafetySystem::digitalEStopServiceCallback, this);
}

void SafetySystem::update() {
	diagnostic_msgs::DiagnosticStatus status;
	status.level = diagnostic_msgs::DiagnosticStatus::OK;
	status.name = "ESTOP";
	status.message = "OK";
	status.hardware_id = "ESTOP";
	status_pub_.publish(status);

	print(ros::console::Level::Info, "ESTOP OK");
}

bool SafetySystem::digitalEStopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
	response.success = true;
	response.message = "hi";
	return 0;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "safety_system");
	SafetySystem safety_system;

	ros::Rate loop_rate(10);

	while(ros::ok()) {
		safety_system.update();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
