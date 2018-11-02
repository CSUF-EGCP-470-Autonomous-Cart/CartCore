#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <ros/console.h>
#include <std_srvs/Trigger.h>
#include <vector>

void print(ros::console::Level level, const std::string& s) {
	ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME, "%s", s.c_str());
}

class BatteryManagementSystem {
	public:
		BatteryManagementSystem();
		void update();
	private:
		ros::NodeHandle nh_;
		ros::Publisher batteryStatePublisher;
		ros::ServiceClient safetySystemClient;

		int NUM_CELLS;

		double BANK_VOLTAGE_MIN;
		double BANK_VOLTAGE_LOW;
		double BANK_VOLTAGE_MAX;
		double CAPACITY;

		int CELLS;

		double CELL_VOLTAGE_MIN;
		double CELL_VOLTAGE_LOW;
		double CELL_VOLTAGE_MAX;

		double CELL_TEMP_MIN;
		double CELL_TEMP_MAX;

};

BatteryManagementSystem::BatteryManagementSystem():
	BANK_VOLTAGE_MIN(0.0),
	BANK_VOLTAGE_LOW(0.0),
	BANK_VOLTAGE_MAX(0.0),
	CAPACITY(NAN),
	CELL_VOLTAGE_MIN(0.0),
	CELL_VOLTAGE_LOW(0.0),
	CELL_VOLTAGE_MAX(0.0),
	CELL_TEMP_MIN(0.0),
	CELL_TEMP_MAX(0.0),
	CELLS(1)
{

	nh_.param("bank_voltage_min", BANK_VOLTAGE_MIN, BANK_VOLTAGE_MIN);
	nh_.param("bank_voltage_low", BANK_VOLTAGE_LOW, BANK_VOLTAGE_LOW);
	nh_.param("bank_voltage_max", BANK_VOLTAGE_MAX, BANK_VOLTAGE_MAX);
	nh_.param("capacity", CAPACITY, CAPACITY);
	nh_.param("cells", CELLS, CELLS);
	nh_.param("cell_voltage_min", CELL_VOLTAGE_MIN, CELL_VOLTAGE_MIN);
	nh_.param("cell_voltage_low", CELL_VOLTAGE_LOW, CELL_VOLTAGE_LOW);
	nh_.param("cell_voltage_max", CELL_VOLTAGE_MAX, CELL_VOLTAGE_MAX);
	nh_.param("cell_temp_min", CELL_TEMP_MIN, CELL_TEMP_MIN);
	nh_.param("cell_temp_max", CELL_TEMP_MAX, CELL_TEMP_MAX);

	batteryStatePublisher = nh_.advertise<sensor_msgs::BatteryState>("battery_state", 1);
	safetySystemClient = nh_.serviceClient<std_srvs::Trigger>("digital_estop");
}

void BatteryManagementSystem::update() {
	//TODO Get battery data from arduino

	double cellVoltages[] = {
		8.0, 8.0, 8.0, 8.0, 8.0, 8.0
	};
	double cellTemps[] ={
		30.0, 30.0, 30.0, 30.0, 30.0, 30.0
	};

	double bankVoltage = 46.0;
	double bankCurrent = 0.1;

	sensor_msgs::BatteryState battery;
	battery.power_supply_technology = 7;
	battery.voltage = bankVoltage;
	battery.current = bankCurrent;
	battery.present = true;
	battery.capacity = CAPACITY;
	battery.charge = NAN;
	battery.design_capacity = NAN;
	battery.percentage = NAN;
	battery.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
//	battery.cell_voltage = cellVoltages;
	battery.location = "Under the seat";
	battery.serial_number = "";


	if(bankVoltage <= 0.0){
		ROS_LOG_STREAM(ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "Battery not present");
		battery.present = false;
	}
	else if (bankVoltage <= BANK_VOLTAGE_MIN) {
		ROS_LOG_STREAM(ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "Battery bank voltage below minimum: " << bankVoltage << "V <= " << BANK_VOLTAGE_MIN << "V");
	}
	else if(bankVoltage <= BANK_VOLTAGE_LOW) {
		ROS_LOG_STREAM(ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, "Battery bank voltage low: " << bankVoltage << "V <= " << BANK_VOLTAGE_LOW << "V");
	}
	else if(bankVoltage >= BANK_VOLTAGE_MAX) {
		ROS_LOG_STREAM(ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "Battery bank voltage above max: " << bankVoltage << "V >= " << BANK_VOLTAGE_MAX << "V");
	}
	else {
		ROS_LOG_STREAM(ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, "Battery bank voltage OK: " << bankVoltage);
	}

	bool cellOverheating = false;
	bool cellCold = false;
//	battery.cell_voltage = new float[CELLS];
	std::vector<float> cells;
	for(int i=0; i<NUM_CELLS;++i) {
//		battery.cell_voltage[i] = (float)cellVoltages[i];

		if(cellTemps[i] > CELL_TEMP_MAX) {
			ROS_LOG_STREAM(ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "Battery cell " << i << " temperature high: " << cellTemps[i] << "C > " << CELL_TEMP_MAX << "C");
			cellOverheating = true;
		}

		if(cellTemps[i] < CELL_TEMP_MIN) {
			ROS_LOG_STREAM(ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "Battery cell " << i << " temperature low: " << cellTemps[i] << "C < " << CELL_TEMP_MIN << "C");
			cellCold = true;
		}


		if (cellVoltages[i] <= CELL_VOLTAGE_MIN) {
			ROS_LOG_STREAM(ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "Battery cell " << i << " voltage below minimum: " << cellVoltages[i] << "V <= " << CELL_VOLTAGE_MIN << "V");
		}
		else if(cellVoltages[i] <= CELL_VOLTAGE_LOW) {
			ROS_LOG_STREAM(ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, "Battery cell " << i << " voltage low: " << cellVoltages[i] << "V <= " << CELL_VOLTAGE_LOW << "V");
		}
		else if(cellVoltages[i] >= CELL_VOLTAGE_MAX) {
			ROS_LOG_STREAM(ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME,  "Battery cell " << i << " voltage above max: " << cellVoltages[i] << "V >= " << CELL_VOLTAGE_MAX << "V");
		}
		else {
			ROS_LOG_STREAM(ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, "Battery cell " << i << " voltage OK: " << cellVoltages[i] << "V");
		}
	}


	battery.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
	if(cellOverheating) {
		battery.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	else if(cellCold) {
		battery.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_COLD;
	}
	else if (bankVoltage <= BANK_VOLTAGE_MIN) {
		battery.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
	}
	else if(bankVoltage >= BANK_VOLTAGE_MAX) {
		battery.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}

	if(battery.power_supply_health != sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD) {
/*		if(safetySystemClient.call()) {
			print();
		}
		else {
			print();
		}
*/
	}


	batteryStatePublisher.publish(battery);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "battery_management_system");
	BatteryManagementSystem battery_management_system;

	ros::Rate loop_rate(1);

	while(ros::ok()) {
		battery_management_system.update();

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
