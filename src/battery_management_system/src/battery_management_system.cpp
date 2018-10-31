#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

class BatteryManagementSystem {
	public:
		BatteryManagementSystem();
		void update();
	private:
		ros::NodeHandler nh_;
		ros::Publisher batteryStatePublisher;
		ros::ServiceClient safetySystemClient;
		
		double BANK_VOLTAGE_MIN;
		double BANK_VOLTAGE_LOW;
		double BANK_VOLTAGE_MAX;
		
		double CELL_VOLTAGE_MIN;
		double CELL_VOLTAGE_LOW;
		double CELL_VOLTAGE_MAX;
		
		double CELL_TEMP_MIN;
		double CELL_TEMP_MAX;
		
		
	
};

BattertManagementSystem::BatteryManagementSystem{
	batteryStatePublisher = nh_.advertise<sensor_msgs::BatteryState>("battery_state", 1);
	safetySystemClient = nh_.serviceClient<>("");
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
	battery.cellVoltage = cellVoltages;
	
	
	if(bankVoltage <= 0.0){
		print();
		battery.present = false;
	}
	else if (bankVoltage <= BANK_VOLTAGE_MIN) {
		print(Fatal, "Battery bank voltage below minimum: %d <= %d", bankVoltage, BANK_VOLTAGE_MIN);
	}
	else if(bankVoltage <= BANK_VOLTAGE_LOW) {
		print(Warn, "Battert bank voltage low: %d <= %d", bankVoltage, BANK_VOLTAGE_LOW);
	}
	else if(bankVoltage >= BANK_VOLTAGE_MAX) {
		print(Fatal, "Battery bank voltage above max: %d >= %d", bankVoltage, BANK_VOLTAGE_MAX);
	}
	else {
		print(Info, "Battery bank voltage OK: %d", bankVoltage);
	}
	
	bool cellOverheating = alse, cellCold=false;
	for(int i=0; i<NUM_CELLS;++i) {
		if(cellTemps[i] > CELL_TEMP_MAX) {
			print(Fatal, "Battery cell %i temperature high: %dC > %dC", i, cellTemps[i], CELL_TEMP_MAX);
			cellOverheating = true;
		}
			
		if(cellTemps[i] < CELL_TEMP_MIN) {
			print(Fatal, "Battery cell %i temperature low: %dC < %dC", i, cellTemps[i], CELL_TEMP_MIN);
			cellCold = true;
		}
			
		
		if (cellVoltages[i] <= CELL_VOLTAGE_MIN) {
			print(Fatal, "Battery cell %i voltage below minimum: %d <= %d", i, bankVoltage, BANK_VOLTAGE_MIN);
		}
		else if(cellVoltages[i] <= CELL_VOLTAGE_LOW) {
			print(Warn, "Battery cell %i voltage low: %d <= %d", i, bankVoltage, BANK_VOLTAGE_LOW);
		}
		else if(cellVoltages[i] >= CELL_VOLTAGE_MAX) {
			print(Fatal, "Battery cell %i voltage above max: %d >= %d", i, bankVoltage, BANK_VOLTAGE_MAX);
		}
		else {
			print(Info, "Battery cell %i voltage OK: %d", i, bankVoltage);
		}
	}
	
	
	battery.power_supply_health = sensor_msgs::BatteryState.POWER_SUPPLY_HEALTH_GOOD;
	if(cellOverheating) {
		battery.power_supply_healty = sensor_msgs::BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	else if(cellCold) {
		battery.power_supply_healty = sensor_msgs::BatteryState.POWER_SUPPLY_HEALTH_COLD;
	}
	else if (bankVoltage <= BANK_VOLTAGE_MIN) {
		battery.power_supply_healty = sensor_msgs::BatteryState.POWER_SUPPLY_HEALTH_DEAD;
	}
	else if(bankVoltage >= BANK_VOLTAGE_MAX) {
		battery.power_supply_healty = sensor_msgs::BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}
	
	if(battery.power_supply_state != sensor_msgs::BatteryState.POWER_SUPPLY_HEALTH_GOOD) {
		if(safetySystemClient.call()) {
			print();
		}
		else {
			print();
		}
	}

	
	batteryStatePublisher.Publish(battery);
}

int main(int argc, char** argv){
	ros:init(argc, argv, "battery_management_system");
	BatteryManagementSystem battery_management_system;
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
		battery_management_system.update();
		
		ros::SpinOnce();
		
		loop_rate.wait();
	}
	
	return 0;
}

