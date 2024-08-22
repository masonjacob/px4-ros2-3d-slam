#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include <geometry_msgs/msg/PoseStamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class GPSBridge : public rclcpp::Node {
    public:

		GPSBridge() : Node("Bridge") {

			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
			[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
				std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
				std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
				std::cout << "============================="   << std::endl;
				std::cout << "ts: "          << msg->timestamp    << std::endl;
				std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
				std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
				std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
				std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
				std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
				std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
				std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
				std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
				std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;

				Sensor_data = msg;
			});

			Opti_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(" name of opti topic", qos,
			[this](const geometry_msgs::msg::PoseStamped::UniquePtr msg) {
				Opti_data = msg;
			});

			vehicle_local_position_pub = this->create_publisher<VehicleLocalPosition>("/fmu/out/vehicle_local_position", 10);

			auto timer_callback = [this]() -> void {
				this->publish_local_position(Sensor_data, Opti_data);
			};

			timer_ = this->create_wall_timer(100ms, timer_callback);
		}

    private:
        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr Opti_sub;

        rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_pub;

		px4_msgs::msg::SensorCombined::UniquePtr Sensor_data;
		geometry_msgs::msg::PoseStamped::UniquePtr Opti_data;

        void publish_local_position(px4_msgs::msg::SensorCombined::UniquePtr Sensor_data,
									geometry_msgs::msg::PoseStamped::UniquePtr Opti_data);


}

void GPSBridge::publish_local_position(Sensor_data, Opti_data)
{
	VehicleLocalPosition msg{};

	vehicle_local_position_pub->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting GPSBridge listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GPSBridge>());

	rclcpp::shutdown();
	return 0;
}