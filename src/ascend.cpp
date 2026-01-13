#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "romur_interfaces/msg/romur_control.hpp"

#include <fstream>
#include <iostream>

namespace ROMUR
{

constexpr std::string_view MOTOR_STATUS[2] = {"OK", "BAD VALUE"};
constexpr std::string_view LED_STATUS[2]   = {"OFF", "ON"};
constexpr int8_t           MIN_PWM_DUTY    = -100;
constexpr int8_t           MAX_PWM_DUTY    = 100;

class CSVWriter
{
  public:
    CSVWriter() = default;
    ~CSVWriter()
    {
        close();
    };

    bool open(const std::string& file_path)
    {
        file_.open(file_path);
        return file_.is_open();
    };

    void close()
    {
        if (file_.is_open())
            file_.close();
    }

    void write(const std::string& content)
    {
        if (file_.is_open())
            file_ << content;
    }
    bool is_open()
    {
        return file_.is_open();
    }

  private:
    std::ofstream file_;
};

class Ascend : public rclcpp::Node
{
  public:
    Ascend() : Node("ascend")
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "recording file path";
        this->declare_parameter<std::string>("rec_path", "asc_recording.csv", desc);

        recording_path_ = this->get_parameter("rec_path").as_string();

        desc.description = "pwm duty";
        this->declare_parameter<int>("pwm_duty", 50, desc);

        duty_ = this->get_parameter("pwm_duty").as_int();

        duty_ = std::clamp(duty_, (int8_t)-100, (int8_t)100);

        p_feedback_subscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "driver_feedback_data",
            10,
            std::bind(&Ascend::feedbackCallback, this, std::placeholders::_1));

        p_publisher_ =
            this->create_publisher<romur_interfaces::msg::ROMURControl>("romur_control", 10);

        p_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&Ascend::ROMURControlPublisher, this));

        writer_.open(recording_path_);
    };
    ~Ascend()
    {
        writer_.close();
    };

  private:
    std::string recording_path_;
    CSVWriter   writer_;
    int8_t      duty_;

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr   p_feedback_subscriber;
    rclcpp::Publisher<romur_interfaces::msg::ROMURControl>::SharedPtr p_publisher_;
    rclcpp::TimerBase::SharedPtr                                      p_timer_;

    void feedbackCallback(const std_msgs::msg::UInt8MultiArray msg)
    {
        std::stringstream ss;
        auto&             data = msg.data;

        // motors
        uint8_t status = data[0] & 0x3;
        ss << MOTOR_STATUS[status] << ',';

        status = (data[0] >> 2) & 0x3;
        ss << MOTOR_STATUS[status] << ',';

        status = (data[0] >> 4) & 0x3;
        ss << MOTOR_STATUS[status] << ',';

        status = (data[0] >> 6) & 0x3;
        ss << MOTOR_STATUS[status] << ',';

        // led
        status = data[1] & 1;
        ss << MOTOR_STATUS[status] << ',';

        // pressure
        float ms_data = 0;
        std::memcpy(&ms_data, &data[2], sizeof(ms_data));
        ss << std::to_string(ms_data) << ',';

        // temperature
        std::memcpy(&ms_data, &data[6], sizeof(ms_data));
        ss << std::to_string(ms_data) << ',';

        ss << '\n';

        writer_.write(ss.str());
    }

    void ROMURControlPublisher()
    {
        romur_interfaces::msg::ROMURControl msg;
        msg.motors.motor0_pwm = duty_;
        msg.motors.motor1_pwm = duty_;
        msg.motors.motor2_pwm = 0;
        msg.motors.motor3_pwm = 0;

        msg.light.status = false;

        p_publisher_->publish(msg);
    }
};
}  // namespace ROMUR

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROMUR::Ascend>());
    rclcpp::shutdown();
    return 0;
}