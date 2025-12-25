#include <rclcpp/rclcpp.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "romur_interfaces/msg/motors_pwm_control.hpp"

namespace ROMUR
{
constexpr int SLIDER_MAX_POS   = 200;
constexpr int SLIDER_START_POS = SLIDER_MAX_POS / 2;

using pwm_t = int;

class ControlGUI : public rclcpp::Node
{
  public:
    ControlGUI() : Node("control_gui")
    {
        p_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/image/compressed",
            10,
            std::bind(&ControlGUI::displayCallback, this, std::placeholders::_1));

        p_publisher_ =
            this->create_publisher<romur_interfaces::msg::MotorsPwmControl>("motor_control", 10);

        p_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&ControlGUI::MotorsPwmPublisher, this));

        cv::namedWindow("ROMUR Control Panel");
        cv::createTrackbar("Motor 0", "ROMUR Control Panel", &motor_sliders_[0], SLIDER_MAX_POS);
        cv::createTrackbar("Motor 1", "ROMUR Control Panel", &motor_sliders_[1], SLIDER_MAX_POS);
        cv::createTrackbar("Motor 2", "ROMUR Control Panel", &motor_sliders_[2], SLIDER_MAX_POS);
        cv::createTrackbar("Motor 3", "ROMUR Control Panel", &motor_sliders_[3], SLIDER_MAX_POS);
    };
    ~ControlGUI() {};

  private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr    p_subscriber_;
    rclcpp::Publisher<romur_interfaces::msg::MotorsPwmControl>::SharedPtr p_publisher_;
    rclcpp::TimerBase::SharedPtr                                          p_timer_;

    pwm_t motor_sliders_[4] = {(pwm_t)SLIDER_START_POS,
                               (pwm_t)SLIDER_START_POS,
                               (pwm_t)SLIDER_START_POS,
                               (pwm_t)SLIDER_START_POS};

    void displayCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::imshow("ROMUR Control Panel", cv_ptr->image);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void MotorsPwmPublisher()
    {
        // vals below 0 - spin opposite direction
        romur_interfaces::msg::MotorsPwmControl msg;
        msg.motor0_pwm = motor_sliders_[0] - SLIDER_START_POS;
        msg.motor1_pwm = motor_sliders_[1] - SLIDER_START_POS;
        msg.motor2_pwm = motor_sliders_[2] - SLIDER_START_POS;
        msg.motor3_pwm = motor_sliders_[3] - SLIDER_START_POS;

        p_publisher_->publish<romur_interfaces::msg::MotorsPwmControl>(msg);
    }
};
}  // namespace ROMUR

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROMUR::ControlGUI>());
    rclcpp::shutdown();
    return 0;
}