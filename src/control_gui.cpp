#include <rclcpp/rclcpp.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "romur_interfaces/msg/romur_control.hpp"

#define FEEDBACK_MSG_SIZE 11
#define SIDEBAR_WIDTH     400

#define FEEDBACK_PADDING_TOP                40
#define FEEDBACK_ELEMENT_PADDING_TOP        30
#define FEEDBACK_ELEMENT_PADDING_HORIZONTAL 10

namespace ROMUR
{
constexpr int SLIDER_MAX_POS   = 200;
constexpr int SLIDER_START_POS = SLIDER_MAX_POS / 2;

namespace name
{
constexpr std::string_view MOTOR[4] = {
    "MOTOR0 status", "MOTOR1 status", "MOTOR2 status", "MOTOR3 status"};
constexpr std::string_view LED         = "Light";
constexpr std::string_view PRESSURE    = "External pressure";
constexpr std::string_view TEMPERATURE = "External temperature";
}  // namespace name

namespace color
{
static const cv::Scalar DEFAULT = cv::Scalar(0, 0, 0);
static const cv::Scalar ERROR   = cv::Scalar(255, 0, 0);
static const cv::Scalar OK      = cv::Scalar(0, 128, 0);
static const cv::Scalar WARNING = cv::Scalar(255, 204, 0);
}  // namespace color

constexpr std::string_view MOTOR_STATUS[2] = {"OK", "BAD VALUE"};
constexpr std::string_view LED_STATUS[2]   = {"OFF", "ON"};

using pwm_t = int;
using control_subscriber_t =
    std::variant<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr,
                 rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr>;

typedef struct
{
    std::string_view name;
    std::string      value;
    cv::Scalar       color;
} feedback_element_t;

class ControlGUI : public rclcpp::Node
{
  public:
    enum class controlMode_E
    {
        SLIDER = 0,
        KEYBOARD,
        JOY
    };

    ControlGUI() : Node("control_gui")
    {
        rcl_interfaces::msg::ParameterDescriptor desc;

        desc.description =
            "available modes: \n\tslider control - 0 [default] \n\tteleop keyboard - 1 \n\tteleop "
            "joy - 2";
        this->declare_parameter<int>("control_mode", 0, desc);

        p_img_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/image/compressed",
            10,
            std::bind(&ControlGUI::displayCallback, this, std::placeholders::_1));

        p_feedback_subscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "driver_feedback_data",
            10,
            std::bind(&ControlGUI::feedbackCallback, this, std::placeholders::_1));

        p_publisher_ =
            this->create_publisher<romur_interfaces::msg::ROMURControl>("motor_control", 10);

        p_timer_      = this->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&ControlGUI::ROMURControlPublisher, this));
        control_mode_ = (controlMode_E)this->get_parameter("control_mode").as_int();

        cv::namedWindow("ROMUR Control Panel");

        switch (control_mode_)
        {
            case controlMode_E::SLIDER:
                setupControlSlider();
                break;
            case controlMode_E::KEYBOARD:
                setupControlKeyboard();
                break;
            case controlMode_E::JOY:
                setupControlJoy();
                break;

            default:
                setupControlSlider();
                break;
        }
    };
    ~ControlGUI() {};

  private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr p_img_subscriber_;
    control_subscriber_t                                               p_control_subscriber;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr    p_feedback_subscriber;
    rclcpp::Publisher<romur_interfaces::msg::ROMURControl>::SharedPtr  p_publisher_;
    rclcpp::TimerBase::SharedPtr                                       p_timer_;

    controlMode_E        control_mode_;
    pwm_t                motor_vals[4] = {0, 0, 0, 0};
    bool                 light_state_  = false;
    std::vector<uint8_t> feedback_data_;
    std::mutex           feedback_mutex_;

    void setupControlSlider()
    {
        motor_vals[0] = (pwm_t)SLIDER_START_POS;
        motor_vals[1] = (pwm_t)SLIDER_START_POS;
        motor_vals[2] = (pwm_t)SLIDER_START_POS;
        motor_vals[3] = (pwm_t)SLIDER_START_POS;

        cv::createTrackbar("Motor 0", "ROMUR Control Panel", &motor_vals[0], SLIDER_MAX_POS);
        cv::createTrackbar("Motor 1", "ROMUR Control Panel", &motor_vals[1], SLIDER_MAX_POS);
        cv::createTrackbar("Motor 2", "ROMUR Control Panel", &motor_vals[2], SLIDER_MAX_POS);
        cv::createTrackbar("Motor 3", "ROMUR Control Panel", &motor_vals[3], SLIDER_MAX_POS);
    }

    void setupControlKeyboard()
    {
        p_control_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&ControlGUI::keyboardControlCallback, this, std::placeholders::_1));
    }

    void setupControlJoy()
    {
        p_control_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControlGUI::joyControlCallback, this, std::placeholders::_1));
    }

    void parseFeedback(const std::vector<uint8_t>&      feedback,
                       std::vector<feedback_element_t>& elements)
    {
        if (feedback.size() < FEEDBACK_MSG_SIZE)
        {
            elements.clear();
            RCLCPP_INFO(this->get_logger(), "Received feedback msg too short");
            return;
        }

        elements.resize(7);

        // parsing motor statuses
        uint8_t status = feedback[0] & 0x3;

        elements[0].name  = name::MOTOR[0];
        elements[0].value = MOTOR_STATUS[status];  // get first 2 bits
        elements[0].color = status > 0 ? ROMUR::color::ERROR : ROMUR::color::OK;

        status            = (feedback[0] >> 2) & 0x3;
        elements[1].name  = name::MOTOR[1];
        elements[1].value = MOTOR_STATUS[status];  // get next 2 bits
        elements[1].color = status > 0 ? ROMUR::color::ERROR : ROMUR::color::OK;

        status            = (feedback[0] >> 4) & 0x3;
        elements[2].name  = name::MOTOR[2];
        elements[2].value = MOTOR_STATUS[status];  // get next 2 bits
        elements[2].color = status > 0 ? ROMUR::color::ERROR : ROMUR::color::OK;

        status            = (feedback[0] >> 6) & 0x3;
        elements[3].name  = name::MOTOR[3];
        elements[3].value = MOTOR_STATUS[status];  // get last 2 bits
        elements[3].color = status > 0 ? ROMUR::color::ERROR : ROMUR::color::OK;

        // parsing led status
        status            = feedback[1] & 1;
        elements[4].name  = name::LED;
        elements[4].value = LED_STATUS[status];
        elements[4].color = ROMUR::color::DEFAULT;

        // parsing pressure
        float data;
        std::memcpy(&data, &feedback[2], sizeof(data));
        elements[5].name  = name::PRESSURE;
        elements[5].value = std::to_string(data);
        elements[5].color = ROMUR::color::DEFAULT;

        // parsing temperature
        std::memcpy(&data, &feedback[6], sizeof(data));
        elements[6].name  = name::TEMPERATURE;
        elements[6].value = std::to_string(data);
        elements[6].color = ROMUR::color::DEFAULT;
    }

    void displayCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            cv::Mat& img = img_ptr->image;

            // create canvas that fits sidebar next to the image
            cv::Mat canvas(img.rows, img.cols + SIDEBAR_WIDTH, img.type());

            img.copyTo(canvas(cv::Rect(0, 0, img.cols, img.rows)));

            cv::rectangle(canvas,
                          cv::Point(img.cols, 0),
                          cv::Point(img.cols + SIDEBAR_WIDTH, img.rows),
                          cv::Scalar(50, 50, 50),
                          cv::FILLED);

            std::vector<uint8_t> feedback;
            {
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                feedback = feedback_data_;
            }

            std::vector<feedback_element_t> fb_elements;
            parseFeedback(feedback, fb_elements);

            for (size_t i = 0; i < fb_elements.size(); ++i)
            {
                std::stringstream ss;
                ss << fb_elements[i].name << ": " << fb_elements[i].value;

                cv::putText(canvas,
                            ss.str(),
                            cv::Point(img.cols + FEEDBACK_ELEMENT_PADDING_HORIZONTAL,
                                      FEEDBACK_PADDING_TOP + i * FEEDBACK_ELEMENT_PADDING_TOP),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            fb_elements[i].color,
                            2);
            }

            cv::imshow("ROMUR Control Panel", canvas);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void feedbackCallback(const std_msgs::msg::UInt8MultiArray msg)
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        feedback_data_ = msg.data;
    }

    // tank controls
    void joyControlCallback(const sensor_msgs::msg::Joy msg)
    {
        static bool prev_light_state = false;

        float motor_l = msg.axes[1] + msg.axes[0];
        float motor_r = msg.axes[1] - msg.axes[0];
        float max_mag = std::max({1.0f, std::abs(motor_l), std::abs(motor_r)});

        motor_vals[0] = motor_l / max_mag * 100;
        motor_vals[1] = motor_r / max_mag * 100;

        motor_l = msg.axes[4] + msg.axes[3];
        motor_r = msg.axes[4] - msg.axes[3];
        max_mag = std::max({1.0f, std::abs(motor_l), std::abs(motor_r)});

        motor_vals[2] = motor_l / max_mag * 100;
        motor_vals[3] = motor_r / max_mag * 100;

        if ((bool)msg.buttons[3] && !prev_light_state)
            light_state_ ^= 1;

        prev_light_state = (bool)msg.buttons[3];
    }

    // thrust lever control
    void keyboardControlCallback(const geometry_msgs::msg::Twist msg)
    {
        motor_vals[0] = msg.linear.x * 100;
        motor_vals[1] = msg.linear.y * 100;
        motor_vals[2] = msg.linear.z * 100;
        motor_vals[3] = msg.angular.x * 100;

        light_state_ = (bool)msg.angular.y;

        return;
    }

    void ROMURControlPublisher()
    {
        // vals below 0 - spin in opposite direction
        romur_interfaces::msg::ROMURControl msg;
        if (control_mode_ == controlMode_E::SLIDER)
        {
            msg.motors.motor0_pwm = motor_vals[0] -= SLIDER_START_POS;
            msg.motors.motor1_pwm = motor_vals[1] -= SLIDER_START_POS;
            msg.motors.motor2_pwm = motor_vals[2] -= SLIDER_START_POS;
            msg.motors.motor3_pwm = motor_vals[3] -= SLIDER_START_POS;
        }

        msg.motors.motor0_pwm = motor_vals[0];
        msg.motors.motor1_pwm = motor_vals[1];
        msg.motors.motor2_pwm = motor_vals[2];
        msg.motors.motor3_pwm = motor_vals[3];

        msg.light.status = light_state_;

        p_publisher_->publish(msg);
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