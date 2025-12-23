#include <rclcpp/rclcpp.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

class ControlGUI : public rclcpp::Node
{
  public:
    ControlGUI() : Node("control_gui")
    {
        p_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/image/compressed",
            10,
            std::bind(&ControlGUI::displayCallback, this, std::placeholders::_1));
    };
    ~ControlGUI() {};

  private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr p_subscriber_;

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
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlGUI>());
    rclcpp::shutdown();
    return 0;
}