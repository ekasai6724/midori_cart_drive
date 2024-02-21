#ifndef NAVIGATION_CLIENT_HPP
#define NAVIGATION_CLIENT_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <midori_cart_messages/msg/servo_output.hpp>
#include <midori_cart_messages/msg/servo_input.hpp>

class NavigationClient : public rclcpp::Node
{
public:
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	NavigationClient(const std::string &name);
	
private:
	int16_t	m_ServoInput;

	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr		m_initialpose_pub;
	rclcpp::Publisher<midori_cart_messages::msg::ServoOutput>::SharedPtr			m_output_pub;
	rclcpp::Subscription<midori_cart_messages::msg::ServoInput>::SharedPtr			m_input_sub;
	//rclcpp::TimerBase::SharedPtr													m_timer;
	rclcpp_action::Client<NavigateToPose>::SharedPtr								m_navigation_client;
	
	void ServoInputSubscriptionEvent(const midori_cart_messages::msg::ServoInput::SharedPtr msg);
	void PublishInitialPose(void);
	void StartNavigation(void);
	void NavGoalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future);
	void NavFeedbackCallback(
		rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
		const std::shared_ptr<const NavigateToPose::Feedback> feedback
	);
	void NavResultCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result);
	//void CyclicTimerEvent(void);
};

#endif	// NAVIGATION_CLIENT_HPP