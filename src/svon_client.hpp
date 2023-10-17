#ifndef SVON_CLIENT_HPP
#define SVON_CLIENT_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <midori_cart_messages/srv/svon_message.hpp>

class SvonClient : public rclcpp::Node
{
public:
	SvonClient(const std::string &name);
	
private:
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr				m_joy_sub;
	rclcpp::Client<midori_cart_messages::srv::SvonMessage>::SharedPtr	m_svon_client;
	//rclcpp::TimerBase::SharedPtr										m_timer;
	sensor_msgs::msg::Joy												m_Joymsg_ex;

	void SubscriptionEvent(const sensor_msgs::msg::Joy::SharedPtr msg);
	void SvonResponseEvent(rclcpp::Client<midori_cart_messages::srv::SvonMessage>::SharedFuture future);
	//void CyclicTimerEvent(void);
};

#endif	// SVON_CLIENT_HPP