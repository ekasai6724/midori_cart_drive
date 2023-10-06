#ifndef MIDORI_CART_DRIVE_H
#define MIDORI_CART_DRIVE_H

#include <QCoreApplication>
#include <QTextStream>
#include <QThread>
#include <QtSerialPort>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MidoriCartDrive : public rclcpp::Node
{
public:
	MidoriCartDrive(const std::string &topic_name);
	~MidoriCartDrive();
	
private:
	double					m_CmdLinear, m_CmdAngular;
	QVector<QSerialPort*>	m_TmpPorts;
	class QSerialPort		*m_SerialPort_L;
	class QSerialPort		*m_SerialPort_R;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr	m_subscriber;
	rclcpp::TimerBase::SharedPtr								m_timer;

	void SetupUSBport(void);
	bool SerialPortsOpen(void);
	void SubscriptionEvent(const geometry_msgs::msg::Twist::SharedPtr msg);
	void CyclicTimerEvent(void);
	void SerialPortSend(QSerialPort *port, QByteArray send);
};

#endif	// MIDORI_CART_DRIVE_H
