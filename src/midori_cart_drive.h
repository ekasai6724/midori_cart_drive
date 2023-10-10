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
	QVector<QSerialPort*>	m_TmpPorts;
	class QSerialPort		*m_SerialPort_L;
	class QSerialPort		*m_SerialPort_R;
	double					m_CmdLinear,		// 前後方向指令速度[m/s]
							m_CmdAngular;		// 旋回方向指令速度[rad/s]
	double					m_GearRate,			// 減速機の減速比(1/5減速機の場合は0.5)
							m_NumTeeth1,		// 減速機側プーリ歯数
							m_NumTeeth2,		// タイヤ側プーリ歯数
							m_TireDiameter,		// タイヤ直径[mm]
							m_TredWidth;		// トレッド幅[mm]

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr	m_subscriber;
	rclcpp::TimerBase::SharedPtr								m_timer;

	void SetupUSBport(void);
	bool SerialPortsOpen(void);
	void SubscriptionEvent(const geometry_msgs::msg::Twist::SharedPtr msg);
	void CyclicTimerEvent(void);
	void SerialPortSend(QSerialPort *port, QByteArray send);
};

#endif	// MIDORI_CART_DRIVE_H
