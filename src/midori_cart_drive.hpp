#ifndef MIDORI_CART_DRIVE_H
#define MIDORI_CART_DRIVE_H

#include <QCoreApplication>
#include <QTextStream>
#include <QThread>
#include <QtSerialPort>

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <midori_cart_messages/msg/servo_input.hpp>
#include <midori_cart_messages/msg/servo_output.hpp>
#include <midori_cart_messages/srv/svon_message.hpp>

class MidoriCartDrive : public rclcpp::Node
{
public:
	MidoriCartDrive(const std::string &name);
	~MidoriCartDrive();
	
private:
	double					p_GearRate,				// 減速機の減速比(1/5減速機の場合は0.2)
							p_NumTeeth1,			// 減速機側プーリ歯数
							p_NumTeeth2,			// タイヤ側プーリ歯数
							p_TireDiameter,			// タイヤ直径[mm]
							p_TredWidth;			// トレッド幅[mm]
	int32_t					p_CmdResetInterval;		// 速度指令をリセットする間隔[ms]
	std::string				p_OdmFrameID,
							p_OdmChildID;
	bool					p_PublishTF;

	QVector<QSerialPort*>	m_TmpPorts;
	class QSerialPort		*m_SerialPort_L;
	class QSerialPort		*m_SerialPort_R;
	double					m_CmdLinear,			// 前後方向指令速度[m/s]
							m_CmdAngular;			// 旋回方向指令速度[rad/s]
	int32_t					m_CmdIntervalCnt;		// cmd_vel受信していない期間のカウント
	bool					m_SvonCmd,				// サーボオン指令リクエスト
							m_SvoffCmd,				// サーボオフ指令リクエスト
							m_IoRequest;			// サーボ制御入出力更新リクエスト
	int16_t					m_OutputR_cmd;			// 右車輪サーボ制御出力指令
	std::array<double, 2>	m_JointPositions;		// 左右車輪のスタート角度からの回転量[rad]
	std::array<double, 3>	m_RobotPose;
	std::array<double, 3>	m_RobotVel;
	
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr					m_cmdvel_sub;
	rclcpp::Service<midori_cart_messages::srv::SvonMessage>::SharedPtr			m_svon_srv;
	rclcpp::Subscription<midori_cart_messages::msg::ServoOutput>::SharedPtr		m_outputL_sub;
	rclcpp::Publisher<midori_cart_messages::msg::ServoInput>::SharedPtr			m_inputL_pub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr						m_odom_pub;
	rclcpp::TimerBase::SharedPtr												m_timer;
	std::unique_ptr<tf2_ros::TransformBroadcaster>								m_tf_broadcaster;

	void SetupUSBport(void);
	bool SerialPortsOpen(void);
	void CmdvelSubscriptionEvent(const geometry_msgs::msg::Twist::SharedPtr msg);
	void SvonCommandEvent(
        const std::shared_ptr<midori_cart_messages::srv::SvonMessage::Request> request,
        std::shared_ptr<midori_cart_messages::srv::SvonMessage::Response>      response);
	void ServoOutputSubscriptionEvent(const midori_cart_messages::msg::ServoOutput::SharedPtr msg);
	void CyclicTimerEvent(void);
	bool ServoIO(QSerialPort *port, int16_t out);
	void OdometryMain(const rclcpp::Time &now);
	bool CalculateOdometry(const rclcpp::Duration & duration);
	void PublishOdometry(const rclcpp::Time &now);
	void SerialPortSend(QSerialPort *port, QByteArray send);
	bool SerialPortRecv(QSerialPort *port, QByteArray *recv);

};

#endif	// MIDORI_CART_DRIVE_H
