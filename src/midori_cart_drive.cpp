#include "midori_cart_drive.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

/******************************************************************************
	コンストラクタ
******************************************************************************/
MidoriCartDrive::MidoriCartDrive(const std::string &name):Node(name)
{
	// パラメータ宣言
	this->declare_parameter("gear_rate");
	this->declare_parameter("num_of_teeth1");
	this->declare_parameter("num_of_teeth2");
	this->declare_parameter("tire_diameter");
	this->declare_parameter("tred_width");
	this->declare_parameter("cmd_reset_interval");
	this->declare_parameter("odometry.frame_id");
	this->declare_parameter("odometry.child_frame_id");
	this->declare_parameter("odometry.publish_tf");

	// Launchファイルから渡されたパラメータデータの取得
	for(auto & parameter : this->get_parameters
		({"gear_rate", "num_of_teeth1", "num_of_teeth2", "tire_diameter", "tred_width", "cmd_reset_interval",
		  "odometry.frame_id", "odometry.child_frame_id", "odometry.publish_tf"								}))
	{
		std::stringstream ss;
		ss << "Name: " << parameter.get_name() << ", Value (" << parameter.get_type_name() << "): " << parameter.value_to_string();
		RCLCPP_INFO(this->get_logger(), ss.str().c_str());
	}
	this->get_parameter_or<double>("gear_rate",     p_GearRate,     1.0   );
	this->get_parameter_or<double>("num_of_teeth1", p_NumTeeth1,    24.0  );
	this->get_parameter_or<double>("num_of_teeth2", p_NumTeeth2,    72.0  );
	this->get_parameter_or<double>("tire_diameter", p_TireDiameter, 210.0 );
	this->get_parameter_or<double>("tred_width",    p_TredWidth,    537.0 );

	this->get_parameter_or<int32_t>("cmd_reset_interval", p_CmdResetInterval, 0);
	
	this->get_parameter_or<std::string>("odometry.frame_id",       p_OdmFrameID, std::string("odom")           );
	this->get_parameter_or<std::string>("odometry.child_frame_id", p_OdmChildID, std::string("base_footprint") );
	this->get_parameter_or<bool>       ("odometry.publish_tf",     p_PublishTF,  false                         );

	QThread::msleep(1000);
	
	// USBシリアルポートオープン
	SetupUSBport(); 
	if(SerialPortsOpen())
	{
		RCLCPP_INFO(this->get_logger(), "Both serial ports open.");

		// 速度指令サブスクライバ作成して受信コールバック関数を登録
		m_cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
			"/cmd_vel",
			rclcpp::QoS(10),
			std::bind(&MidoriCartDrive::CmdvelSubscriptionEvent, this, _1)
		);
		
		// サーボオン/オフサービス作成してコールバック関数を登録
		m_svon_srv = this->create_service<midori_cart_messages::srv::SvonMessage>(
			"svon_service",
			std::bind(&MidoriCartDrive::SvonCommandEvent, this, _1, _2)
		);

		// 左車輪サーボ制御入力返信パブリッシャ作成
		m_inputL_pub = this->create_publisher<midori_cart_messages::msg::ServoInput>(
			"/servo_input_L",
			rclcpp::QoS(10)
		);

		// 左車輪サーボ制御出力指令サブスクライバ作成して受信コールバック関数を登録
		m_outputL_sub = this->create_subscription<midori_cart_messages::msg::ServoOutput>(
			"/servo_output_L",
			rclcpp::QoS(10),
			std::bind(&MidoriCartDrive::ServoOutputSubscriptionEvent, this, _1)
		);

		// odometryデータパブリッシャ作成
		m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
			"/odom",
			rclcpp::QoS(10)
		);
		m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

		// 周期タイマ作成してコールバック関数を登録
		m_timer = this->create_wall_timer(
			100ms,
			std::bind(&MidoriCartDrive::CyclicTimerEvent, this)
		);
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Counldn't open the port.");
	}
}

/******************************************************************************
	デストラクタ
******************************************************************************/
MidoriCartDrive::~MidoriCartDrive()
{
	SerialPortSend(m_SerialPort_R, "9A;SVOFF");
	SerialPortSend(m_SerialPort_L, "9A;SVOFF");
	RCLCPP_INFO(this->get_logger(), "Terminated.");
	QThread::msleep(1000);
}

/*=============================================================================
	USBシリアルポート検索して接続準備
=============================================================================*/
void MidoriCartDrive::SetupUSBport(void)
{
	//QTextStream qso(stdout);

	m_TmpPorts.clear();
	foreach(const QSerialPortInfo info, QSerialPortInfo::availablePorts())
	{
		if(info.portName().left(6) == "ttyACM")
		{
			//qso << "Name        :" << info.portName() << endl;
			//qso << "Description :" << info.description() << endl;
			//qso << "Manufacturer:" << info.manufacturer() << endl;

			QSerialPort *port = new QSerialPort();
			port->setPortName(info.portName());
			port->setBaudRate(QSerialPort::Baud9600);
			port->setDataBits(QSerialPort::Data8);
			port->setParity(QSerialPort::EvenParity);
			port->setStopBits(QSerialPort::OneStop);
			m_TmpPorts.append(port);
		}
	}
}

/*=========================================================================
	USBシリアルポートオープンして車輪モータ軸を確認
	(有効なポートをすべてオープンして軸番号読出し、2個検出したらtrue)
 *========================================================================*/
bool MidoriCartDrive::SerialPortsOpen(void)
{
	int cnt = 0;

	foreach(QSerialPort *port , m_TmpPorts)
	{
		if(port->open(QIODevice::ReadWrite))
		{
			{
				std::stringstream ss;
				ss << "Port: " << port->portName().toStdString() << " open.";
				RCLCPP_INFO(this->get_logger(), ss.str().c_str());
			}

			SerialPortSend(port, "9A;PR;14");   // N0020「軸番号」読出しコマンド
			QByteArray recv;
			if(SerialPortRecv(port, &recv))
			{
				{
					std::stringstream ss;
					ss << "[recv]:" << recv.toStdString();
					RCLCPP_INFO(this->get_logger(), ss.str().c_str());
				}

				if      (recv.mid(7, 8) == "00000000")  { m_SerialPort_R = port; cnt++; }   // 軸番号0:右車輪
				else if (recv.mid(7, 8) == "00000001")  { m_SerialPort_L = port; cnt++; }   // 軸番号1:左車輪
			}

			SerialPortSend(port, "9A;IOTEN;0007");		// 制御入力テスト全ビット有効化
			SerialPortRecv(port, &recv);
			SerialPortSend(port, "9A;ZSET;0");			// サーボ座標リセット
			SerialPortRecv(port, &recv);
		}
	}
	m_SvonCmd = false;
	m_SvoffCmd = true;
	m_IoRequest = false;

	if(cnt == 2)    return true;
	else            return false;
}

/*=============================================================================
	ロボット速度指令受信コールバック関数
=============================================================================*/
void MidoriCartDrive::CmdvelSubscriptionEvent(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	// 受信したL/A指令をメンバ変数に保存するだけ
	m_CmdLinear = msg->linear.x;
	m_CmdAngular = msg->angular.z;
	m_CmdIntervalCnt = 0;
}

/*=============================================================================
	サーボオン/オフ指令サービスのコールバック関数
=============================================================================*/
void MidoriCartDrive::SvonCommandEvent(
		const std::shared_ptr<midori_cart_messages::srv::SvonMessage::Request> request,
		std::shared_ptr<midori_cart_messages::srv::SvonMessage::Response>      response)
{
	// 本来はサーボオン/オフ指令を受けてUSBシリアル通信コマンドを送信して
	// その返信を受信してサービスレスポンスを返したいが、
	// ROSのサービス通信ではそのような処理はできないようなので断念
	// とりあえずサービス通信構造を残し、タイマ処理にサーボオン/オフ指令を渡すだけとする

	bool	ret = true;

	if(request->cmd == true)
	{
		RCLCPP_INFO(this->get_logger(), "Servo ON command.");
		m_SvonCmd = true;
		m_SvoffCmd = false;
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Servo OFF command.");
		m_SvonCmd = false;
		m_SvoffCmd = true;
	}
	response->result = ret;
}

/*=============================================================================
	サーボ制御出力指令受信コールバック関数
=============================================================================*/
void MidoriCartDrive::ServoOutputSubscriptionEvent(const midori_cart_messages::msg::ServoOutput::SharedPtr msg)
{
	m_OutputR_cmd = msg->output;
	//m_IoRequest = true;
}

/*=============================================================================
	周期タイマコールバック関数(100ms)
=============================================================================*/
#define	TIMER_INTERVAL_MS	(100)
void MidoriCartDrive::CyclicTimerEvent(void)
{
	// 速度指令作成
	// cmd_velを一定の期間受信しなかったら停止させる
	if(p_CmdResetInterval)
	{
		if(++m_CmdIntervalCnt >= p_CmdResetInterval / TIMER_INTERVAL_MS)
		{
			m_CmdIntervalCnt = p_CmdResetInterval / TIMER_INTERVAL_MS;
			m_CmdLinear = 0.0;
			m_CmdAngular = 0.0;
		}
	}

	double  gear_rate = p_GearRate * p_NumTeeth1 / p_NumTeeth2;     // モータ軸から車輪軸までの減速比
	double  vl, va;
	
	vl = m_CmdLinear * 1000 / (p_TireDiameter * 3.14) / gear_rate * 60;
	va = m_CmdAngular * (p_TredWidth / 2 / 1000) * 1000 / (p_TireDiameter * 3.14) / gear_rate * 60;
	int vcom_R = vl + va,
		vcom_L = vl - va;

	//QTextStream qso(stdout);
	//qso << "linear.x = " << QString::number(m_CmdLinear) << ", angular.z = " << QString::number(m_CmdAngular) << endl;
	//qso << "vl = " << QString::number(vl) << ", va = " << QString::number(va) << endl;

	// 速度指令シリアル通信コマンド送信
	if(m_SerialPort_R->isOpen() && m_SerialPort_L->isOpen())
	{
		QByteArray	recv;

		// サーボオン指令
		if(m_SvonCmd)
		{
			SerialPortSend(m_SerialPort_R, "9A;SVON");
			SerialPortSend(m_SerialPort_L, "9A;SVON");
			SerialPortRecv(m_SerialPort_R, &recv);
			SerialPortRecv(m_SerialPort_L, &recv);
			m_SvonCmd = false;
		}
		// サーボオフ指令
		else if(m_SvoffCmd)
		{
			SerialPortSend(m_SerialPort_R, "9A;SVOFF");
			SerialPortSend(m_SerialPort_L, "9A;SVOFF");
			SerialPortRecv(m_SerialPort_R, &recv);
			SerialPortRecv(m_SerialPort_L, &recv);
			m_SvoffCmd = false;
		}
		// サーボ制御出力/制御入力読込
		else if(m_IoRequest)
		{
			ServoIO(m_SerialPort_R, m_OutputR_cmd);
			m_IoRequest = false;	// 次回周期は速度指令通信
		}
		// 左右車輪速度指令出力/車輪回転角度読込
		else
		{
			QByteArray  sbf;
			sbf.append("9A;VREF;");
			sbf.append(QString::number(vcom_R, 16).rightJustified(8, '0').toLatin1());
			SerialPortSend(m_SerialPort_R, sbf);
			sbf.clear();
			sbf.append("9A;VREF;");
			sbf.append(QString::number(vcom_L, 16).rightJustified(8, '0').toLatin1());
			SerialPortSend(m_SerialPort_L, sbf);
			sbf.clear();

			// [VREF]コマンド返信の現在位置[pulse]を[rad]に変換
			SerialPortRecv(m_SerialPort_R, &recv);
			int32_t bf0 = recv.mid(9, 8).toLong(0, 16);
			m_JointPositions[0] = (double)bf0 * gear_rate * 2 * 3.14 / 10000;
			SerialPortRecv(m_SerialPort_L, &recv);
			int32_t bf1 = recv.mid(9, 8).toLong(0, 16);
			m_JointPositions[1] = (double)bf1 * gear_rate * 2 * 3.14 / 10000;
			std::stringstream ss;
			ss << "JointPosition_R: " << std::to_string(m_JointPositions[0]) << ", JointPosition_L: " << std::to_string(m_JointPositions[1]);
			//ss << "JointPosition_R: " << std::to_string(bf0) << ", JointPosition_L: " << std::to_string(bf1);
			//RCLCPP_INFO(this->get_logger(), ss.str().c_str());

			// odometryデータを算出してpublish
			rclcpp::Clock ros_clock(RCL_ROS_TIME);
			OdometryMain(ros_clock.now());

			m_IoRequest = true;		// 次回周期はサーボI/O通信
		}
	}
}

/*=============================================================================
	サーボ制御入出力処理
=============================================================================*/
bool MidoriCartDrive::ServoIO(QSerialPort *port, int16_t out)
{
	QByteArray	send, recv;
	bool 		ret = false;

	// 制御出力データ送信
	send.append("9A;IOTDT;");
	send.append(QString::number(out, 16).rightJustified(8, '0').toLatin1());
	SerialPortSend(port, send);

	// 制御入力状態を受信してノード外部に発行
	ret = SerialPortRecv(port, &recv);
	if(ret)
	{
		std::stringstream ss;
		ss << "[recv]:" << recv.toStdString();
		//RCLCPP_INFO(this->get_logger(), ss.str().c_str());
		
		if(recv.mid(4, 5) == "IOTDT")
		{
			auto msg = midori_cart_messages::msg::ServoInput();
			msg.input = recv.mid(10, 8).toInt(0, 16);
			m_inputL_pub->publish(msg);
			ret = true;
		}
	}
	return ret;
}

/*=============================================================================
	odometry(現在位置/速度のデータ)メイン
=============================================================================*/
void MidoriCartDrive::OdometryMain(const rclcpp::Time &now)
{
	static rclcpp::Time last_time = now;
	rclcpp::Duration duration(now.nanoseconds() - last_time.nanoseconds());

	CalculateOdometry(duration);
	PublishOdometry(now);
}

/*=============================================================================
	odometryデータ算出
=============================================================================*/
bool MidoriCartDrive::CalculateOdometry(const rclcpp::Duration &duration)
{
	static std::array<double, 2>	last_joint_positions = {0.0f, 0.0f};

	double delta_s = 0.0;
	double delta_theta = 0.0;
	double theta = 0.0;

	double v = 0.0;		// translational velocity [m/s]
	double w = 0.0;		// rotational velocity [rad/s]

	double step_time = duration.seconds();

	if (step_time == 0.0) {
		return false;
	}

	// rotation value of wheel [rad]
	double	wheel_r = m_JointPositions[0] - last_joint_positions[0];
	double	wheel_l = m_JointPositions[1] - last_joint_positions[1];

	if (std::isnan(wheel_l))	wheel_l = 0.0;
	if (std::isnan(wheel_r))	wheel_r = 0.0;

	delta_s = (p_TireDiameter / 1000 / 2.0) * (wheel_r + wheel_l) / 2.0;
	theta = (p_TireDiameter / 1000 / 2.0) * (wheel_r - wheel_l) / (p_TredWidth / 1000);
	delta_theta = theta;

	// compute odometric pose
	m_RobotPose[0] += delta_s * cos(m_RobotPose[2] + (delta_theta / 2.0));
	m_RobotPose[1] += delta_s * sin(m_RobotPose[2] + (delta_theta / 2.0));
	m_RobotPose[2] += delta_theta;

	{
		std::stringstream ss;
		ss << "RobotPose[0]: " << std::to_string(m_RobotPose[0]) << ", " 
		   << "RobotPose[1]: " << std::to_string(m_RobotPose[1]) << ", "
		   << "RobotPose[2]: " << std::to_string(m_RobotPose[2]);
		//RCLCPP_INFO(this->get_logger(), ss.str().c_str());
	}
	//RCLCPP_DEBUG(nh_->get_logger(), "x : %f, y : %f", m_RobotPose[0], m_RobotPose[1]);

	// compute odometric instantaneouse velocity
	v = delta_s / step_time;
	w = delta_theta / step_time;

	m_RobotVel[0] = v;
	m_RobotVel[1] = 0.0;
	m_RobotVel[2] = w;

	last_joint_positions[0] = m_JointPositions[0];
	last_joint_positions[1] = m_JointPositions[1];

	return true;
}

/*=============================================================================
	odometryデータ発行
=============================================================================*/
void MidoriCartDrive::PublishOdometry(const rclcpp::Time &now)
{
	auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

	odom_msg->header.frame_id = p_OdmFrameID;
	odom_msg->child_frame_id = p_OdmChildID;
	odom_msg->header.stamp = now;

	odom_msg->pose.pose.position.x = m_RobotPose[0];
	odom_msg->pose.pose.position.y = m_RobotPose[1];
	odom_msg->pose.pose.position.z = 0;

	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, m_RobotPose[2]);

	odom_msg->pose.pose.orientation.x = q.x();
	odom_msg->pose.pose.orientation.y = q.y();
	odom_msg->pose.pose.orientation.z = q.z();
	odom_msg->pose.pose.orientation.w = q.w();

	odom_msg->twist.twist.linear.x = m_RobotVel[0];
	odom_msg->twist.twist.angular.z = m_RobotVel[2];

	geometry_msgs::msg::TransformStamped odom_tf;

	odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
	odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
	odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
	odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

	odom_tf.header.frame_id = p_OdmFrameID;
	odom_tf.child_frame_id = p_OdmChildID;
	odom_tf.header.stamp = now;

	m_odom_pub->publish(std::move(odom_msg));
	if(p_PublishTF)	m_tf_broadcaster->sendTransform(odom_tf);
}

/*=============================================================================
	USBシリアルポート送信
=============================================================================*/
void MidoriCartDrive::SerialPortSend(QSerialPort *port, QByteArray send)
{
	QByteArray  sbf;

	port->clear(QSerialPort::Input);	// 受信バッファクリア
	
	sbf.append(0x02);   // STX
	sbf.append(send);
	sbf.append(0x03);   // ETX

	port->write(sbf);
}

/*=============================================================================
	USBシリアルポート受信
=============================================================================*/
bool MidoriCartDrive::SerialPortRecv(QSerialPort *port, QByteArray *recv)
{
	bool ret = false;

	recv->clear();
	while(port->waitForReadyRead(500))
	{
			if(port->bytesAvailable() > 0)
			{
				*recv += port->readAll();
			}
			if((recv->indexOf(0x03) != -1) || (recv->indexOf(0x04) != -1))
			{
				ret = true;
				break;
			}
	}
	return ret;
}

/******************************************************************************
	メイン関数
******************************************************************************/
int main(int argc, char *argv[])
{
	// Qtアプリケーションインスタンスの作成
	QCoreApplication a(argc, argv);

	// クライアントライブラリの初期化
	rclcpp::init(argc, argv);

	// サブスクライバノード作成
	auto node = std::make_shared<MidoriCartDrive>("midori_cart_drive");

	// ROSとQtアプリケーションループ
	rclcpp::WallRate loop_rate(20);
	while(rclcpp::ok())
	{
		a.processEvents();
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	node.reset();
	rclcpp::shutdown();
	return 0;
}