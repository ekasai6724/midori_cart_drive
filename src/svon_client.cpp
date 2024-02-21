#include "svon_client.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define	OUT_TRG_BUTTON		0		// [A]ボタン
#define SVON_BUTTON			7		// [START]ボタン
#define SVOFF_BUTTON		6		// [BACK]ボタン

/******************************************************************************
    コンストラクタ
******************************************************************************/
SvonClient::SvonClient(const std::string &name):Node(name)
{
	m_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
		"/joy",
		rclcpp::QoS(10),
		std::bind(&SvonClient::JoySubscriptionEvent, this, _1)
	);
	m_svon_client = this->create_client<midori_cart_messages::srv::SvonMessage>("svon_service");
	m_output_pub = this->create_publisher<midori_cart_messages::msg::ServoOutput>(
		"/servo_output_L",
		rclcpp::QoS(10)
	);
	m_input_sub = this->create_subscription<midori_cart_messages::msg::ServoInput>(
		"/servo_input_L",
		rclcpp::QoS(10),
		std::bind(&SvonClient::ServoInputSubscriptionEvent, this, _1)
	);
	
	
	for(int i = 0; i < 8; i++)		m_joymsg_ex.axes.push_back(0);
	for(int i = 0; i < 11; i++)		m_joymsg_ex.buttons.push_back(0);
}

/*=============================================================================
	ゲームパッド入力コールバック関数
=============================================================================*/
void SvonClient::JoySubscriptionEvent(const sensor_msgs::msg::Joy::SharedPtr msg)
{
	// サーボオン/オフ指令
	if(	(msg->buttons[SVON_BUTTON]  && !m_joymsg_ex.buttons[SVON_BUTTON] ) ||
		(msg->buttons[SVOFF_BUTTON] && !m_joymsg_ex.buttons[SVOFF_BUTTON])		)
	{
		// サーボオン指令サービスの起動確認
		while(!m_svon_client->wait_for_service(500ms))
		{
			if(!rclcpp::ok())
			{
				RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service");
				return;
			}
			//RCLCPP_INFO(this->get_logger(), "waiting for service...");
		}

		// サーボオン指令サービスへのリクエスト設定
		auto request = std::make_shared<midori_cart_messages::srv::SvonMessage::Request>();
		if(msg->buttons[SVON_BUTTON])	request->cmd = true;
		else							request->cmd = false;
		
		auto future_res = m_svon_client->async_send_request(
			request,
			std::bind(&SvonClient::SvonResponseEvent, this, _1)
		);
	}
	// サーボ制御出力指令と制御入力読込要求
	// 試験実装:ゲームパッドのAボタンをトリガとして、サーボから読み出した
	//			制御入力状態をそのまま制御出力に伝達する
	else if(msg->buttons[OUT_TRG_BUTTON] && !m_joymsg_ex.buttons[OUT_TRG_BUTTON])
	{
		auto msg = midori_cart_messages::msg::ServoOutput();
		msg.output = m_ServoInput;
		m_output_pub->publish(msg);
	}

	// 今回のゲームパッド入力データを保存
	m_joymsg_ex = *msg;
}

/*=============================================================================
	サーボオン指令サービスレスポンスコールバック関数
=============================================================================*/
void SvonClient::SvonResponseEvent(rclcpp::Client<midori_cart_messages::srv::SvonMessage>::SharedFuture future)
{
	std::stringstream ss;
	if(future.get()->result)	ss << "res: True";
	else						ss << "res: False";
	RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

/*=============================================================================
	サーボ制御入力受信コールバック関数
=============================================================================*/
void SvonClient::ServoInputSubscriptionEvent(const midori_cart_messages::msg::ServoInput::SharedPtr msg)
{
	std::stringstream ss;
	ss << "Servo Input: " << std::to_string(msg->input);
	m_ServoInput = msg->input;
	//RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

/******************************************************************************
    メイン関数
******************************************************************************/
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SvonClient>("svon_client"));
	rclcpp::shutdown();
	return 0;
}
