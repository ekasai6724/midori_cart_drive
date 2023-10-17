#include "svon_client.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

/******************************************************************************
    コンストラクタ
******************************************************************************/
SvonClient::SvonClient(const std::string &name):Node(name)
{
	m_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
		"/joy",
		rclcpp::QoS(10),
		std::bind(&SvonClient::SubscriptionEvent, this, _1)
	);
	m_svon_client = this->create_client<midori_cart_messages::srv::SvonMessage>("svon_service");
}

/*=============================================================================
	サブスクライバ受信コールバック関数
=============================================================================*/
#define SVON_BUTTON		7		// [START]ボタン
#define SVOFF_BUTTON	6		// [BACK]ボタン
void SvonClient::SubscriptionEvent(const sensor_msgs::msg::Joy::SharedPtr msg)
{
	if(	(msg->buttons[SVON_BUTTON]  && !m_Joymsg_ex.buttons[SVON_BUTTON] ) ||
		(msg->buttons[SVOFF_BUTTON] && !m_Joymsg_ex.buttons[SVOFF_BUTTON])		)
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

	// 今回のゲームパッド入力データを保存
	m_Joymsg_ex = *msg;
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
