#include "navigation_client.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

/******************************************************************************
    コンストラクタ
******************************************************************************/
NavigationClient::NavigationClient(const std::string &name):Node(name)
{
	// 初期位置指定パブリッシャ作成
	m_initialpose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/initialpose",
		rclcpp::QoS(10)
	);

	// 左車輪サーボ制御出力指令パブリッシャ作成(現時点では使用していない)
	m_output_pub = this->create_publisher<midori_cart_messages::msg::ServoOutput>(
		"/servo_output_L",
		rclcpp::QoS(10)
	);

	// 左車輪サーボ制御入力サブスクライバ作成して受信コールバック関数を登録
	m_input_sub = this->create_subscription<midori_cart_messages::msg::ServoInput>(
		"/servo_input_L",
		rclcpp::QoS(10),
		std::bind(&NavigationClient::ServoInputSubscriptionEvent, this, _1)
	);

	// ナビゲーション(目標位置移動)アクションクライアント作成
	m_navigation_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
}

/*=============================================================================
	サーボ制御入力受信コールバック関数
=============================================================================*/
void NavigationClient::ServoInputSubscriptionEvent(const midori_cart_messages::msg::ServoInput::SharedPtr msg)
{
	//std::stringstream ss;
	//ss << "Servo Input: " << std::to_string(msg->input);
	//RCLCPP_INFO(this->get_logger(), ss.str().c_str());

	// IN0立上りで初期位置指定
	if(!(m_ServoInput & 0x0001) && (msg->input & 0x0001))	PublishInitialPose();

	// IN1立上りで初期位置に戻る
	if(!(m_ServoInput & 0x0002) && (msg->input & 0x0002))	StartNavigation();

	m_ServoInput = msg->input;
}

/*=============================================================================
	初期位置指定データ送信
=============================================================================*/
void NavigationClient::PublishInitialPose(void)
{
	auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
	rclcpp::Clock ros_clock(RCL_ROS_TIME);

    initial_pose.header.stamp=ros_clock.now();
    initial_pose.header.frame_id="map";
    initial_pose.pose.pose.position.x = 0.0;
    initial_pose.pose.pose.position.y = 0.0;
    initial_pose.pose.pose.orientation.z = 0.0;
    initial_pose.pose.covariance =
	{
		0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.06853891945200942
	};
    m_initialpose_pub->publish(initial_pose);
}

/*=============================================================================
	目標位置移動(アクション)起動
=============================================================================*/
void NavigationClient::StartNavigation(void)
{
	// アクションサーバの起動確認
	while(!m_navigation_client->wait_for_action_server(500ms))
	{
		if(!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service");
			return;
		}
		RCLCPP_INFO(get_logger(), "Waiting for action server...");
	}

	// goalメッセージの作成
	auto goal_msg = NavigateToPose::Goal();
	
	rclcpp::Clock ros_clock(RCL_ROS_TIME);
	goal_msg.pose.header.stamp = ros_clock.now();
	goal_msg.pose.header.frame_id = "map";

	goal_msg.pose.pose.position.x = 0.0;
	goal_msg.pose.pose.position.y = 0.0;
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, 0.0);
	goal_msg.pose.pose.orientation.x = q.x();
	goal_msg.pose.pose.orientation.y = q.y();
	goal_msg.pose.pose.orientation.z = q.z();
	goal_msg.pose.pose.orientation.w = q.w();

	// コールバック登録登録構造体
	auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
	send_goal_options.goal_response_callback = std::bind(&NavigationClient::NavGoalResponseCallback, this, _1);
	send_goal_options.feedback_callback = std::bind(&NavigationClient::NavFeedbackCallback, this, _1, _2);
	send_goal_options.result_callback = std::bind(&NavigationClient::NavResultCallback, this, _1);

	// goalをサーバに送信
	m_navigation_client->async_send_goal(goal_msg, send_goal_options);
}

/*=============================================================================
	目標位置移動(アクション)目標位置受信コールバック
=============================================================================*/
void NavigationClient::NavGoalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future)
{
	auto goal_handle = future.get();
	if (goal_handle)	RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    else				RCLCPP_WARN(this->get_logger(), "Goal was rejected by server");
}

/*=============================================================================
	目標位置移動(アクション)途中経過受信コールバック
=============================================================================*/
void NavigationClient::NavFeedbackCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
	RCLCPP_INFO(get_logger(), "Distance remaining = %f", feedback->distance_remaining);
}

/*=============================================================================
	目標位置移動(アクション)実行完了受信コールバック
=============================================================================*/
void NavigationClient::NavResultCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result)
{
	switch (result.code)
	{
		case rclcpp_action::ResultCode::SUCCEEDED:
			RCLCPP_INFO(get_logger(), "Success!!!");
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(get_logger(), "Goal was aborted");
			return;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(get_logger(), "Goal was canceled");
			return;
		default:
			RCLCPP_ERROR(get_logger(), "Unknown result code");
			return;
    }
}

/******************************************************************************
    メイン関数
******************************************************************************/
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<NavigationClient>("navigation_client"));
	rclcpp::shutdown();
	return 0;
}
