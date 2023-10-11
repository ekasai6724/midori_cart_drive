#include "midori_cart_drive.h"

/******************************************************************************
    コンストラクタ
******************************************************************************/
MidoriCartDrive::MidoriCartDrive(const std::string &topic_name):Node(topic_name)
{
    QTextStream qso(stdout);
    using namespace std::chrono_literals;

    // パラメータ宣言
    this->declare_parameter("gear_rate", 1.0);
    this->declare_parameter("num_of_teeth1", 24.0);
    this->declare_parameter("num_of_teeth2", 72.0);
    this->declare_parameter("tire_diameter", 205.0);
    this->declare_parameter("tred_width", 500.0);
    qso << "Paramter declared." << endl;

    // Launchファイルで設定されたパラメータデータの取得
#if 1
    for(auto & parameter : this->get_parameters
        ({"gear_rate", "num_of_teeth1", "num_of_teeth2", "tire_diameter", "tred_width"}))
    {
        std::stringstream ss;
        ss << "Name: " << parameter.get_name() << ", Value (" << parameter.get_type_name() << "): " << parameter.value_to_string();
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
    this->get_parameter("gear_rate",     m_GearRate     );
    this->get_parameter("num_of_teeth1", m_NumTeeth1    );
    this->get_parameter("num_of_teeth2", m_NumTeeth2    );
    this->get_parameter("tire_diameter", m_TireDiameter );
    this->get_parameter("tred_width",    m_TredWidth    );
#endif

    // USBシリアルポートオープン
    SetupUSBport(); 
    if(SerialPortsOpen())
    {
        qso << "Both serial ports open." << endl;

        // サブスクライバ作成して受信コールバック関数を登録
        m_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::QoS(10),
            std::bind(&MidoriCartDrive::SubscriptionEvent, this, std::placeholders::_1)
        );

        // 周期タイマ作成してコールバック関数を登録
        m_timer = this->create_wall_timer(
            50ms,
            std::bind(&MidoriCartDrive::CyclicTimerEvent, this)
        );
    }
    else
    {
        qso << "Counldn't open the port." << endl;
    }
}

/******************************************************************************
    デストラクタ
******************************************************************************/
MidoriCartDrive::~MidoriCartDrive()
{
    QThread::msleep(100);
}

/*=============================================================================
    USBシリアルポート検索して接続準備
=============================================================================*/
void MidoriCartDrive::SetupUSBport(void)
{
    //QTextStream qstdout(stdout);

    m_TmpPorts.clear();
    foreach(const QSerialPortInfo info, QSerialPortInfo::availablePorts())
    {
        if(info.portName().left(6) == "ttyACM")
        {
            //qstdout << "Name        :" << info.portName() << endl;
            //qstdout << "Description :" << info.description() << endl;
            //qstdout << "Manufacturer:" << info.manufacturer() << endl;

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
    QTextStream so(stdout);
    int cnt = 0;

    foreach(QSerialPort *port , m_TmpPorts)
    {
        if(port->open(QIODevice::ReadWrite))
        {
            so << "Port: " << port->portName() << " open." << endl;

            SerialPortSend(port, "9A;PR;14");   // N0020「軸番号」読出しコマンド
            QByteArray recv;
             while(port->waitForReadyRead(500))
             {
                 if(port->bytesAvailable() > 0)
                 {
                     recv += port->readAll();
                 }
                 if((recv.indexOf(0x03) != -1) || (recv.indexOf(0x04) != -1))    break;
             }
            so << "[recv]:" << recv << endl;

            if      (recv.mid(7, 8) == "00000000")  { m_SerialPort_R = port; cnt++; }   // 軸番号0:右車輪
            else if (recv.mid(7, 8) == "00000001")  { m_SerialPort_L = port; cnt++; }   // 軸番号1:左車輪

            SerialPortSend(port, "9A;SVON");
        }
    }
    if(cnt == 2)    return true;
    else            return false;
}

/*=============================================================================
    サブスクライバ受信イベントハンドラ
=============================================================================*/
void MidoriCartDrive::SubscriptionEvent(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    //QTextStream qstdout(stdout);
    //qstdout << "Subscriber event" << endl;

    // 受信したL/A指令をメンバ変数に保存するだけ
    m_CmdLinear = msg->linear.x;
    m_CmdAngular = msg->angular.z;
}

/*=============================================================================
    周期タイマイベントハンドラ(TBDms)
=============================================================================*/
void MidoriCartDrive::CyclicTimerEvent(void)
{
#if 0
    m_GearRate = 1.0;
    m_NumTeeth1 = 24.0;
    m_NumTeeth2 = 72.0;
    m_TireDiameter = 205.0;
    m_TredWidth = 500.0;
#endif
    // 速度指令作成
    double  gear_rate = m_GearRate * m_NumTeeth1 / m_NumTeeth2;     // モータ軸から車輪軸までの減速比
    double  vl, va;
    
    vl = m_CmdLinear * 1000 / (m_TireDiameter * 3.14) / gear_rate * 60;
    va = m_CmdAngular * (m_TredWidth / 2 / 1000) * 1000 / (m_TireDiameter * 3.14) / gear_rate * 60;
    int vcom_R = vl + va,
        vcom_L = vl - va;

    //QTextStream so(stdout);
    //so << "linear.x = " << QString::number(m_CmdLinear) << ", angular.z = " << QString::number(m_CmdAngular) << endl;
    //so << "vl = " << QString::number(vl) << ", va = " << QString::number(va) << endl;

    // 速度指令シリアル通信コマンド送信
    QByteArray  sbf;
    sbf.append("9A;VREF;");
    sbf.append(QString::number(vcom_R, 16).rightJustified(8, '0').toLatin1());
    SerialPortSend(m_SerialPort_R, sbf);
    sbf.clear();
    sbf.append("9A;VREF;");
    sbf.append(QString::number(vcom_L, 16).rightJustified(8, '0').toLatin1());
    SerialPortSend(m_SerialPort_L, sbf);
}

/*=============================================================================
    USBシリアルポート送信
=============================================================================*/
void MidoriCartDrive::SerialPortSend(QSerialPort *port, QByteArray send)
{
    QByteArray  sbf;

    sbf.append(0x02);   // STX
    sbf.append(send);
    sbf.append(0x03);   // ETX

    port->write(sbf);

    //QTextStream qstdout(stdout);
    //qstdout << "send : " << sbf << endl;
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
    rclcpp::shutdown();
}