#include <chrono>
#include <cstdio>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <map>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "RFT_UART_SAMPLE.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// Global Variables
std::vector<CRT_RFT_UART> RFT_SENSOR; // 센서 개수에 따라 동적

class RFTSensorSerial : public rclcpp::Node {
public:
    RFTSensorSerial() : Node("rft_sensor_serial") {
        // 주파수 맵 (Hz -> freq_code)
        read_fqs_ = { {10,1},{20,2},{50,3},{100,4},{200,5},{333,6},{500,7},{1000,8} };
        filter_fqs_ = { {500,1},{300,2},{200,3},{150,4},{100,5},{100,6},{50,6},{40,7},{30,8},{20,9},{10,10},{5,11},{3,12},{2,13},{1,14} };
        baud_params_ = { {115200,B115200},{921600,B921600},{460800,B460800},{230400,B230400},{57600,B57600} };

        RCLCPP_INFO(this->get_logger(), "Initializing ROBOTOUS sensors");

        // 센서 개수만큼 DEV_NAME 동적 선언
        this->declare_parameter<std::vector<std::string>>("DEV_NAMES");
        this->get_parameter("DEV_NAMES", dev_names_);
        sensor_count_ = dev_names_.size();

        if (sensor_count_ == 0) {
            RCLCPP_ERROR(this->get_logger(), "No DEV_NAMES provided!");
            rclcpp::shutdown();
            return;
        }

        // 나머지 파라미터 선언
        this->declare_parameter<int>("BAUD");
        this->declare_parameter<float>("FORCE_DIVIDER");
        this->declare_parameter<float>("TORQUE_DIVIDER");
        this->declare_parameter<int>("FREQUENCY", 200);
        this->declare_parameter<bool>("SET_CUTOFF_FILTER");
        this->declare_parameter<int>("FILTER_FREQUENCY");

        this->get_parameter("BAUD", baud_rate_);
        this->get_parameter("FORCE_DIVIDER", force_divider_);
        this->get_parameter("TORQUE_DIVIDER", torque_divider_);
        this->get_parameter("FREQUENCY", read_freq_);
        this->get_parameter("SET_CUTOFF_FILTER", cutoff_filter_);
        this->get_parameter("FILTER_FREQUENCY", filter_freq_);

        RCLCPP_INFO(this->get_logger(), "Baud-rate: %d", baud_rate_);
        RCLCPP_INFO(this->get_logger(), "Force Divider: %f", force_divider_);
        RCLCPP_INFO(this->get_logger(), "Torque Divider: %f", torque_divider_);
        RCLCPP_INFO(this->get_logger(), "Read/Publish Frequency %d", read_freq_);
        RCLCPP_INFO(this->get_logger(), "Cutoff filter: %s", cutoff_filter_ ? "True" : "False");
        RCLCPP_INFO(this->get_logger(), "Filter frequency: %d", filter_freq_);

        bias_status_ = false;
        check_params();

        // 센서 객체 생성
        RFT_SENSOR = std::vector<CRT_RFT_UART>(sensor_count_);
        publisher_ = std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr>(sensor_count_);

        // 센서 초기화 + publisher 생성
        for (int i = 0; i < sensor_count_; i++) {
            RCLCPP_INFO(this->get_logger(), "Initializing sensor %d", i + 1);
            bool success = init_sensor(i);
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "RFT Sensor %d startup error.", i + 1);
                rclcpp::shutdown();
                return;
            }

            std::string topic_name = "RFT_FORCE" + std::to_string(i + 1);
            publisher_[i] = this->create_publisher<geometry_msgs::msg::WrenchStamped>(topic_name, 10);
        }

        // Bias service
        bias_service_ = this->create_service<std_srvs::srv::SetBool>(
            "Set_bias",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                std::unique_lock<std::mutex> lock(com_port_mutex_);
                bool status = true;

                for (int i = 0; i < sensor_count_; i++) {
                    bool ok = RFT_SENSOR[i].set_FT_Bias(request->data ? 1 : 0);
                    status &= ok;
                }

                response->success = status;
                response->message = status ? "Bias set Success" : "Bias Control Failed";
                RCLCPP_INFO(this->get_logger(), "Bias service called: request=%s, result=%s",
                            request->data ? "true" : "false", response->message.c_str());
            });

        // FT Output Control service
        ft_control_service_ = this->create_service<std_srvs::srv::SetBool>(
            "Ft_continuous",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                std::unique_lock<std::mutex> lock(com_port_mutex_);
                bool status = true;

                for (int i = 0; i < sensor_count_; i++) {
                    bool ok = request->data ?
                              RFT_SENSOR[i].rqst_FT_Continuous() :
                              RFT_SENSOR[i].rqst_FT_Stop();
                    status &= ok;
                }

                response->success = status;
                response->message = status ? "FT Output Control OK" : "FT Output Control Fail";
                RCLCPP_INFO(this->get_logger(), "FT Control service called: request=%s, result=%s",
                            request->data ? "true" : "false", response->message.c_str());
            });

        // LPF service
        lpf_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "Set_filter",
            [this](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                   std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
                std::unique_lock<std::mutex> lock(com_port_mutex_);
                int sub_type = request->a;
                bool status = true;

                for (int i = 0; i < sensor_count_; i++) {
                    bool ok = RFT_SENSOR[i].set_FT_Filter_Type(1, sub_type);
                    status &= ok;
                }

                response->sum = status ? sub_type : -1;
                RCLCPP_INFO(this->get_logger(), "Set filter request=%d, result=%s",sub_type, status ? "SUCCESS" : "FAIL");
            });
            
            
             // baudrate service
        baudrate_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "Set_baudrate",
            [this](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                   std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
                std::unique_lock<std::mutex> lock(com_port_mutex_);
                int sub_type = request->a;
                bool status = true;

                for (int i = 0; i < sensor_count_; i++) {
                    bool ok = RFT_SENSOR[i].set_Comm_Speed(sub_type);
                    status &= ok;
                }

                response->sum = status ? sub_type : -1;
                //RCLCPP_INFO(this->get_logger(), "baudrate set status: %d", status);
                RCLCPP_INFO(this->get_logger(), "Set baudrate request=%d, result=%s",sub_type, status ? "SUCCESS" : "FAIL");

            });

        // Frequency service (Hz 직접 설정)
        frequency_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "Set_data_output_rate",
            [this](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                   std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
                std::unique_lock<std::mutex> lock(com_port_mutex_);
                int sub_type = request->a;
                bool status = true;

                

                for (int i = 0; i < sensor_count_; i++) {
                    bool ok = RFT_SENSOR[i].set_FT_Cont_Interval(sub_type);
                    if (!ok) {
                        RCLCPP_ERROR(this->get_logger(), "Sensor %d failed to set frequency %d ", i+1, sub_type);
                    }
                    status &= ok;
                }

                response->sum = status ? sub_type : -1;
                RCLCPP_INFO(this->get_logger(), "Set output rate request=%d, result=%s",sub_type, status ? "SUCCESS" : "FAIL");
            });
            
   read_model_name_ = this->create_service<std_srvs::srv::SetBool>(
    "Read_model_name",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::unique_lock<std::mutex> lock(com_port_mutex_);

        if (!request->data) {
            return;   // ✅ 아무 것도 안 함
        }
/*
        // ✅ 1. 수신 플래그 초기화
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].m_bIsRcvd_Response_Pkt = false;
        }
*/
        // ✅ 2. 모든 센서에 모델명 요청
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].rqst_ProductName();
        }

        // ✅ 3. 그냥 일정 시간 대기 (판정 없음)
        usleep(300000);   // 300ms 고정 대기

        // ✅ 4. 받은 것만 출력 (success 판단 없음)
        for (int i = 0; i < sensor_count_; i++) {
            if (RFT_SENSOR[i].m_bIsRcvd_Response_Pkt) {
                std::string model =
                    RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvd_product_name;

                RCLCPP_INFO(this->get_logger(),
                    "Sensor%d Model Name: %s",
                    i + 1, model.c_str());
            }
        }

        // ✅ 5. 서비스 응답도 의미 없이 true 고정
        response->success = true;
        response->message = "Model name printed";
    });
    
    read_serial_number_ = this->create_service<std_srvs::srv::SetBool>(
    "Read_serial_number",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::unique_lock<std::mutex> lock(com_port_mutex_);

        if (!request->data) {
            return;   // ✅ 아무 것도 안 함
        }
/*
        // ✅ 1. 수신 플래그 초기화
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].m_bIsRcvd_Response_Pkt = false;
        }
*/
        // ✅ 2. 모든 센서에 모델명 요청
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].rqst_SerialNumber();
        }

        // ✅ 3. 그냥 일정 시간 대기 (판정 없음)
        usleep(300000);   // 300ms 고정 대기

        // ✅ 4. 받은 것만 출력 (success 판단 없음)
        for (int i = 0; i < sensor_count_; i++) {
            if (RFT_SENSOR[i].m_bIsRcvd_Response_Pkt) {
                std::string model =
                    RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvd_serial_number;

                RCLCPP_INFO(this->get_logger(),
                    "Sensor%d serial number: %s",
                    i + 1, model.c_str());
            }
        }

        // ✅ 5. 서비스 응답도 의미 없이 true 고정
        response->success = true;
        response->message = "serial number printed";
    });
    
    read_firmware_version_ = this->create_service<std_srvs::srv::SetBool>(
    "Read_firmware_version",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::unique_lock<std::mutex> lock(com_port_mutex_);

        if (!request->data) {
            return;   // ✅ 아무 것도 안 함
        }
/*
        // ✅ 1. 수신 플래그 초기화
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].m_bIsRcvd_Response_Pkt = false;
        }
*/
        // ✅ 2. 모든 센서에 모델명 요청
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].rqst_Firmwareverion();
        }

        // ✅ 3. 그냥 일정 시간 대기 (판정 없음)
        usleep(300000);   // 300ms 고정 대기

        // ✅ 4. 받은 것만 출력 (success 판단 없음)
        for (int i = 0; i < sensor_count_; i++) {
            if (RFT_SENSOR[i].m_bIsRcvd_Response_Pkt) {
                std::string model =
                    RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvd_firmware_version;

                RCLCPP_INFO(this->get_logger(),
                    "Sensor%d firmware version: %s",
                    i + 1, model.c_str());
            }
        }

        // 
        response->success = true;
        response->message = "firmware version printed";
    });
    
   read_baud_rate_ = this->create_service<std_srvs::srv::SetBool>(
"Read_baud_rate",
[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
       std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    std::unique_lock<std::mutex> lock(com_port_mutex_);

    if (!request->data) {
        response->success = false;
        response->message = "request false";
        return;
    }
/*   // ✅ 1. flag 초기화 (딱 한 번만)
    for (int i = 0; i < sensor_count_; i++) {
        RFT_SENSOR[i].m_bIsRcvd_Response_Pkt = false;
    }
*/
    // ✅ 2. 요청 송신
    for (int i = 0; i < sensor_count_; i++) {
        RFT_SENSOR[i].rqst_CommSpeed();
    }

    // ✅ 3. 최대 300ms 폴링 대기
    bool any = false;
    for (int t = 0; t < 30; t++) {
        for (int i = 0; i < sensor_count_; i++) {
            if (RFT_SENSOR[i].m_bIsRcvd_Response_Pkt) {
                any = true;
                break;
            }
        }
        if (any) break;
        usleep(10000); // 10ms
    }

     response->success = true;
        response->message = "read baudrate printed";
});


    
    
    read_filter_setting_ = this->create_service<std_srvs::srv::SetBool>(
    "Read_filter_setting",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::unique_lock<std::mutex> lock(com_port_mutex_);

        if (!request->data) {
            return;   // ✅ 아무 것도 안 함
        }
/*       // ✅ 1. 수신 플래그 초기화
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].m_bIsRcvd_Response_Pkt = false;
        }
*/
        // ✅ 2. 모든 센서에 모델명 요청
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].rqst_FT_Filter_Type();
        }

        // ✅ 3. 그냥 일정 시간 대기 (판정 없음)
        usleep(300000);   // 300ms 고정 대기


        // 
        response->success = true;
        response->message = "read filter printed";
    });
    
    
    read_data_output_rate_ = this->create_service<std_srvs::srv::SetBool>(
    "Read_data_output_rate",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::unique_lock<std::mutex> lock(com_port_mutex_);

        if (!request->data) {
            return;   // ✅ 아무 것도 안 함
        }
/*       // ✅ 1. 수신 플래그 초기화
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].m_bIsRcvd_Response_Pkt = false;
        }
*/
        // ✅ 2. 모든 센서에 모델명 요청
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].rqst_FT_Cont_Interval();
        }

        // ✅ 3. 그냥 일정 시간 대기 (판정 없음)
        usleep(300000);   // 300ms 고정 대기


        // 
        response->success = true;
        response->message = "read frequency printed";
    });
    
    read_ft_overloadcnt_ = this->create_service<std_srvs::srv::SetBool>(
    "Read_ft_overloadcnt",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::unique_lock<std::mutex> lock(com_port_mutex_);

        if (!request->data) {
            return;   // ✅ 아무 것도 안 함
        }
/*       // ✅ 1. 수신 플래그 초기화
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].m_bIsRcvd_Response_Pkt = false;
        }
*/
        // ✅ 2. 모든 센서에 모델명 요청
        for (int i = 0; i < sensor_count_; i++) {
            RFT_SENSOR[i].rqst_FT_OverloadCnt();
        }

        // ✅ 3. 그냥 일정 시간 대기 (판정 없음)
        usleep(300000);   // 300ms 고정 대기


        // 
        response->success = true;
        response->message = "read overloadcnt printed";
    });


        // 타이머 (센서 출력 주기보다 조금 빠르게)
        timer_ = this->create_wall_timer(
            2ms,
            std::bind(&RFTSensorSerial::timer_callback, this));
    }

private:
    void timer_callback() {
        std::unique_lock<std::mutex> lock(com_port_mutex_);

        for (int i = 0; i < sensor_count_; i++) {
            bool isNewData = RFT_SENSOR[i].readWorker();
            if (isNewData && (RFT_SENSOR[i].m_nCurrMode == CMD_FT_CONT)) {
                auto msg = geometry_msgs::msg::WrenchStamped();
                msg.header.stamp = this->now();
                msg.wrench.force.x = RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvdForce[0];
                msg.wrench.force.y = RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvdForce[1];
                msg.wrench.force.z = RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvdForce[2];
                msg.wrench.torque.x = RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvdForce[3];
                msg.wrench.torque.y = RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvdForce[4];
                msg.wrench.torque.z = RFT_SENSOR[i].m_RFT_IF_PACKET.m_rcvdForce[5];
                publisher_[i]->publish(msg);
            }
        }
    }

    bool init_sensor(int idx) {
        DWORD baud = (DWORD)baud_reg_;
        DWORD byte_size = CS8;

        if (!RFT_SENSOR[idx].openPort((char*)dev_names_[idx].c_str(), 0, baud, byte_size)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open interface: %s", dev_names_[idx].c_str());
            return false;
        }

        RFT_SENSOR[idx].m_RFT_IF_PACKET.setDivider(force_divider_, torque_divider_);
        RFT_SENSOR[idx].rqst_FT_Stop();
        usleep(50000);
        RFT_SENSOR[idx].discardInput();

        auto it = read_fqs_.find(read_freq_);
        if (it != read_fqs_.end()) {
            int freq_code = it->second;
            bool ok = RFT_SENSOR[idx].set_FT_Cont_Interval(freq_code);

            if (!ok) {
                RCLCPP_ERROR(this->get_logger(), "Sensor %d did not respond to frequency command!", idx + 1);
            }

            int timeout_cnt = 0;
            const int timeout_limit = 1000;
            while (!RFT_SENSOR[idx].m_bIsRcvd_Response_Pkt && timeout_cnt < timeout_limit) {
                RFT_SENSOR[idx].readWorker();
                usleep(1000);
                timeout_cnt++;
            }

            if (RFT_SENSOR[idx].m_bIsRcvd_Response_Pkt) {
                RCLCPP_INFO(this->get_logger(), "Sensor %d frequency set to %d Hz (freq_code=%d)", idx + 1, read_freq_, freq_code);
            } else {
                RCLCPP_WARN(this->get_logger(), "Sensor %d did not acknowledge frequency command", idx + 1);
            }
        }

        if (!RFT_SENSOR[idx].rqst_FT_Continuous()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start continuous output for sensor %d", idx + 1);
        }

        return true;
    }

    void check_params() {
        if (baud_params_.find(baud_rate_) != baud_params_.end()) {
            baud_reg_ = baud_params_.at(baud_rate_);
        } else {
            baud_rate_ = 115200;
            baud_reg_ = B115200;
            RCLCPP_INFO(this->get_logger(), "Invalid baud rate, reset to %d", baud_rate_);
        }
    }

    // Parameters
    std::vector<std::string> dev_names_;
    int sensor_count_;
    int baud_rate_;
    int baud_reg_;
    float force_divider_;
    float torque_divider_;
    int read_freq_;
    int filter_freq_;
    bool cutoff_filter_;
    bool bias_status_;

    std::map<int, int> read_fqs_;
    std::map<int, int> filter_fqs_;
    std::map<int, int> baud_params_;

    std::mutex com_port_mutex_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr bias_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ft_control_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr read_model_name_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr read_serial_number_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr read_firmware_version_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr read_baud_rate_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr read_filter_setting_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr read_data_output_rate_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr read_ft_overloadcnt_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr lpf_service_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr baudrate_service_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr frequency_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RFTSensorSerial>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
