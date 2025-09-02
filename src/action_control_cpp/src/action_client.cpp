#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "rclcpp.hpp"
#include"control_interface/action/control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class ControlActionClient:rclcpp::Node{
    public:
        using control = control_interface::action::Control;
        using handleControl = rclcpp_action::ClientGoalHandle<control>;

       explicit ControlActionClient(const rclcpp::NodeOptions & options):Node("controlClient",options){
            this->client_ptr = rclcpp_action::create_client<control>(this,"actuator");
            this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&ControlActionClient::send_goal,this));
        }

        void send_goal(){
            using namespace std::placeholders;
            this->timer_->cancel();
            if(!this->client_ptr->wait_for_action_server()){
                RCLCPP_ERROR(this->get_logger(),"Action server is not available");
                rclcpp::shutdown();
            }
            auto goal = control::Goal();
            goal.depth = 11;
            RCLCPP_INFO(this->get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<control>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&ControlActionClient::goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&ControlActionClient::feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&ControlActionClient::result_callback, this, _1);
            this->client_ptr->async_send_goal(goal, send_goal_options);
        }

        
    private:
        void goal_response_callback(const handleControl::SharedPtr & goal_handle){
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Goal accpetd by server wait!");
            }
            
        }
        void feedback_callback(handleControl::SharedPtr,const std::shared_ptr<const control::Feedback>feedback){
            std::stringstream ss;
            ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
      // 공유포인터로 전달 받을수있고 벡터로도 전달 받기 가능하다.
      
    }
    modifiedData = std::move(feedback->partial_sequence);//스택의 값을 전달한다.
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        }
        void result_callback(const handleControl::WrappedResult &result){
            switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }


    }

        rclcpp_action::Client<control>::SharedPtr client_ptr;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<int>modifiedData;
        std::vector<int> result;

};
