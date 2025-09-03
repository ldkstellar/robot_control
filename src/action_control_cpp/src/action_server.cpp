#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include"control_interface/action/control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class ControlActionServer:public rclcpp::Node{
    public:
        using control = control_interface::action::Control;
        using handleControl = rclcpp_action::ServerGoalHandle<control>;
        explicit ControlActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):Node("controlServer",options){
             using namespace std::placeholders;
            action_server = rclcpp_action::create_server<control>(this,"control",
            std::bind(&ControlActionServer::handle_goal,this,_1,_2),
            std::bind(&ControlActionServer::handle_cancel,this,_1),
            std::bind(&ControlActionServer::handle_accepted,this,_1));

        }
    private:
        rclcpp_action::Server<control>::SharedPtr action_server;

        rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const control::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->depth);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<handleControl> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<handleControl>goal_handle){
        using namespace std::placeholders;
         std::thread{std::bind(&ControlActionServer::execute, this, _1), goal_handle}.detach();// background  
    }

    void execute(const std::shared_ptr<handleControl>goal_handle){
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback =std::make_shared<control::Feedback>();
        auto sequence = feedback->partial_sequence;
        auto result = std::make_shared<control::Result>();
        
        for (int i = 1; i < goal->depth &&rclcpp::ok(); i++)
        {
            if (i%2 ==1)
            {
                sequence.push_back(-1);// 홀수가 발생하면 feedback으로 보낸다.
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback");
                
            }
            else
            {
                sequence.push_back(1); 
            }
            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } 
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(ControlActionServer)
