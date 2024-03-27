/*
    here i used the trailing underscore member convention 
    and snake case for variables
    if to be ever edited please keep it that way all change all of them
*/

//cpp headers
#include <memory>
#include <thread>


//ros packages headers
#include "rclcpp/rclcpp.hpp"
//#include <moveit/move_group_interface/move_group_interface.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

//my headers
#include "kinova_gen3_control_interfaces/action/move_arm_effector.hpp"
#include "kinova_gen3_control_cpp/visibility_control.h"

namespace kinova_gen3_control_cpp
{
    class ArmMoveToPointServer : public rclcpp::Node
    {
        public:
        using MoveArmEffector = kinova_gen3_control_interfaces::action::MoveArmEffector;
        using GoalHandleMsg = rclcpp_action::ServerGoalHandle<MoveArmEffector>;

        KINOVA_GEN3_CONTROL_CPP_PUBLIC

        //creating the constructor 
        explicit ArmMoveToPointServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("arm_move_to_point_server", options)
        {
            using namespace std::placeholders;
            
            this->action_server_ = rclcpp_action::create_server<MoveArmEffector>(
                this,
                "move_arm_effector",
                std::bind(&ArmMoveToPointServer::handleGoal, this, _1, _2),
                std::bind(&ArmMoveToPointServer::handleCancel, this, _1),
                std::bind(&ArmMoveToPointServer::handleAccepted, this, _1)
            );
            RCLCPP_INFO(this->get_logger(), "Server alive");
        }

        private:
            rclcpp_action::Server<MoveArmEffector>::SharedPtr action_server_;

            //TODO: make it so when the position is out of manipulator workspace it rejects and returns an error for now it only accepts
            rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID, std::shared_ptr<const MoveArmEffector::Goal> goal)
            {
                
                const auto& [x, y, z] = goal->goal_point;
                RCLCPP_INFO(this->get_logger(), "Goal received. Goal: [x: %f, y: %f, z: %f]", x, y, z);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            //will simply accept the request to cancel (twter sim)
            rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleMsg>)
            {
                RCLCPP_INFO(this->get_logger(),"Goal Cancelled");
                return rclcpp_action::CancelResponse::ACCEPT;
            }
            
            //will spin up a new thread cuz blocking the executor is not ideal
            void handleAccepted(const std::shared_ptr<GoalHandleMsg> goal_handle)
            {
                //initializer lists are said to be faster so there you go
                std::thread{std::bind(&ArmMoveToPointServer::execute, this, std::placeholders::_1), goal_handle}.detach();
            }

            void execute(const std::shared_ptr<GoalHandleMsg> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Executing goal");
                rclcpp::Rate loop_rate(1);
                //setting up the message variables
                const auto goal = goal_handle->get_goal();

                std::shared_ptr<MoveArmEffector::Feedback> feedback = std::make_shared<MoveArmEffector::Feedback>();
                auto& current_effector_position = feedback->current_effector_position;
                auto& error = feedback->error;
                auto& [curr_x, curr_y, curr_z] = current_effector_position;
                curr_x = curr_y = curr_z = 0;//placeholder values
                //TODO: current_effector_position
                error = "";
                std::shared_ptr<MoveArmEffector::Result> result = std::make_shared<MoveArmEffector::Result>();
                
                //TODO: create path and start the robot
                //TODO: while loop that checks if the arm is still moveing

                //checking for cancel request
                if(goal_handle->is_canceling())
                {
                    result->success = false;
                    goal_handle->canceled(result);
                    //feedback->current_effector_position = take position from the robot
                    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
                    return;
                }

                //TODO: check if robot sent any errors then update and publish feedback
                //feedback->error=error from the robot;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publishing feedback");

                loop_rate.sleep();
                //end while

                //check if goal has been reached
                if(rclcpp::ok())
                {
                    result->success = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal has been reached");
                }
            }
    };
}
//registering the node
RCLCPP_COMPONENTS_REGISTER_NODE(kinova_gen3_control_cpp::ArmMoveToPointServer)