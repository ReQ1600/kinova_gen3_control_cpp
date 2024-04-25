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
#include <moveit/move_group_interface/move_group_interface.h>
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
        using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

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

            //creating moveit interface
            

            RCLCPP_INFO(this->get_logger(), "Server alive");
        }
        
        //variables
        private:
            rclcpp_action::Server<MoveArmEffector>::SharedPtr action_server_;
            std::shared_ptr<MoveGroupInterface> move_group_interface_;

        //functions
        private:
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
                rclcpp::Logger lgr = this->get_logger();

                RCLCPP_INFO(lgr, "Executing goal");
                rclcpp::Rate loop_rate(1);

                //move group interface must cant be created in the constructor so its created here
                if(move_group_interface_ == nullptr) move_group_interface_ = std::make_shared<MoveGroupInterface>(MoveGroupInterface(shared_from_this(), "manipulator"));

                //setting up message variables
                const auto goal = goal_handle->get_goal();

                std::shared_ptr<MoveArmEffector::Feedback> feedback = std::make_shared<MoveArmEffector::Feedback>();
                auto& current_effector_position = feedback->current_effector_position;
                auto& error = feedback->error;
                error = "";
                std::shared_ptr<MoveArmEffector::Result> result = std::make_shared<MoveArmEffector::Result>();

                geometry_msgs::msg::Pose pose;
                pose.position = goal->goal_point;
                move_group_interface_->setPoseTarget(pose);

                //creating a plan to target pose
                auto const [success, plan] = [&move_group_interface = move_group_interface_]{
                    moveit::planning_interface::MoveGroupInterface::Plan msg;
                    auto const ok = static_cast<bool>(move_group_interface->plan(msg));
                    return std::make_pair(ok, msg);
                }();

                if(success)
                {
                    move_group_interface_->asyncExecute(plan);
                }
                else
                {
                    result->success = false;
                    feedback->current_effector_position = move_group_interface_->getCurrentPose().pose.position;
                    error = "Could't create a path to given point.";
                    goal_handle->publish_feedback(feedback);
                    goal_handle->abort(result);
                    RCLCPP_INFO(lgr, "Could't create a path");
                    return;
                }

                //kortex doesnt perfectly reach the point most times so precision is needed
                auto const isGoalPointReached = [&move_group_interface = move_group_interface_, pose](float precision){

                    auto& [g_x, g_y, g_z] = pose.position;
                    const auto& [x, y, z] = move_group_interface->getCurrentPose().pose.position;

                    bool out = ((g_x - precision <= x) && (g_x + precision) >= x
                    && (g_y - precision <= y) && (g_y + precision) >= y
                    && (g_z - precision <= z) && (g_z + precision) >= z);

                    return out;
                };

                while (rclcpp::ok() && !isGoalPointReached(0.01))
                {
                    //checking for cancel request
                    if(goal_handle->is_canceling())
                    {
                        result->success = false;
                        goal_handle->canceled(result);
                        feedback->current_effector_position = move_group_interface_->getCurrentPose().pose.position;
                        error = "Goal cancelled.";
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(lgr, "Goal cancelled");
                        return;
                    }

                    current_effector_position = move_group_interface_->getCurrentPose().pose.position;

                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(lgr, "Publishing feedback");

                    loop_rate.sleep();
                }
                
                //check if goal has been reached
                if(rclcpp::ok())
                {
                    result->success = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(lgr, "Goal has been reached");
                }
            }
    };
}
//registering the node
RCLCPP_COMPONENTS_REGISTER_NODE(kinova_gen3_control_cpp::ArmMoveToPointServer)
