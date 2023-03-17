#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <intersection_msgs/MakeDecision.h>
#include <intersection_msgs/DetectUsers.h>
#include "simple_problem.h"

class DecisionNode{
    ros::NodeHandle nh;
    ros::ServiceServer srv_decision;
    ros::ServiceClient client;
    ros::Publisher pub_pos;
    ros::Publisher pub_g_pos;
    ros::Publisher pub_comparison;

    public:
        DecisionNode()
            : nh()
        {
            srv_decision = nh.advertiseService("/srv_decision", &DecisionNode::srv_callback, this);
            client = nh.serviceClient<intersection_msgs::DetectUsers>("/duckiebot4/runtime_detector/detect_users");
            pub_pos = nh.advertise<geometry_msgs::Point32>("/duckiebot4/decision_making_node/debug/projected_pos", 0);
            pub_g_pos = nh.advertise<geometry_msgs::Point32>("/duckiebot4/decision_making_node/debug/projected_geo", 0);
            pub_comparison = nh.advertise<geometry_msgs::Point32>("/duckiebot4/decision_making_node/debug/comparison", 0);
        }

        SimpleProblem simple_problem;

        // constructor trajectories

        bool srv_callback(intersection_msgs::MakeDecisionRequest &request,
                          intersection_msgs::MakeDecisionResponse &response){

            ros::WallTime start_time, end_time, start_mulrtivariate, end_multivariate;

            simple_problem.destination = request.destination.data;
            simple_problem.best.action = 0;            

            int num_of_detections = 0, idx, class_idx, x_idx, y_idx, multivariate_action = 0, multivariate_selection = 0, probable_projections = 0, decision_1 = 0, decision_2 = 0, final_decision = 0;
            float excecution_time, multivariate_time = 0, row_1 = 1, row_2 = 1;

            intersection_msgs::DetectUsers detection_service;
            if (client.call(detection_service))
            {

                num_of_detections = detection_service.response.detections.layout.data_offset;
                for(int i = 0; i < num_of_detections; i++){

                    simple_problem.best.expected_utility = -10;
                    simple_problem.row = 1;
                    simple_problem.projected_user.x = 10;
                    simple_problem.projected_user.y = 10;
                    simple_problem.primary_projection.pos.x = 10;
                    simple_problem.primary_projection.pos.y = 10;
                    simple_problem.second_projection.pos.x = 10;
                    simple_problem.second_projection.pos.y = 10;

                    row_1 = 1;
                    row_2 = 1;

                    class_idx = 3 * i;
                    x_idx = class_idx + 1;
                    y_idx = class_idx + 2;

                    simple_problem.user = int(detection_service.response.detections.data[class_idx]);
                    simple_problem.projected_user.x = detection_service.response.detections.data[x_idx] - request.stop_dist.data;
                    simple_problem.projected_user.y = detection_service.response.detections.data[y_idx];

                    if (pub_g_pos.getNumSubscribers() > 0){

                        geometry_msgs::Point32 g_pos;
                        g_pos.x = simple_problem.projected_user.x;
                        g_pos.y = simple_problem.projected_user.y;

                        pub_g_pos.publish(g_pos);
                    }

                    start_time = ros::WallTime::now();

                    if(simple_problem.user == 1){

                        probable_projections = simple_problem.new_project_to_crosswalk();

                    }else{

                        ROS_INFO("project trajectory");
                        probable_projections = simple_problem.new_project_to_trajectory();
                    }

                    ROS_INFO("Probable proj: %d", probable_projections);

                    if (pub_pos.getNumSubscribers() > 0){

                        geometry_msgs::Point32 context_projection;

                        context_projection.x = simple_problem.primary_projection.pos.x;
                        context_projection.y = simple_problem.primary_projection.pos.y;
                        pub_pos.publish(context_projection);

                        context_projection.x = simple_problem.second_projection.pos.x;
                        context_projection.y = simple_problem.second_projection.pos.y;
                        pub_pos.publish(context_projection);
                    }

                    row_1 = simple_problem.new_bayesian_network(simple_problem.primary_projection);
                    ROS_INFO("row 1: %f", row_1);

                    if(probable_projections > 1){
                        row_2 = simple_problem.new_bayesian_network(simple_problem.second_projection);
                        ROS_INFO("row 2: %f", row_2);
                    }

                    simple_problem.row = row_1;
                    simple_problem.new_solve();
                    decision_1 = simple_problem.best.action;
                    ROS_INFO("Decision 1: %i", simple_problem.best.action);

                    simple_problem.row = row_2;
                    simple_problem.new_solve();
                    decision_2 = simple_problem.best.action;
                    ROS_INFO("Decision 2: %i", simple_problem.best.action);

                    final_decision = decision_1 * decision_2;
                    ROS_INFO("Final: %d", final_decision);

                    end_time = ros::WallTime::now();
                    excecution_time = (end_time - start_time).toNSec() * 1e-6;

                    start_mulrtivariate = ros::WallTime::now();

                    multivariate_action = simple_problem.solveMultivariate(simple_problem.projected_user);

                    if(multivariate_action == 1){
                        multivariate_selection = multivariate_action;
                    }

                    end_multivariate = ros::WallTime::now();

                    multivariate_time += (end_multivariate - start_mulrtivariate).toNSec() * 1e-6;

                    if(final_decision == 0){
                        break;
                    }
                }
                
            }
            else
            {
                ROS_ERROR("Failed to call service detect_users");
                return 1;
            }

            end_time = ros::WallTime::now();
            excecution_time = (end_time - start_time).toNSec() * 1e-6;

            if (pub_comparison.getNumSubscribers() > 0){

                geometry_msgs::Point32 comparison_data;
                comparison_data.x = excecution_time;
                comparison_data.y = multivariate_time;
                comparison_data.z = multivariate_selection;

                pub_comparison.publish(comparison_data);
            }

            ROS_INFO("Response: %d", final_decision);
            response.decision.data = final_decision;

            return true;
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decision_making_node");

  DecisionNode node;
  
  ROS_INFO("Decision node started");
  ros::spin();
  return 0;
}
