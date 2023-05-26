#include <ros/ros.h>
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
        }

        SimpleProblem simple_problem;

        // constructor trajectories

        bool srv_callback(intersection_msgs::MakeDecisionRequest &request,
                          intersection_msgs::MakeDecisionResponse &response){

            simple_problem.destination = request.destination.data;
            simple_problem.best.action = 0;            

            int num_of_detections = 0, idx, class_idx, angle_idx, distance_idx, probable_projections = 0, decision_1 = 0, decision_2 = 0, final_decision = 0;
            float excecution_time, row_1 = 1, row_2 = 1, world_angle = 0, cam_2_base = 0.0582;

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
                    angle_idx = class_idx + 1;
                    distance_idx = class_idx + 2;

                    world_angle = request.stop_pose.orientation.z + detection_service.response.detections.data[angle_idx];

                    simple_problem.user = int(detection_service.response.detections.data[class_idx]);
                    simple_problem.projected_user.x = cam_2_base * cos(request.stop_pose.orientation.z) + detection_service.response.detections.data[distance_idx] * cos(world_angle) - request.stop_pose.position.x;
                    simple_problem.projected_user.y = cam_2_base * cos(request.stop_pose.orientation.z) + detection_service.response.detections.data[distance_idx] * sin(world_angle) - request.stop_pose.position.y;

                    if(simple_problem.user == 1){

                        probable_projections = simple_problem.project_to_crosswalk();

                    }else{

                        probable_projections = simple_problem.project_to_trajectory();
                    }

                    if (pub_pos.getNumSubscribers() > 0){

                        geometry_msgs::Point32 context_projection;

                        context_projection.x = simple_problem.primary_projection.pos.x;
                        context_projection.y = simple_problem.primary_projection.pos.y;
                        pub_pos.publish(context_projection);

                        context_projection.x = simple_problem.second_projection.pos.x;
                        context_projection.y = simple_problem.second_projection.pos.y;
                        pub_pos.publish(context_projection);
                    }

                    row_1 = simple_problem.bayesian_network(simple_problem.primary_projection);

                    if(probable_projections > 1){
                        row_2 = simple_problem.bayesian_network(simple_problem.second_projection);
                    }

                    simple_problem.row = row_1;
                    simple_problem.solve();
                    decision_1 = simple_problem.best.action;

                    simple_problem.row = row_2;
                    simple_problem.solve();
                    decision_2 = simple_problem.best.action;

                    final_decision = decision_1 * decision_2;

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

            ROS_INFO("Response: %d", final_decision);
            response.decision.data = final_decision;

            return true;
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decision_making_node");

  DecisionNode node;
  
  ROS_INFO("decision_making_node node started");
  ros::spin();
  return 0;
}
