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
    ros::Publisher pub_time;

    public:
        DecisionNode()
            : nh()
        {
            srv_decision = nh.advertiseService("/srv_decision", &DecisionNode::srv_callback, this);
            client = nh.serviceClient<intersection_msgs::DetectUsers>("/duckiebot4/runtime_detector/detect_users");
            pub_pos = nh.advertise<geometry_msgs::Point32>("/duckiebot4/decision_making_node/debug/projected_pos", 0);
            pub_g_pos = nh.advertise<geometry_msgs::Point32>("/duckiebot4/decision_making_node/debug/projected_geo", 0);
            pub_time = nh.advertise<std_msgs::Float32>("/duckiebot4/decision_making_node/debug/calculation_time", 0);
        }

        SimpleProblem simple_problem;

        bool srv_callback(intersection_msgs::MakeDecisionRequest &request,
                          intersection_msgs::MakeDecisionResponse &response){

            simple_problem.destination = request.destination.data;
            simple_problem.best.action = 0;            

            int num_of_detections = 0, idx, class_idx, x_idx, y_idx;
            float excecution_time;

            intersection_msgs::DetectUsers detection_service;
            if (client.call(detection_service))
            {
                ros::WallTime start_time, end_time;
                start_time = ros::WallTime::now();

                num_of_detections = detection_service.response.detections.layout.data_offset;
                for(int i = 0; i < num_of_detections; i++){

                    class_idx = 3 * i;
                    x_idx = class_idx + 1;
                    y_idx = class_idx + 2;

                    simple_problem.observation.user = int(detection_service.response.detections.data[class_idx]);
                    SimpleProblem::Point observed;
                    observed.x = detection_service.response.detections.data[x_idx] - request.stop_dist.data;
                    observed.y = detection_service.response.detections.data[y_idx];

                    if (pub_g_pos.getNumSubscribers() > 0){

                        geometry_msgs::Point32 g_pos;
                        g_pos.x = observed.x;
                        g_pos.y = observed.y;

                        pub_g_pos.publish(g_pos);
                    }

                    if(simple_problem.observation.user == 1){
                        simple_problem.get_duckie_group(observed);
                    }else{
                        simple_problem.project_onto_trajectory(observed);
                    }

                    if (pub_pos.getNumSubscribers() > 0){

                        geometry_msgs::Point32 pos;
                        pos.x = simple_problem.observation.position_coor.x;
                        pos.y = simple_problem.observation.position_coor.y;

                        pub_pos.publish(pos);
                    }

                    simple_problem.solve();

                    if(simple_problem.best.action == 1){
                        break;
                    }
                }
                simple_problem.best.expected_utility = -10;
                simple_problem.row = 1;
                end_time = ros::WallTime::now();
                excecution_time = (end_time - start_time).toNSec() * 1e-6;
            }
            else
            {
                ROS_ERROR("Failed to call service detect_users");
                return 1;
            }

            if (pub_time.getNumSubscribers() > 0){

                std_msgs::Float32 calculation_time;
                calculation_time.data = excecution_time;

                pub_time.publish(calculation_time);
            }

            response.decision.data = simple_problem.best.action;

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
