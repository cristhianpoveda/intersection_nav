#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <intersection_msgs/MakeDecision.h>
#include <intersection_msgs/DetectUsers.h>
#include "simple_problem.h"

class DecisionNode{
    ros::NodeHandle nh;
    ros::ServiceServer srv_decision;
    ros::ServiceClient client;

    public:
        DecisionNode()
            : nh()
        {
            srv_decision = nh.advertiseService("/srv_decision", &DecisionNode::srv_callback, this);
            client = nh.serviceClient<intersection_msgs::DetectUsers>("/duckiebot4/runtime_detector/detect_users");
        }

        SimpleProblem sp;

        bool srv_callback(intersection_msgs::MakeDecisionRequest &request,
                          intersection_msgs::MakeDecisionResponse &response){

            ros::WallTime start_time, end_time;
            start_time = ros::WallTime::now();

            sp.destination = request.destination.data;
            sp.best.action = 0;            

            int num_of_detections = 0, idx, class_idx, x_idx, y_idx;

            intersection_msgs::DetectUsers detection_service;
            if (client.call(detection_service))
            {
                ROS_INFO("detection succeded");
                num_of_detections = detection_service.response.detections.layout.data_offset;
                for(int i = 0; i < num_of_detections; i++){

                    class_idx = 3 * i;
                    x_idx = class_idx + 1;
                    y_idx = class_idx + 2;

                    sp.o.user = int(detection_service.response.detections.data[class_idx]);
                    SimpleProblem::Point observed;
                    observed.x = detection_service.response.detections.data[x_idx] - request.stop_dist.data;
                    observed.y = detection_service.response.detections.data[y_idx];

                    ROS_INFO("x_p: %f", observed.x);
                    ROS_INFO("y_p: %f", observed.y);

                    if(sp.o.user == 1){
                        sp.get_duckie_group(observed);
                    }else{
                        sp.project_onto_trajectory(observed);
                    }

                    sp.solve();

                    if(sp.best.action == 1){
                        break;
                    }
                }
                sp.best.expected_utility = -10;
                sp.row = 1;
            }
            else
            {
                ROS_ERROR("Failed to call service detect_users");
                return 1;
            }

            end_time = ros::WallTime::now();
            float excecution_time = (end_time - start_time).toNSec() * 1e-6;

            ROS_INFO("excecution time: %f", excecution_time);

            response.decision.data = sp.best.action;

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
