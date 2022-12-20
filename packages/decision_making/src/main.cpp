#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_map>
#include <std_srvs/Empty.h>
#include <std_msgs/Int16MultiArray.h>
#include <intersection_msgs/MakeDecision.h>
#include <intersection_msgs/DetectUsers.h>

class SimpleProblem{

    public:

        struct Point{
            float x;
            float y;
        };

        struct Projected_point{
            Point coordinates;
            float projection_distance;
        };

        struct Trajectory_point{
            float mean;
            float projection_distance;
            float reference;
            float trajectory_lenght;
        };

        struct Projected_pos{
            Point position;
            Trajectory_point point;
        };

        struct Observation{
            Point position_coor;
            int user;
            float ANN_value = 1;
            Trajectory_point point;
            int trajectory_group;
            float direction = 1;
        };

        struct Best{
            int action;
            float expected_utility;
        };

        struct Led{
            Point pixel;
            int DB;
        };

        typedef std::unordered_map<int, std::string> Actions;

        // atributes

        // evidence
        Observation o;
        
        // chance var (right of way)
        float row;

        Actions A;

        // decision (action) and utility(u) vars;
        Best best;

        int destination;

        Point nearest_group;

        // utilities;
        float utilities[2][2] = {{0.9, -1}, {-0.1, 1}};
        
        SimpleProblem(){
            row = 1;
            best.expected_utility = -10;
            A[0] = "cross";
            A[1] = "wait";
        }

        // functions

        void min_distance(Point DB_proj, float (*ref_points)[3], int len){
    
            float min_distance = 1, distance;
            o.trajectory_group = 0;
    
            for(int i = 0; i < len; i++){
            
                distance = sqrt(pow((ref_points[i][0] - DB_proj.x), 2) + pow((ref_points[i][1] - DB_proj.y), 2));
            
                if(distance < min_distance){
                    min_distance = distance;
                    o.trajectory_group = i;
                    nearest_group.x = ref_points[i][0];
                    nearest_group.y = ref_points[i][1];
                    o.direction = ref_points[i][2];
                }
            }
        }

        // geometrical projections
        Projected_point line_projection(float parallel_coor, float orthogonal_coor, float line_intercept){
    
            Projected_point Pp;
    
            Pp.coordinates.x = parallel_coor;
            Pp.coordinates.y = line_intercept;
            Pp.projection_distance = abs(orthogonal_coor - line_intercept);

            return Pp;
        }
        
        Projected_pos arc_projection(Point point_1, float h, float k, float r, float axis){

            Projected_pos Pp;
    
            float m = (point_1.y - k) / (point_1.x - h);
            float b_line = -m * point_1.x + point_1.y;
    
            // solve quadratic eq
    
            // because of + 1 in "a" calculation, there is never a discontinuity for the quadratic equation.
            float a = pow(m, 2) + 1;
            float b_quad_eq = 2 * m * (b_line - k) - 2 * h;
            float c = pow((b_line - k),2) - pow(r, 2) + pow(h, 2);
    
            float x1 = (- b_quad_eq + sqrt(pow(b_quad_eq, 2) - 4 * a * c)) / (2 * a);
            float x2 = (- b_quad_eq - sqrt(pow(b_quad_eq, 2) - 4 * a * c)) / (2 * a);
    
            float y1 = m * x1 + b_line;
            float y2 = m * x2 + b_line;
    
            // find projected distance
            float proj_distance_1 = sqrt(pow((x1 - point_1.x), 2) + pow((y1 - point_1.y), 2));
            float proj_distance_2 = sqrt(pow((x2 - point_1.x), 2) + pow((y2 - point_1.y), 2));
    
            if(proj_distance_1 < proj_distance_2){
        
                Pp.position.x = x1;
                Pp.position.y = y1;
                Pp.point.projection_distance = proj_distance_1;
        
            }else{

                Pp.position.x = x2;
                Pp.position.y = y2;
                Pp.point.projection_distance = proj_distance_2;

            }

            if(axis == 0){
                Pp.point.mean = r * acos(abs(Pp.position.y - k) / r);
            }else{
                Pp.point.mean = r * acos(abs(Pp.position.x - h) / r);
            }

            return Pp;   
        }

        // projection duckie
        void get_duckie_group(SimpleProblem::Point D_observed){

            // header
            //0 axis, 1 m, 2 b, 3 len.

            float ref_points_duckie[4][2] = {{0.0, 0.125},{0.25, -0.125},{0.5, 0.125},{0.25, 0.375}};
    
            float trajectories[4][4] = {{1, 0, 0, 0.5},
                                        {0, 0, -0.125, 0.5},
                                        {1, 0, 0.5, 0.5},
                                        {0, 0, 0.375, 0.5}};

            o.point.projection_distance = 1;

            int num_of_trajectories = sizeof(trajectories) / sizeof(trajectories[0]);
            float ref = 0, u = 0;

            Projected_point Pp_line;
            Projected_point Pp_line_corrected;

            for(int i = 0; i < num_of_trajectories; i++){

                if(trajectories[i][0] == 0){

                    Pp_line = line_projection(D_observed.x, D_observed.y, trajectories[i][2]);
                    ref = ref_points_duckie[i][0];
                    u = Pp_line.coordinates.x;

                }else{

                    Pp_line_corrected = line_projection(D_observed.y, D_observed.x, trajectories[i][2]);
                    Pp_line.coordinates.x = Pp_line_corrected.coordinates.y;
                    Pp_line.coordinates.y = Pp_line_corrected.coordinates.x;
                    Pp_line.projection_distance = Pp_line_corrected.projection_distance;
                    ref = ref_points_duckie[i][1];
                    u = Pp_line.coordinates.y;

                }
        
                if(Pp_line.projection_distance < o.point.projection_distance){
                
                    o.position_coor = Pp_line.coordinates;
                    o.point.projection_distance = Pp_line.projection_distance;
                    o.point.mean = u;
                    o.point.reference = ref;
                    o.point.trajectory_lenght = trajectories[i][3];
                    o.trajectory_group = i;

                }
            }
    
        }

        // projection duckiebot

        void project_onto_trajectory(SimpleProblem::Point DB_observed, int detection_class){

            // Parallel axis, intersection limit, right arc, line, left arc.
            // 0 axis, 1 min limit, 2 max limit, 3:5(h1, k1, r1) 6 len, 7:8(m, b) 9 len, 10:12(h2, k2, r2) 13 len.
            // axis: x = 0, y = 1.
            // limit: not considered = -1.
    
            float trajectories[8][14] = {{1, -0.125, 1, 0.5, -0.125, 0.125, 0.39, 0, 0.375, 0.5, 0, -0.125, 0.375, 1.18},
                                        {0, -1, 0.5, 0.5, 0.375, 0.125, 0.39, 0, 0.25, 0.5, 0.5, -0.125, 0.375, 1.18},
                                        {1, -1, 0.375, 0, 0.375, 0.125, 0.39, 0, 0.125, 0.5, 0.5, 0.375, 0.375, 1.18},
                                        {1, -0.125, 1, 0.5, -0.125, 0.375, 1.18, 0, 0.125, 0.5, 0, 0, 10, 10},
                                        {0, -1, 0.5, 0.5, -0.125, 0.125, 0.39, 0, 0, 0.5, 0.5, 0.375, 0.375, 1.18},
                                        {1, -1, 0.375, 0.5, 0.375, 0.125, 0.39, 0, 0.375, 0.5, 0, 0, 10, 10},
                                        {1, -0.125, 0.375, 0.5, -0.125, 0.125, 0.39, 0, 0.375, 0.5, 0.5, 0.375, 0.125, 0.39},
                                        {1, -0.125, 0.375, 0, 0.375, 0.125, 0.39, 0, 0.125, 0.5, 0.5, -0.125, 0.375, 1.18}};

            if(detection_class == 0){
                min_distance(DB_observed, back_ref_points, len_back);
            }else{
                min_distance(DB_observed, front_ref_points, len_front);
            }
    
            Projected_point Pp_line;
    
            Projected_pos Pp_arc_1;
            Projected_pos Pp_arc_2;
    
            float parallel_coor = 0;
            float orthogonal_coor = 0;

            if(trajectories[o.trajectory_group][0] == 0){
                parallel_coor = DB_observed.x;
                orthogonal_coor = DB_observed.y;
                o.point.reference = nearest_group.x;

            }else{
                parallel_coor = DB_observed.y;
                orthogonal_coor = DB_observed.x;
                o.point.reference = nearest_group.y;
            }

            Pp_line = line_projection(parallel_coor, orthogonal_coor, trajectories[o.trajectory_group][8]);
                o.point.trajectory_lenght = trajectories[o.trajectory_group][9];

            if(trajectories[o.trajectory_group][0] == 1){
                o.position_coor.x = Pp_line.coordinates.y;
                o.position_coor.y = Pp_line.coordinates.x;
                
            }else{
                o.position_coor = Pp_line.coordinates;
            }
            o.point.mean = Pp_line.coordinates.x;
            o.point.projection_distance = Pp_line.projection_distance;
    
            if(parallel_coor > trajectories[o.trajectory_group][1] && parallel_coor < trajectories[o.trajectory_group][2]){
            
                // arc1
                Pp_arc_1 = arc_projection(DB_observed, trajectories[o.trajectory_group][3], trajectories[o.trajectory_group][4],
                trajectories[o.trajectory_group][5], trajectories[o.trajectory_group][0]);

                if(Pp_arc_1.point.projection_distance < o.point.projection_distance){
                    o.position_coor = Pp_arc_1.position;
                    o.point.mean = Pp_arc_1.point.mean;
                    o.point.projection_distance = Pp_arc_1.point.projection_distance;
                    o.point.trajectory_lenght = trajectories[o.trajectory_group][6];   
                }
            
                // arc2
                Pp_arc_2 = arc_projection(DB_observed, trajectories[o.trajectory_group][10], trajectories[o.trajectory_group][11],
                trajectories[o.trajectory_group][12], trajectories[o.trajectory_group][0]);

                if(Pp_arc_2.point.projection_distance < o.point.projection_distance){
                    o.position_coor = Pp_arc_2.position;
                    o.point.mean = Pp_arc_2.point.mean;
                    o.point.projection_distance = Pp_arc_2.point.projection_distance;
                    o.point.trajectory_lenght = trajectories[o.trajectory_group][13];
                }
            }
        }

        // probabilistic distributions
        float normal_dist(float x){
    
            //header
            // 3o = 0.05 m
            float std_dev = 0.05/3;
    
            float z = (x - o.point.mean) / std_dev;
            float p = 0.5 * erfc(-z * M_SQRT1_2);
    
            return p;
        }

        void joint_probability_distribution(){
    
            float x1, x2;
    
            // header
            float car_inflation = 0.08, gap = 0.05, projection_lim = 0.1;
    
            if (o.user == 0 && o.point.projection_distance <= projection_lim){

                // Di
                if(o.trajectory_group == 0 || o.trajectory_group == destination){
                    x1 = o.point.reference + o.point.trajectory_lenght / 2;
                    x2 = o.point.reference - o.point.trajectory_lenght / 2;
                    row = 1 - abs(normal_dist(x2) - normal_dist(x1));
                }
        
                row *= o.ANN_value;
            
            }else if (o.user == 1 && o.point.projection_distance <= projection_lim){

                // DB0
                x1 = o.point.reference - car_inflation * o.direction;
                x2 = o.point.reference + o.direction * (o.point.trajectory_lenght + car_inflation);
                row = 1 - abs(normal_dist(x2) - normal_dist(x1));

                // DB1
                if(o.trajectory_group == 0){

                    x1 = o.point.reference - o.direction * car_inflation;
                    x2 = x1 - o.direction * gap;
                    row *= 1 - abs(normal_dist(x2) - normal_dist(x1));
                }

                // DB2                
                if (destination == 3 && o.trajectory_group == 1){
                    x1 = o.point.reference - o.direction * car_inflation;
                    x2 = x1 - o.direction * gap;
                    row *= 1 - abs(normal_dist(x2) - normal_dist(x1));
                }
            }
        }

        // decision problem solution

        void solve(){

            joint_probability_distribution();

            float EU;

            for(int a = 0; a < 2; a++){                

                EU = row * utilities[a][0] + (1 - row) * utilities[a][1];

                if(EU > best.expected_utility){
                    best.action = a;
                    best.expected_utility = EU;
                }
            }

        }

    private:
        // header
        // x coordinate, y coordinate, direction along trajectory axis(positive = 1, negative = -1)
        float front_ref_points[4][3] = {{0.375, -0.125, 1},
                                            {0.5, 0.25, -1},
                                            {0.125, 0.375, -1},
                                            {0.0, 0.25, 1}};

        int len_front = 4;

        float back_ref_points[3][3] = {{0.125, -0.125, 1},
                                            {0.5, 0.0, -1},
                                            {0.375, 0.375, -1}};

        int len_back = 3;

};

class DecisionNode{
    ros::NodeHandle nh;
    ros::ServiceServer srv_decision;
    ros::ServiceClient client;

    public:
        DecisionNode()
            : nh()
        {
            srv_decision = nh.advertiseService("/srv_decision", &DecisionNode::srv_callback, this);
            client = nh.serviceClient<intersection_msgs::DetectUsers>("/duckiebot4/detector_node/detect_users");
        }

        SimpleProblem sp;

        bool srv_callback(intersection_msgs::MakeDecisionRequest &request,
                          intersection_msgs::MakeDecisionResponse &response){
            
            // service callback
            ROS_INFO("Decision service called");

            ros::WallTime start_time, end_time;
            start_time = ros::WallTime::now();

            sp.destination = request.destination.data;
            sp.best.action = 0;            

            int num_of_detections = 0, detection_class, idx;

            intersection_msgs::DetectUsers detection_service;
            if (client.call(detection_service))
            {
                ROS_INFO("detection succeded");
                num_of_detections = detection_service.response.detections.data[0];
                for(int i = 0; i < num_of_detections; i++){

                    detection_class = detection_service.response.detections.data[4 * i + 1];
                    SimpleProblem::Point observed;
                    observed.x = - 0.2 + detection_service.response.detections.data[4 * i + 3] / 100;
                    observed.y = detection_service.response.detections.data[4 * i + 4] / 100;

                    if(detection_class == 1){
                        sp.get_duckie_group(observed);
                    }else{
                        sp.project_onto_trajectory(observed, detection_class);
                    }

                    sp.solve();

                    if(sp.best.action == 1){
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
