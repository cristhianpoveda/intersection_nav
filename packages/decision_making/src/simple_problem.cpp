#include <ros/ros.h>
#include "simple_problem.h"
#include "math.h"

SimpleProblem::SimpleProblem(){
    row = 1;
    best.expected_utility = -10;
    projected_user.x = 10;
    projected_user.y = 10;
    primary_projection.pos.x = 10;
    primary_projection.pos.y = 10;
    second_projection.pos.x = 10;
    second_projection.pos.y = 10;
}

// member functions

int SimpleProblem::new_project_to_crosswalk(){

    float min_projection_distance = 11, duckie_distance_th = 0.086;
    int probable_projections = 0;

    SimpleProblem::Projection projection;
    projection.distance = 10;
    primary_projection.pos.x = 10;
    primary_projection.pos.y = 10;
    primary_projection.distance = 10;
    second_projection.pos.x = 10;
    second_projection.pos.y = 10;
    second_projection.distance = 10;

    for(int i = 0; i < 4; i++){

        if(crosswalk_axis[i] == 0){

            projection.pos.x = projected_user.x;
            projection.pos.y = intersection_limits[i];
            projection.distance = abs(intersection_limits[i] - projected_user.y);
            projection.group = i;

        }else{

            projection.pos.x = intersection_limits[i];
            projection.pos.y = projected_user.y;
            projection.distance = abs(intersection_limits[i] - projected_user.x);
            projection.group = i;
        }

        if(projection.distance < min_projection_distance && projection.distance < duckie_distance_th){

            ROS_INFO("Assign duckie projection");

            if(projection.distance < primary_projection.distance){
                second_projection = primary_projection;
                primary_projection = projection;
            }else{
                second_projection = projection;
            }

            min_projection_distance = second_projection.distance;
            probable_projections++;

        }
    }

    return probable_projections;
}

int SimpleProblem::new_min_distance(float (*ref_points)[3]){

    float min_distance = 0, distance = 0;
    int group = 0;

    min_distance = sqrt(pow((intersection_center[0] - projected_user.x), 2)
                        + pow((intersection_center[1] - projected_user.y), 2));

    for(int i = 0; i < 3; i++){
        distance = sqrt(pow((ref_points[i][0] - projected_user.x), 2)
                      + pow((ref_points[i][1] - projected_user.y), 2));

        if(distance < min_distance){
            min_distance = distance;
            group = i + 1;
        }
    }

    return group;
}

SimpleProblem::Projection SimpleProblem::new_line_projection(int idx){

    SimpleProblem::Projection projection;
    if(intersection_trajectories[idx][3] == 0){

        projection.pos.x = projected_user.x;
        projection.pos.y = intersection_trajectories[idx][4];
        projection.distance = abs(intersection_trajectories[idx][4] - projected_user.y);

    }else{
        projection.pos.x = intersection_trajectories[idx][4];
        projection.pos.y = projected_user.y;
        projection.distance = abs(intersection_trajectories[idx][4] - projected_user.x);
    }

    return projection;
    
}

SimpleProblem::Projection SimpleProblem::new_arc_projection(int idx){

    SimpleProblem::Projection projection;

    float m = (projected_user.y - intersection_trajectories[idx][4]) / (projected_user.x - intersection_trajectories[idx][3]);
    float b_line = -m * projected_user.x + projected_user.y;

    // solve quadratic eq

    // because of + 1 in "a" calculation, there is never a discontinuity for the quadratic equation.
    float a = pow(m, 2) + 1;
    float b_quad_eq = 2 * m * (b_line - intersection_trajectories[idx][4]) - 2 * intersection_trajectories[idx][3];
    float c = pow((b_line - intersection_trajectories[idx][4]),2) - pow(intersection_trajectories[idx][5], 2) + pow(intersection_trajectories[idx][3], 2);

    float x1 = (- b_quad_eq + sqrt(pow(b_quad_eq, 2) - 4 * a * c)) / (2 * a);
    float x2 = (- b_quad_eq - sqrt(pow(b_quad_eq, 2) - 4 * a * c)) / (2 * a);

    float y1 = m * x1 + b_line;
    float y2 = m * x2 + b_line;

    // find projected distance
    float proj_distance_1 = sqrt(pow((x1 - projected_user.x), 2) + pow((y1 - projected_user.y), 2));
    float proj_distance_2 = sqrt(pow((x2 - projected_user.x), 2) + pow((y2 - projected_user.y), 2));

    if(proj_distance_1 < proj_distance_2){

        projection.pos.x = x1;
        projection.pos.y = y1;
        projection.distance = proj_distance_1;

    }else{

        projection.pos.x = x2;
        projection.pos.y = y2;
        projection.distance = proj_distance_2;

    }

    return projection;
}

int SimpleProblem::new_project_to_trajectory(){

    SimpleProblem::Projection projection;
    projection.distance = 10;
    primary_projection.pos.x = 10;
    primary_projection.pos.y = 10;
    primary_projection.distance = 10;
    second_projection.pos.x = 10;
    second_projection.pos.y = 10;
    second_projection.distance = 10;

    int idx = 0, probable_projections = 0, group = 0;
    float min_projection_distance = 11, duckiebot_distance_th = 0.125;
    
    bool outside_projection = projected_user.x < intersection_limits[0] || projected_user.x > intersection_limits[2] ||
                    projected_user.y < intersection_limits[1] || projected_user.y > intersection_limits[3];

    if(user == 0){
        group = new_min_distance(back_ref_points);
        idx = 2;
    }else{
        group = new_min_distance(front_ref_points);
        idx = 1;
    }

    ROS_INFO("B primary x: %f", primary_projection.pos.x);
    ROS_INFO("B primary y: %f", primary_projection.pos.y);
    ROS_INFO("B primary distance: %f", primary_projection.distance);
    ROS_INFO("B second x: %f", second_projection.pos.x);
    ROS_INFO("B second y: %f", second_projection.pos.y);
    ROS_INFO("B secondary distance: %f", second_projection.distance);

    int num_of_trajectories = sizeof(intersection_trajectories) / sizeof(intersection_trajectories[0]);
    
    // change 9 by trajectory var
    for(int i = 0; i < num_of_trajectories; i++){

        if((group == 0 && intersection_trajectories[i][0] == 1) ||
            group == intersection_trajectories[i][idx]){

            ROS_INFO("FOR: %d", i);
            ROS_INFO("index: %d", idx);
            ROS_INFO("outside: %d", outside_projection);

            if(intersection_trajectories[i][5] ==  0){
                ROS_INFO("Line");
                projection = new_line_projection(i);
            }else if(!outside_projection){
                ROS_INFO("Arc");
                projection = new_arc_projection(i);
            }

            if(projection.distance < min_projection_distance && projection.distance < duckiebot_distance_th){

                ROS_INFO("Assign projection");

                if(projection.distance < primary_projection.distance){
                    second_projection = primary_projection;
                    primary_projection = projection;
                }else{
                    second_projection = projection;
                }

                min_projection_distance = second_projection.distance;
                probable_projections++;

            }
            ROS_INFO("primary x: %f", primary_projection.pos.x);
            ROS_INFO("primary y: %f", primary_projection.pos.y);
            ROS_INFO("B primary distance: %f", primary_projection.distance);
            ROS_INFO("second x: %f", second_projection.pos.x);
            ROS_INFO("second y: %f", second_projection.pos.y);
            ROS_INFO("B secondary distance: %f", second_projection.distance);

        }
    }

    primary_projection.group = group;
    second_projection.group = group;

    return probable_projections;
}

float SimpleProblem::new_normal_dist(float mean, float x){

    //header
    // 3o = 0.08 m
    float o_1 = 0.1;
    float std_dev = o_1/3;

    float z = (x - mean) / std_dev;
    float probability = 0.5 * erfc(-z * M_SQRT1_2);

    return probability;
}

float SimpleProblem::new_bayesian_network(SimpleProblem::Projection projection){

    float row = 1, mean_x = projection.pos.x, mean_y = projection.pos.y, x = 0, x2 = 0, car_inf, inflation_back = 0.15, inflation_front = 0.1, gap = 0.1, clearance = 0.05;

    if(user == 1 && (projection.group == destination || projection.group == 0)){

        if(crosswalk_axis[projection.group] == 0){

            row = 1 - abs(new_normal_dist(mean_x, intersection_limits[0]) - new_normal_dist(mean_x, intersection_limits[2]));

        }else{

            row = 1 - abs(new_normal_dist(mean_y, intersection_limits[1]) - new_normal_dist(mean_y, intersection_limits[3]));
        }

    }else if(user == 0 || user == 2){

        if(user == 2){

            car_inf = inflation_front;

        }else if(destination == projection.group){
            
            car_inf = inflation_back + clearance;

        }else{
            car_inf = inflation_back;
        }

        row = 1 - abs(new_normal_dist(mean_x, (intersection_limits[0] - car_inf)) - new_normal_dist(mean_x, (intersection_limits[2] + car_inf)))
                        *abs(new_normal_dist(mean_y, (intersection_limits[1] - car_inf)) - new_normal_dist(mean_y, (intersection_limits[3] + car_inf)));

        ROS_INFO("row 0: %f", row);

        if(user == 2){

            if(projection.group == 1){

                x = front_ref_points[0][1] - car_inf;
                row *= 1 - abs(new_normal_dist(mean_y, x) - new_normal_dist(mean_y, (x - gap)));
                ROS_INFO("row db1: %f", row);
            }
            if(projection.group == 2 && destination == 3){

                x = front_ref_points[1][0] +  car_inf;
                row *= 1 - abs(new_normal_dist(mean_x, x) - new_normal_dist(mean_x, (x + gap)));

                ROS_INFO("row db2: %f", row);
            }
        }
    }

    return row;
}

void SimpleProblem::new_solve(){

    float EU = 0;

    for(int a = 0; a < 2; a++){                

        EU = row * utilities[a][0] + (1 - row) * utilities[a][1];

        if(EU > best.expected_utility){
            
            best.action = a;
            best.expected_utility = EU;
        }
    }
}



int SimpleProblem::solveMultivariate(SimpleProblem::Point DB_observed){

    float duckie_limits [4][4] = {{-0.086, 0.086, -0.125, 0.375},
                    {0, 0.5, -0.211, -0.039},
                    {0.414, 0.586, -0.125, 0.375},
                    {0, 0.5, 0.289, 0.461}};

    float duckiebot_limits [3][4] = {{-0.1, 0.6, -0.225, 0.475},
                    {0.25, 0.5, -0.325, -0.225},
                    {0.6, 0.7, 0.25, 0.375}};

    float expected_u = -10, row_multivariate = 1, presence = 0, presence_partial = 0;
    int selected_action = 1, group = 0;

    if(user == 1){

        for (int i = 0; i < 4; i++){

            presence_partial = abs(new_normal_dist(projected_user.x, duckie_limits[i][0]) - new_normal_dist(projected_user.x, duckie_limits[i][1])) * abs(new_normal_dist(projected_user.y, duckie_limits[i][2]) - new_normal_dist(projected_user.y, duckie_limits[i][3]));

            if(presence_partial > presence){
                presence = presence_partial;
                group = i;
            }
            if(group == destination || group == 0){
                
                row_multivariate = 1 - presence;
            }else{
                row_multivariate = 1;
            }
        }

    }else{

        if(user == 2){
            // DB0
            row_multivariate = 1 - (abs(new_normal_dist(projected_user.x, duckiebot_limits[0][0]) - new_normal_dist(projected_user.x, duckiebot_limits[0][1])) * abs(new_normal_dist(projected_user.y, duckiebot_limits[0][2]) - new_normal_dist(projected_user.y, duckiebot_limits[0][3])));

        }else{
            row_multivariate = 1 - (abs(new_normal_dist(projected_user.x, -0.275) - new_normal_dist(projected_user.x, 0.525)) * abs(new_normal_dist(projected_user.y, -0.15) - new_normal_dist(projected_user.y, 0.65)));
        }

        // DB1
        row_multivariate *= (1 - (abs(new_normal_dist(projected_user.x, duckiebot_limits[1][0]) - new_normal_dist(projected_user.x, duckiebot_limits[1][1])) * abs(new_normal_dist(projected_user.y, duckiebot_limits[1][2]) - new_normal_dist(projected_user.y, duckiebot_limits[1][3]))));

        // DB2
        if(destination == 3){
            row_multivariate *= (1 - (abs(new_normal_dist(projected_user.x, duckiebot_limits[2][0]) - new_normal_dist(projected_user.x, duckiebot_limits[2][1])) * abs(new_normal_dist(projected_user.y, duckiebot_limits[2][2]) - new_normal_dist(projected_user.y, duckiebot_limits[2][3]))));
        }
    }

    float EU = 0;

    for(int a = 0; a < 2; a++){                

        EU = row_multivariate * utilities[a][0] + (1 - row_multivariate) * utilities[a][1];

        if(EU > expected_u){
            
            selected_action = a;
            expected_u = EU;
        }
    }

    return selected_action;

}