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

int SimpleProblem::project_to_crosswalk(){

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

int SimpleProblem::min_distance(float (*ref_points)[3]){

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

SimpleProblem::Projection SimpleProblem::line_projection(int idx){

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

SimpleProblem::Projection SimpleProblem::arc_projection(int idx){

    SimpleProblem::Projection projection;
    float = theta = 0, k = 0;

    theta = atan2(projected_user.y - intersection_trajectories[idx][4], projected_user.x - intersection_trajectories[idx][3]);

    k = intersection_trajectories[idx][5] * intersection_trajectories[idx][6] / sqrt(pow(intersection_trajectories[idx][6] * cos(theta), 2) + pow(intersection_trajectories[idx][5] * sin(theta), 2));

    projection.pos.x = k * cos(theta) + intersection_trajectories[idx][3];
    projection.pos.y = k * sin(theta) + intersection_trajectories[idx][4];
    projection.distance = sqrt(pow((projection.pos.x - projected_user.x), 2) + pow((projection.pos.y - projected_user.y), 2));

    return projection;
}

int SimpleProblem::project_to_trajectory(){

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
        group = min_distance(back_ref_points);
        idx = 2;
    }else{
        group = min_distance(front_ref_points);
        idx = 1;
    }

    int num_of_trajectories = sizeof(intersection_trajectories) / sizeof(intersection_trajectories[0]);
    
    for(int i = 0; i < num_of_trajectories; i++){

        if((group == 0 && intersection_trajectories[i][0] == 1) ||
            group == intersection_trajectories[i][idx]){

            if(intersection_trajectories[i][5] ==  0){
                projection = line_projection(i);
            }else if(!outside_projection){
                projection = arc_projection(i);
            }

            if(projection.distance < min_projection_distance && projection.distance < duckiebot_distance_th){

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
    }

    primary_projection.group = group;
    second_projection.group = group;

    return probable_projections;
}

float SimpleProblem::normal_dist(float mean, float x){

    //header
    // 3o = 0.1 m
    float o_1 = 0.1;
    float std_dev = o_1/3;

    float z = (x - mean) / std_dev;
    float probability = 0.5 * erfc(-z * M_SQRT1_2);

    return probability;
}

float SimpleProblem::bayesian_network(SimpleProblem::Projection projection){

    float row = 1, mean_x = projection.pos.x, mean_y = projection.pos.y, x = 0, x2 = 0, car_inf, inflation_back = 0.15, inflation_front = 0.1, gap = 0.1, clearance = 0.05;

    if(user == 1 && (projection.group == destination || projection.group == 0)){

        if(crosswalk_axis[projection.group] == 0){

            row = 1 - abs(normal_dist(mean_x, intersection_limits[0]) - normal_dist(mean_x, intersection_limits[2]));

        }else{

            row = 1 - abs(normal_dist(mean_y, intersection_limits[1]) - normal_dist(mean_y, intersection_limits[3]));
        }

    }else if(user == 0 || user == 2){

        if(user == 2){

            car_inf = inflation_front;

        }else if(destination == projection.group){
            
            car_inf = inflation_back + clearance;

        }else{
            car_inf = inflation_back;
        }

        row = 1 - abs(normal_dist(mean_x, (intersection_limits[0] - car_inf)) - normal_dist(mean_x, (intersection_limits[2] + car_inf)))
                        *abs(normal_dist(mean_y, (intersection_limits[1] - car_inf)) - normal_dist(mean_y, (intersection_limits[3] + car_inf)));

        if(user == 2){

            if(projection.group == 1){

                x = front_ref_points[0][1] - car_inf;
                row *= 1 - abs(normal_dist(mean_y, x) - normal_dist(mean_y, (x - gap)));
            }
            if(projection.group == 2 && destination == 3){

                x = front_ref_points[1][0] +  car_inf;
                row *= 1 - abs(normal_dist(mean_x, x) - normal_dist(mean_x, (x + gap)));
            }
        }
    }

    return row;
}

void SimpleProblem::solve(){

    float EU = 0;

    for(int a = 0; a < 2; a++){                

        EU = row * utilities[a][0] + (1 - row) * utilities[a][1];

        if(EU > best.expected_utility){
            
            best.action = a;
            best.expected_utility = EU;
        }
    }
}