#include "simple_problem.h"
#include "math.h"

SimpleProblem::SimpleProblem(){
    row = 1;
    best.expected_utility = -10;
}

// member functions

void SimpleProblem::min_distance(Point projected_duckiebot, float (*ref_points)[3], int len){

    float min_distance = 1, distance;
    observation.trajectory_group = 0;

    for(int i = 0; i < len; i++){
    
        distance = sqrt(pow((ref_points[i][0] - projected_duckiebot.x), 2) + pow((ref_points[i][1] - projected_duckiebot.y), 2));
    
        if(distance < min_distance){
            min_distance = distance;
            observation.trajectory_group = i;
            nearest_group.x = ref_points[i][0];
            nearest_group.y = ref_points[i][1];
            observation.direction = ref_points[i][2];
        }
    }
}

// geometrical projections
SimpleProblem::Projected_point SimpleProblem::line_projection(float parallel_coor, float orthogonal_coor, float line_intercept){

    Projected_point Pp;

    Pp.coordinates.x = parallel_coor;
    Pp.coordinates.y = line_intercept;
    Pp.projection_distance = abs(orthogonal_coor - line_intercept);

    return Pp;
}

SimpleProblem::Projected_pos SimpleProblem::arc_projection(Point point_1, float h, float k, float r, float axis){

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
void SimpleProblem::get_duckie_group(SimpleProblem::Point D_observed){

    // header
    //0 axis, 1 m, 2 b, 3 len.

    float ref_points_duckie[4][2] = {{0.0, 0.125},{0.25, -0.125},{0.5, 0.125},{0.25, 0.375}};

    float trajectories[4][4] = {{1, 0, 0, 0.5},
                                {0, 0, -0.125, 0.5},
                                {1, 0, 0.5, 0.5},
                                {0, 0, 0.375, 0.5}};

    observation.point.projection_distance = 1;

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

        if(Pp_line.projection_distance < observation.point.projection_distance){
        
            observation.position_coor = Pp_line.coordinates;
            observation.point.projection_distance = Pp_line.projection_distance;
            observation.point.mean = u;
            observation.point.reference = ref;
            observation.point.trajectory_lenght = trajectories[i][3];
            observation.trajectory_group = i;

        }
    }

}

// projection duckiebot

void SimpleProblem::project_onto_trajectory(SimpleProblem::Point DB_observed){

    // Parallel axis, intersection limit, right arc, line, left arc.
    // 0 axis, 1 min limit, 2 max limit, 3:5(h1, k1, r1) 6 len, 7:8(m, b) 9 len, 10:12(h2, k2, r2) 13 len.
    // axis: x = 0, y = 1.
    // limit: not considered = -1.

    float trajectories[8][14] = {{1, -0.125, 1, 0.5, -0.125, 0.125, 0.39, 0, 0.375, 0.5, 0, -0.125, 0.375, 1.18}, // way1 front
                                {0, -1, 0.5, 0.5, 0.375, 0.125, 0.39, 0, 0.25, 0.5, 0.5, -0.125, 0.375, 1.18}, //way 2 front
                                {1, -1, 0.375, 0, 0.375, 0.125, 0.39, 0, 0.125, 0.5, 0.5, 0.375, 0.375, 1.18}, // way 3 front
                                {0, 0, 1, 0, -0.125, 0.375, 1.18, 0, 0.25, 0.5, 0, 0, 10, 10}, // way 0 front
                                {1, -0.125, 0.375, 0, 0, 10, 10, 0, 0.125, 0.25, 0, 0, 10, 10}, // engaged
                                {1, -0.125, 1, 0.5, -0.125, 0.375, 1.18, 0, 0.125, 0.5, 0, 0, 10, 10}, // way 1 back
                                {0, -1, 0.5, 0.5, -0.125, 0.125, 0.39, 0, 0, 0.5, 0.5, 0.375, 0.375, 1.18}, // way 2 back
                                {1, -1, 0.375, 0.5, 0.375, 0.125, 0.39, 0, 0.375, 0.5, 0, 0, 10, 10}}; // way 3 back  

    if(observation.user == 0){
        min_distance(DB_observed, back_ref_points, len_back);
        observation.trajectory_group += 5;
    }else{
        min_distance(DB_observed, front_ref_points, len_front);
    }

    Projected_point Pp_line;

    Projected_pos Pp_arc_1;
    Projected_pos Pp_arc_2;

    float parallel_coor = 0;
    float orthogonal_coor = 0;

    if(trajectories[observation.trajectory_group][0] == 0){
        parallel_coor = DB_observed.x;
        orthogonal_coor = DB_observed.y;
        observation.point.reference = nearest_group.x;

    }else{
        parallel_coor = DB_observed.y;
        orthogonal_coor = DB_observed.x;
        observation.point.reference = nearest_group.y;
    }

    Pp_line = line_projection(parallel_coor, orthogonal_coor, trajectories[observation.trajectory_group][8]);
        observation.point.trajectory_lenght = trajectories[observation.trajectory_group][9];

    if(trajectories[observation.trajectory_group][0] == 1){
        observation.position_coor.x = Pp_line.coordinates.y;
        observation.position_coor.y = Pp_line.coordinates.x;
        
    }else{
        observation.position_coor = Pp_line.coordinates;
    }
    observation.point.mean = Pp_line.coordinates.x;
    observation.point.projection_distance = Pp_line.projection_distance;

    if(parallel_coor > trajectories[observation.trajectory_group][1] && parallel_coor < trajectories[observation.trajectory_group][2]){
    
        // arc1
        Pp_arc_1 = arc_projection(DB_observed, trajectories[observation.trajectory_group][3], trajectories[observation.trajectory_group][4],
        trajectories[observation.trajectory_group][5], trajectories[observation.trajectory_group][0]);

        if(Pp_arc_1.point.projection_distance < observation.point.projection_distance){
            observation.position_coor = Pp_arc_1.position;
            observation.point.mean = Pp_arc_1.point.mean;
            observation.point.projection_distance = Pp_arc_1.point.projection_distance;
            observation.point.trajectory_lenght = trajectories[observation.trajectory_group][6];   
        }
    
        // arc2
        Pp_arc_2 = arc_projection(DB_observed, trajectories[observation.trajectory_group][10], trajectories[observation.trajectory_group][11],
        trajectories[observation.trajectory_group][12], trajectories[observation.trajectory_group][0]);

        if(Pp_arc_2.point.projection_distance < observation.point.projection_distance){
            observation.position_coor = Pp_arc_2.position;
            observation.point.mean = Pp_arc_2.point.mean;
            observation.point.projection_distance = Pp_arc_2.point.projection_distance;
            observation.point.trajectory_lenght = trajectories[observation.trajectory_group][13];
        }
    }
}

// probabilistic distributions
float SimpleProblem::normal_dist(float x){

    //header
    // 3o = 0.1 m
    float std_dev = 0.1/3;

    float z = (x - observation.point.mean) / std_dev;
    float p = 0.5 * erfc(-z * M_SQRT1_2);

    return p;
}

void SimpleProblem::joint_probability_distribution(){

    float x1, x2;

    // header
    float car_inflation, gap = 0.1, projection_lim = 0.125, clearance = 0.05;

    if (observation.user == 1 && observation.point.projection_distance <= projection_lim){

        // Di
        if(observation.trajectory_group == 0 || observation.trajectory_group == destination){
            x1 = observation.point.reference + observation.point.trajectory_lenght / 2;
            x2 = observation.point.reference - observation.point.trajectory_lenght / 2;
            row = 1 - abs(normal_dist(x2) - normal_dist(x1));
        }
    
    }else if ((observation.user == 0 || observation.user == 2) && observation.point.projection_distance <= projection_lim){

        if(observation.user == 0){
            car_inflation = 0.15;
        }else{
            car_inflation = 0.08;
        }

        // DB0
        x1 = observation.point.reference - car_inflation * observation.direction;
        x2 = observation.point.reference + observation.direction * (observation.point.trajectory_lenght + car_inflation);
        if(observation.user == 0 && destination == (observation.trajectory_group - 4)){
            x1 -= (observation.direction * clearance);
        }
        row = 1 - abs(normal_dist(x2) - normal_dist(x1));

        // DB1
        if(observation.trajectory_group == 0){

            x1 = observation.point.reference - observation.direction * car_inflation;
            x2 = x1 - observation.direction * gap;
            row *= 1 - abs(normal_dist(x2) - normal_dist(x1));
        }

        // DB2                
        if (destination == 3 && observation.trajectory_group == 1){
            x1 = observation.point.reference - observation.direction * car_inflation;
            x2 = x1 - observation.direction * gap;
            row *= 1 - abs(normal_dist(x2) - normal_dist(x1));
        }
    }
}

// decision problem solution

void SimpleProblem::solve(){

    joint_probability_distribution();

    float EU = 0;

    for(int a = 0; a < 2; a++){                

        EU = row * utilities[a][0] + (1 - row) * utilities[a][1];

        if(EU > best.expected_utility){
            
            best.action = a;
            best.expected_utility = EU;
        }
    }

}