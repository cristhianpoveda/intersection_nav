#ifndef SIMPLE_PROBLEM
#define SIMPLE_PROBLEM

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
            Trajectory_point point;
            int trajectory_group;
            float direction = 1;
        };

        struct Best{
            int action;
            float expected_utility;
        };

        // atributes

        SimpleProblem();

        // evidence
        Observation observation;
        
        // chance var (right of way)
        float row;

        // decision (action) and utility(u) vars;
        Best best;

        int destination;

        Point nearest_group;

        void min_distance(Point projected_duckiebot, float (*ref_points)[3], int len);

        Projected_point line_projection(float parallel_coor, float orthogonal_coor, float line_intercept);

        Projected_pos arc_projection(Point point_1, float h, float k, float r, float axis);

        void get_duckie_group(SimpleProblem::Point D_observed);

        void project_onto_trajectory(SimpleProblem::Point DB_observed);

        float normal_dist(float x);

        void joint_probability_distribution();
        
        void solve();

    private:
        // header
        float utilities[2][2] = {{0.9, -1}, {-0.1, 1}};
        // x coordinate, y coordinate, direction along trajectory axis(positive = 1, negative = -1)
        float front_ref_points[5][3] = {{0.375, -0.125, 1},
                                            {0.5, 0.25, -1},
                                            {0.125, 0.375, -1},
                                            {0.0, 0.25, 1},
                                            {0.125, 0.125, -1}};

        int len_front = 5;

        float back_ref_points[3][3] = {{0.125, -0.125, 1},
                                            {0.5, 0.0, -1},
                                            {0.375, 0.375, -1}};

        int len_back = 3;

};

#endif