#ifndef SIMPLE_PROBLEM
#define SIMPLE_PROBLEM

class SimpleProblem{

    public:

        struct Point{
            float x;
            float y;
        };

        struct Projection{
            Point pos;
            float distance;
            int group;
        };

        struct Best{
            int action;
            float expected_utility;
        };

        // atributes

        SimpleProblem();

        // evidence

        int user;
        
        // chance var (right of way)
        float row;

        // decision (action) and utility(u) vars;
        Best best;

        int destination;

        Point projected_user;

        Projection primary_projection;

        Projection second_projection;

        int project_to_crosswalk();

        int min_distance(float (*ref_points)[3]);

        Projection line_projection(int idx);

        Projection arc_projection(int idx);

        int project_to_trajectory();

        float normal_dist(float mean, float x);

        float bayesian_network(Projection projection);

        void solve();

    private:
        // header
        float utilities[2][2] = {{-0.5, 1}, {0.3, -1}};

        float intersection_center[2] = {0.25, 0.125};

        float intersection_limits[4] = {0, -0.125, 0.5, 0.375};

        float crosswalk_axis[4] = {1, 0, 1, 0};

        // x coordinate, y coordinate, direction along trajectory axis(positive = 1, negative = -1)
        float front_ref_points[3][3] = {{0.375, -0.125, 1},
                                        {0.5, 0.25, -1},
                                        {0.125, 0.375, -1}};

        float back_ref_points[3][3] = {{0.125, -0.125, 1},
                                        {0.5, 0.0, -1},
                                        {0.375, 0.375, -1}};

        float intersection_trajectories[10][7] = {{1, 1, -1, -0.175, -0.275, 1.1, 1.05},
                                                 {1, 1, 2, 0.625, -0.275, 0.5, 0.56},
                                                 {0, 1, 3, 1, 0.375, 0, 0},
                                                 {0, 2, -1, 0, 0.25, 0, 0},
                                                 {1, 2, 1, 0.65, -0.3, 1.05, 1.1},                                                 
                                                 {1, 2, 3, 0.65, 0.5, 0.56, 0.5},
                                                 {1, 3, -1, -0.125, 0.525, 0.5, 0.56},
                                                 {0, 3, 1, 1, 0.125, 0},
                                                 {1, 3, 2, 0.675, 0.525, 1.1, 1.05},
                                                 {1, -1, 2, 0, 0, 0}};

};

#endif