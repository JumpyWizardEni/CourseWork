#ifndef COURSE_PROJECT_SOLVER_H
#define COURSE_PROJECT_SOLVER_H
#include <vector>
using std::vector;

struct Solver {
    int size = 0;
    float dt = 0.1;
    float visc = 1;
    float diff = 0.5;
    vector<float> density;
    vector<float> density_prev;

    vector<float> vx;
    vector<float> vy;

    vector<float> vx0;
    vector<float> vy0;

    Solver(int size, vector<float> &density,vector<float> &vx, vector<float> &vy, float dt = 0.1, float visc = 1, float diff = 0.5);

    void vel_step();
    void dens_step();
    void add_sources (vector<float> &v, vector<float> &v0);
    void diffuse (int b, vector<float> &x, vector<float> &x0, float diffuse);
    void advect(int b, vector<float> &d, vector<float> d0, vector<float> u, vector<float> v);
    void project(vector<float> & u, vector<float> &v, vector<float> &p, vector<float> &div);
    void set_bounds (int b, vector<float> &x);
};


#endif //COURSE_PROJECT_SOLVER_H
