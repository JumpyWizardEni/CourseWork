#ifndef COURSE_PROJECT_SOLVER_TEST_H
#define COURSE_PROJECT_SOLVER_TEST_H

#include <cxxtest/TestSuite.h>
#include "../src/solver.h"

const float EPS = 1e-3;

#define TS_ASSERT_VECTOR_EQUALS(vec, answer) \
    for (int i = 0; i < vec.size(); i++) {   \
        TS_ASSERT_DELTA(vec[i], answer[i], EPS)\
    }

#define PRINT_VECTOR(vec) \
    for (int i = 0; i < vec.size(); i++) { \
        std::cout << vec[i] << std::endl;\
    }

class SolverTest : public CxxTest::TestSuite {
public:
    void testAddSources() {
        std::vector<float> vec = {1, 2, 3, 4};
        std::vector<float> vy = {};
        std::vector<float> density = {};
        Solver solver = Solver(2, density, vec, vy, 0.1, 1, 0.5);
        std::vector<float> answer = {1.1, 2.2, 3.3, 4.4};

        solver.add_sources(solver.vx, solver.vx0);

        TS_ASSERT_VECTOR_EQUALS(solver.vx, answer);
    }

    void testSetBounds() {
        std::vector<float> vec;
        vec.resize(16);
        for (int i = 0; i < 16; ++i) {
            vec[i] = i + 1;
        }
        int b = 1;
        std::vector<float> vy = {};
        std::vector<float> density = {};
        Solver solver = Solver(4, density, vec, vy, 0.1, 1, 0.5);
        std::vector<float> answer = {0, 6, 7, 0, -6, 6, 7, -7, -10, 10, 11, -11, 0, 10, 11, 0};
        solver.set_bounds(b, solver.vx);
        TS_ASSERT_VECTOR_EQUALS(solver.vx, answer)
    }

    void testDiffuse() {
        std::vector<float> vec;
        vec.resize(16);
        for (int i = 0; i < 16; ++i) {
            vec[i] = i + 1;
        }
        int b = 1;
        std::vector<float> vy = {};
        std::vector<float> density = {};
        Solver solver = Solver(4, density, vec, vy, 0.1, 1, 0.5);
        std::vector<float> answer = {0, 3.7606, 4.1452, 0, -3.7606, 3.7606, 4.1452, -4.1452, -5.2991, 5.2991, 5.6837,
                                     -5.6837, 0, 5.2991, 5.6837, 0};
        solver.diffuse(1, solver.vx, solver.vx0, solver.visc);
        TS_ASSERT_VECTOR_EQUALS(solver.vx, answer)
    }

    void testAdvect() {
        std::vector<float> vec;
        vec.resize(16);
        for (int i = 0; i < 16; ++i) {
            vec[i] = i + 1;
        }
        int b = 1;
        std::vector<float> vy = vec;
        std::vector<float> density = {};
        Solver solver = Solver(4, density, vec, vy, 0.1, 1, 0.5);
        std::vector<float> answer = {0, 3.5, 3.6, 0, -3.5, 3.5, 3.6, -3.6, -3.5, 3.5, 3.5,
                                     -3.5, 0, 3.5,
                                     3.5, 0};
        solver.advect(1, solver.vx, solver.vx0, solver.vx0, solver.vy0);
        TS_ASSERT_VECTOR_EQUALS(solver.vx, answer)
    }

    void testProject() {
        std::vector<float> vec;
        vec.resize(16);
        for (int i = 0; i < 16; ++i) {
            vec[i] = i + 1;
        }
        int b = 1;
        std::vector<float> vy = vec;
        std::vector<float> density = {};
        Solver solver = Solver(4, density, vec, vy, 0.1, 1, 0.5);
        std::vector<float> answer_vx = {0, 6.4166, 7.4166, 0, -6.4166, 6.4166, 7.4166, -7.4166, -10.4166, 10.4166,
                                        11.4166,
                                        -11.4166, 0, 10.4166,
                                        11.4166, 0};

        std::vector<float> answer_vx0 = {-16.1805, -16.1805, -16.5972, -16.5972, -16.1805, -16.1805, -16.5972, -16.5972, -16.5972, -16.5972, -17.0138, -17.0138,
                                        -16.5972, -16.5972,
                                        -17.0138, -17.0138};

        std::vector<float> answer_vy = {0, -6.4166, -7.4166, 0, 6.4166, 6.4166, 7.4166, 7.4166, 10.4166, 10.4166,
                                         11.4166,
                                         11.4166, 0, -10.4166,
                                         -11.4166, 0};

        std::vector<float> answer_vy0 = {-2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5,
                                         -2.5, -2.5, -2.5};
        solver.project(solver.vx, solver.vy, solver.vx0, solver.vy0);
        TS_ASSERT_VECTOR_EQUALS(solver.vx, answer_vx)
        TS_ASSERT_VECTOR_EQUALS(solver.vx0, answer_vx0)
        TS_ASSERT_VECTOR_EQUALS(solver.vy, answer_vy)
        TS_ASSERT_VECTOR_EQUALS(solver.vy0, answer_vy0)
    }
};

#endif //COURSE_PROJECT_SOLVER_TEST_H
