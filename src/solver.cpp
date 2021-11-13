#include "solver.h"

#define IX(x, y) ((x) + (y) * this->size)
const int STEPS_NUM = 20;

void Solver::add_sources(std::vector<float> &v, std::vector<float> &v0) {
    int size_ = this->size * this->size;
    for (int i = 0; i < size_; i++) {
        v[i] += this->dt * v0[i];
    }

}

void Solver::diffuse(int b, vector<float> &x, vector<float> &x0, float diffuse) {
    int N = size - 2;
    float a = dt * diffuse * N * N;

    for (int k = 0; k < STEPS_NUM; k++) {
        for (int i = 1; i <= N; i++) {
            for (int j = 1; j <= N; j++) {
                x[IX(i, j)] = (x0[IX(i, j)] + a * (x[IX(i - 1, j)] + x[IX(i + 1, j)] +
                                                   x[IX(i, j - 1)] + x[IX(i, j + 1)])) / (1 + 4 * a);
            }
        }
        set_bounds(b, x);
    }
}

void Solver::advect(int b, std::vector<float> &d, std::vector<float> d0, std::vector<float> u, std::vector<float> v) {
    int i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;
    int N = size - 2;
    dt0 = dt * N;
    for (int i = 1; i <= N; i++) {
        for (int j = 1; j <= N; j++) {
            x = i - dt0 * u[IX(i, j)];
            y = j - dt0 * v[IX(i, j)];
            if (x < 0.5) x = 0.5;
            if (x > N + 0.5) x = N + 0.5;
            i0 = (int) x;
            i1 = i0 + 1;
            if (y < 0.5) y = 0.5;
            if (y > N + 0.5) y = N + 0.5;
            j0 = (int) y;
            j1 = j0 + 1;
            s1 = x - i0;
            s0 = 1 - s1;
            t1 = y - j0;
            t0 = 1 - t1;
            d[IX(i, j)] = s0 * (t0 * d0[IX(i0, j0)] + t1 * d0[IX(i0, j1)]) +
                          s1 * (t0 * d0[IX(i1, j0)] + t1 * d0[IX(i1, j1)]);
        }
    }
    set_bounds(b, d);
}


void Solver::vel_step() {
    add_sources(vx, vx0);
    add_sources(vy, vy0);

    swap(vx0, vx);
    diffuse(1, vx, vx0, visc);
    swap(vy0, vy);
    diffuse(2, vy, vy0, visc);

    project(vx, vy, vx0, vy0);

    swap(vx0, vx);
    swap(vy0, vy);

    advect(1, vx, vx0, vx0, vy0);
    advect(2, vy, vy0, vx0, vy0);

    project(vx, vy, vx0, vy0);
}

void
Solver::dens_step() {
    add_sources(density, density_prev);
    swap(density_prev, density);
    diffuse(0, density, density_prev, diff);
    swap(density_prev, density);
    advect(0, density, density_prev, vx, vy);
}


void Solver::project(std::vector<float> & u, std::vector<float> &v, std::vector<float> &p, std::vector<float> &div) {
    float h;
    int N = this->size - 2;
    h = 1.0f / N;
    for (int i = 1; i <= N; i++) {
        for (int j = 1; j <= N; j++) {
            div[IX(i, j)] = -0.5 * h * (u[IX(i + 1, j)] - u[IX(i - 1, j)] +
                                        v[IX(i, j + 1)] - v[IX(i, j - 1)]);
            p[IX(i, j)] = 0;
        }
    }
    set_bounds(0, div);
    set_bounds(0, p);

    for (int k = 0; k < STEPS_NUM; k++) {
        for (int i = 1; i <= N; i++) {
            for (int j = 1; j <= N; j++) {
                p[IX(i, j)] = (div[IX(i, j)] + p[IX(i - 1, j)] + p[IX(i + 1, j)] +
                               p[IX(i, j - 1)] + p[IX(i, j + 1)]) / 4;
            }
        }
        set_bounds(0, p);
    }

    for (int i = 1; i <= N; i++) {
        for (int j = 1; j <= N; j++) {
            u[IX(i, j)] -= 0.5 * (p[IX(i + 1, j)] - p[IX(i - 1, j)]) / h;
            v[IX(i, j)] -= 0.5 * (p[IX(i, j + 1)] - p[IX(i, j - 1)]) / h;
        }
    }
    set_bounds(1, u);
    set_bounds(2, v);
}

void Solver::set_bounds(int b, vector<float> &x) {
    int N = size;
    for (int i = 1; i < N; i++) {
        x[IX(0, i)] = b == 1 ? -x[IX(1, i)] : x[IX(1, i)];
        x[IX(N - 1, i)] = b == 1 ? -x[IX(N - 2, i)] : x[IX(N - 2, i)];
        x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(i, 1)];
        x[IX(i, N - 1)] = b == 2 ? -x[IX(i, N - 2)] : x[IX(i, N - 2)];
    }
    x[IX(0, 0)] = 0.5 * (x[IX(1, 0)] + x[IX(0, 1)]);
    x[IX(0, N - 1)] = 0.5 * (x[IX(1, N - 1)] + x[IX(0, N - 2)]);
    x[IX(N - 1, 0)] = 0.5 * (x[IX(N - 2, 0)] + x[IX(N - 1, 1)]);
    x[IX(N - 1, N - 1)] = 0.5 * (x[IX(N - 2, N - 1)] + x[IX(N - 1, N - 2)]);
}

Solver::Solver(int size, vector<float> &density, vector<float> &vx, vector<float> &vy, float dt, float visc, float diff) {
    this->size = size;
    this->density = std::move(density);
    this->density_prev = this->density;
    this->vx = std::move(vx);
    this->vx0 = this->vx;
    this->vy = std::move(vy);
    this->vy0 = this->vy;
    this->dt = dt;
    this->visc = visc;
    this->diff = diff;
}
