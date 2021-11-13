#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <string>
#include "solver.h"
#include "stb_image_write.h"
#include <cstdlib>
#include <iostream>
#include <algorithm>

const int N = 100;
const int GRID_SIZE = 5;
const int MAX_STEPS = 50;

double randfrom(double min, double max) {
    double range = (max - min);
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

static void save_image(const std::string &image_name, std::vector<float> density,
                       std::vector<bool> &is_water) {
    std::vector<unsigned char> image;
    image.resize(N * N * 4 * GRID_SIZE * GRID_SIZE);

    float d_min = *std::min_element(std::begin(density), std::end(density));
    float d_max = *std::max_element(std::begin(density), std::end(density));
    int grid_size = GRID_SIZE;
    for (int i = 0; i < N * N; ++i) {
        float d = (density[i] - d_min) / (d_max - d_min);
        for (int j = 0; j < grid_size; ++j) {
            for (int k = 0; k < grid_size; ++k) {
                int indx = 4 * (i % N * grid_size + (i / N) * grid_size * grid_size * N + k + j * N * grid_size);
                image[indx] = (unsigned char) 255.0f * d;
                image[indx + 1] = 0;
                image[indx + 2] = 0;
                image[indx + 3] = 255;
            }
        }

    }

    stbi_write_jpg(image_name.c_str(), N * grid_size, N * grid_size, 4, &image[0], 100);
}

int main() {
    std::vector<float> density;
    density.resize(N * N);
    for (int i = 0; i < N * N; ++i) {
        density[i] = randfrom(0, 1);
    }
    std::vector<float> vx;
    std::vector<bool> is_water;
    is_water.resize(N * N);
    for (int i = 0; i < N * N; ++i) {
        float b = randfrom(0, 1);
        is_water[i] = b >= 0.7;
    }
    vx.resize(N * N);
    for (int i = 0; i < N * N; ++i) {
        vx[i] = randfrom(-5, 5);
    }
    std::vector<float> vy;
    vy.resize(N * N);
    for (int i = 0; i < N * N; ++i) {
        vy[i] = randfrom(-5, 5);
    }
    Solver solver = Solver(N, density, vx, vy, 0.033, 0.001, 0.00001);
    for (int i = 0; i < MAX_STEPS; ++i) {
        std::cout << i << std::endl;
        solver.vel_step();
        solver.dens_step();
        save_image("images/" + std::to_string(i) + ".jpeg", solver.density, is_water);
    }
}
