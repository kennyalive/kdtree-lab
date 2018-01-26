#pragma once

#include "random.h"
#include "vector.h"

#include <xmmintrin.h> 

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

enum { benchmark_ray_count = 1'000'000 };
enum { debug_rays = false };
enum { debug_ray_count = 4 };

struct Timestamp {
    Timestamp() : t(std::chrono::steady_clock::now()) {}
    const std::chrono::time_point<std::chrono::steady_clock> t;
};

int64_t elapsed_milliseconds(Timestamp timestamp);
int64_t elapsed_nanoseconds(Timestamp timestamp);

double get_base_cpu_frequency_ghz();

inline void error(const std::string& message) {
    printf("error: %s\n", message.c_str());
    exit(1);
}

// x range: [-PI,PI]
inline float fast_sine(float x) {
    constexpr float PI = 3.14159265358f;
    constexpr float B = 4.0f / PI;
    constexpr float C = -4.0f / (PI * PI);
    constexpr float P = 0.225f;

    float y = B * x + C * x * (x < 0 ? -x : x);
    return P * (y * (y < 0 ? -y : y) - y) + y;
}

// x range: [-PI, PI]
inline float fast_cosine(float x) {
    constexpr float PI = 3.14159265358f;
    constexpr float B = 4.0f / PI;
    constexpr float C = -4.0f / (PI * PI);
    constexpr float P = 0.225f;

    x = (x > 0) ? -x : x;
    x += PI/2;

    return fast_sine(x);
}

inline float fast_sqrt(float v) {
    float out;
    __m128 in = _mm_load_ss(&v);
    _mm_store_ss(&out, _mm_mul_ss(in, _mm_rsqrt_ss(in)));
    return out;
}

inline Vector uniform_sample_sphere() {
    constexpr float PI = 3.14159265358f;

    auto u1 = random_float();
    auto u2 = random_float();

    float z = 1.0f - 2.0f * u1;
    float r = fast_sqrt(1.0f - z*z);
    float phi = PI * (-1.0f + 2.0f * u2);
    float x = r * fast_cosine(phi);
    float y = r * fast_sine(phi);
    return Vector(x, y, z);
}

template <typename T>
void print_vector(const std::vector<T>& v) {
    printf("[%lld]: (", v.size());
    for (std::size_t i = 0; i < v.size(); i++) {
        printf("%d", v[i]);
        if (i != v.size() - 1)
            printf(", ");
    }
    printf(")\n");
}
