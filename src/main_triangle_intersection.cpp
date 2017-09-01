#include "common.h"
#include "ray.h"
#include "triangle.h"

int main() {
    Vector ray_origin(0, 0, 0);
    Vector ray_direction(0, 1, 0);

    Ray ray(ray_origin, ray_direction);

    Vector p0(-0.5f, 0, -0.5f);
    Vector p1( 0.5f, 0, -0.5f);
    Vector p2( 0, 0, 0.5f);

    const int N = 10'000'000;
    Triangle_Intersection isect;

    Timestamp t;

    for (int i = 0; i < N; i++) {
        intersect_triangle(ray, p0, p1, p2, isect);
    }

    int64_t ns = elapsed_nanoseconds(t);

    double cpu_ghz = get_base_cpu_frequency_ghz();
    printf("CPU frequency = %.2f GHz\n", cpu_ghz);

    double milliseconds_for_all_triangles = ns / 1'000'000.0;
    printf("All triangles intersection time: %.3f milliseconds\n", milliseconds_for_all_triangles);

    double nanoseconds_per_triangle = ns / double(N);
    int clocks = static_cast<int>(nanoseconds_per_triangle * cpu_ghz);
    printf("Single triangle intersection time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_triangle, clocks);

    return 0;
}
