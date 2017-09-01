#include "common.h"
#include "random.h"
#include "ray_generator.h"
#include "vector.h"

Ray_Generator::Ray_Generator(const Bounding_Box& mesh_bounds) {
    auto diagonal = mesh_bounds.max_point - mesh_bounds.min_point;
    float delta = 2.0f * diagonal.length();

    auto p_min = mesh_bounds.min_point - Vector(delta);
    auto p_max = mesh_bounds.max_point + Vector(delta);
    ray_bounds = Bounding_Box(p_min, p_max);
}

Ray Ray_Generator::generate_ray(const Vector& last_hit, float last_hit_epsilon) const {
    // Ray origin.
    Vector origin;
    origin.x = random_from_range(ray_bounds.min_point.x, ray_bounds.max_point.x);
    origin.y = random_from_range(ray_bounds.min_point.y, ray_bounds.max_point.y);
    origin.z = random_from_range(ray_bounds.min_point.z, ray_bounds.max_point.z);

    const bool use_last_hit = random_float() < 0.25f;
    if (use_last_hit)
        origin = last_hit;

    // Ray direction.
    auto direction = uniform_sample_sphere();
    auto len = direction.length();

    if (random_float() < 1.0f / 32.0f && direction.z != 0.0)
        direction.x = direction.y = 0.0;
    else if (random_float() < 1.0f / 32.0f && direction.y != 0.0)
        direction.x = direction.z = 0.0;
    else if (random_float() < 1.0f / 32.0f && direction.x != 0.0)
        direction.y = direction.z = 0.0;
    direction = direction.normalized();

    auto ray = Ray(origin, direction);
    ray.Advance(use_last_hit ? last_hit_epsilon : 1e-3f);
    return ray;
}
