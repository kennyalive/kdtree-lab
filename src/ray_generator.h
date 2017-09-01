#pragma once

#include "bounding_box.h"

struct Vector;

class Ray_Generator {
public:
    Ray_Generator(const Bounding_Box& mesh_bounds);
    Ray generate_ray(const Vector& last_hit, float last_hit_epsilon) const;

private:
    Bounding_Box ray_bounds;
};
