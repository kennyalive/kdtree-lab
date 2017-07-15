#include "random.h"
#include <algorithm>

const float Float_One_Minus_Epsilon = 0.99999994f;

struct pcg_state {
    uint64_t state; // RNG state.  All values are possible.
    uint64_t inc;   // Controls which RNG sequence (stream) is
                    // selected. Must *always* be odd.
};

const pcg_state PCG_INIT_STATE = {0x853c49e6748fea9bULL, 0xda3e39cb94b95bdbULL};

static pcg_state pcg_global = PCG_INIT_STATE;

void random_init() {
    pcg_global = PCG_INIT_STATE;
}

uint32_t random_uint32() {
    uint64_t oldstate = pcg_global.state;
    pcg_global.state = oldstate * 6364136223846793005ULL + pcg_global.inc;
    uint32_t xorshifted = static_cast<uint32_t>(((oldstate >> 18u) ^ oldstate) >> 27u);
    uint32_t rot = oldstate >> 59u;
    return (xorshifted >> rot) | (xorshifted << ((~rot + 1) & 31));
}

float random_float() {
    float r = random_uint32() * 2.3283064365e-10f;
    return std::min(r, Float_One_Minus_Epsilon);
}

float random_from_range(float a, float b) {
  return a + random_float() * (b - a);
}
