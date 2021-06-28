#pragma once

#include <d3d11-sdf/common.h>

D3D11_SDF_BEGIN

class Ray
{
public:

    Float3 o;
    Float3 d;
    float t0;
    float t1;

    Ray()
        : Ray({}, { 1, 0, 0 })
    {
        
    }

    Ray(
        const Float3 &o,
        const Float3 &d,
        float         t0 = 0,
        float         t1 = std::numeric_limits<float>::infinity())
        : o(o), d(d), t0(t0), t1(t1)
    {
        
    }

    bool isBetween(float t) const
    {
        return t0 <= t && t <= t1;
    }
};

D3D11_SDF_END
