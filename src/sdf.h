#pragma once

#include "common.h"

std::vector<float> generateSDFData(
    const Float3 *vertices,
    const Float3 *normals,
    size_t        triangleCount,
    int           signRayCount,
    const Float3 &lower,
    const Float3 &upper,
    const Int3   &res);

struct SDF
{
    ComPtr<ID3D11ShaderResourceView> srv;
    Int3                             res;
    Float3                           lower;
    Float3                           upper;
};

class SDFGenerator
{
public:

    void setSignRayCount(int count);

    SDF generate(
        const Float3 *vertices,
        const Float3 *normals,
        size_t        triangleCount,
        const Float3 &lower,
        const Float3 &upper,
        const Int3   &res) const;

    SDF generateGPU(
        const Float3 *vertices,
        const Float3 *normals,
        size_t        triangleCount,
        const Float3 &lower,
        const Float3 &upper,
        const Int3   &res) const;

private:

    int signRayCount_ = 3;

    mutable Shader<CS> shader_;
};
