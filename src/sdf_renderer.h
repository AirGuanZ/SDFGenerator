#pragma once

#include "camera.h"

class SDFRenderer
{
public:

    void initilalize();

    void setCamera(const Camera &camera);

    void setTracer(int maxTraceSteps, float absThreshold);

    void setSDF(
        const Float3                    &lower,
        const Float3                    &upper,
        ComPtr<ID3D11ShaderResourceView> sdf);

    void render();

private:

    struct PSParams
    {
        Float3 frustumA; int   maxTraceSteps;
        Float3 frustumB; float absThreshold;
        Float3 frustumC; float pad0;
        Float3 frustumD; float pad1;
        Float3 eye;      float pad2;
        Float3 lower;    float pad3;
        Float3 upper;    float pad4;
        Float3 extent;   float pad5;
    };
    
    Shader<VS, PS>         shader_;
    Shader<VS, PS>::RscMgr shaderRscs_;

    ShaderResourceViewSlot<PS> *sdfSlot_ = nullptr;
    
    Mat4     viewProj_;
    PSParams psParamsData_ = {};
    
    ConstantBuffer<PSParams>    psParams_;
};
