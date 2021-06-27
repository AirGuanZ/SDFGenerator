#pragma once

#include "camera.h"

class SDFShadowRenderer
{
public:

    void initialize();

    void setCamera(const Camera &camera);

    void setSDF(
        const Float3                    &lower,
        const Float3                    &upper,
        ComPtr<ID3D11ShaderResourceView> srv);

    void setLight(const Float3 &direction, const Float3 &radiance);

    void setTracer(
        float rayOffset, float shadowK, int maxTraceSteps, float absThreshold);

    void render(const VertexBuffer<Vertex> &vertexBuffer);

private:

    struct VSTransform
    {
        Mat4 VP;
    };

    struct PSParams
    {
        Float3 lightDirection;  float shadowRayOffset;
        Float3 lightRadiance;   float shadowK;
        Float3 sdfLower;        int   maxTraceSteps;
        Float3 sdfUpper;        float absThreshold;
        Float3 sdfExtent;       float pad0;
    };

    Shader<VS, PS>         shader_;
    Shader<VS, PS>::RscMgr shaderRscs_;

    ShaderResourceViewSlot<PS> *sdfSlot_ = nullptr;

    ComPtr<ID3D11InputLayout> inputLayout_;

    Mat4                        viewProj_;
    ConstantBuffer<VSTransform> vsTransform_;

    PSParams                    psParamsData_ = {};
    ConstantBuffer<PSParams>    psParams_;
};
