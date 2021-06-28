#include "sdf_renderer.h"

void SDFRenderer::initilalize()
{
    shader_.initializeStageFromFile<VS>(
        "./asset/sdf_renderer.hlsl", nullptr, "VSMain");
    shader_.initializeStageFromFile<PS>(
        "./asset/sdf_renderer.hlsl", nullptr, "PSMain");

    shaderRscs_ = shader_.createResourceManager();

    sdfSlot_ = shaderRscs_.getShaderResourceViewSlot<PS>("SDF");
    
    psParams_.initialize();
    shaderRscs_.getConstantBufferSlot<PS>("PSParams")
        ->setBuffer(psParams_);

    auto sdfSampler = device.createSampler(
        D3D11_FILTER_MIN_MAG_MIP_LINEAR,
        D3D11_TEXTURE_ADDRESS_CLAMP,
        D3D11_TEXTURE_ADDRESS_CLAMP,
        D3D11_TEXTURE_ADDRESS_CLAMP);
    shaderRscs_.getSamplerSlot<PS>("SDFSampler")
        ->setSampler(std::move(sdfSampler));
}

void SDFRenderer::setCamera(const Camera &camera)
{
    const auto frustumDirs = camera.getFrustumDirections();

    viewProj_ = camera.getViewProj();
    psParamsData_.frustumA = frustumDirs.frustumA;
    psParamsData_.frustumB = frustumDirs.frustumB;
    psParamsData_.frustumC = frustumDirs.frustumC;
    psParamsData_.frustumD = frustumDirs.frustumD;
    psParamsData_.eye = camera.getPosition();
}

void SDFRenderer::setTracer(int maxTraceSteps, float absThreshold)
{
    psParamsData_.maxTraceSteps = maxTraceSteps;
    psParamsData_.absThreshold  = absThreshold;
}

void SDFRenderer::setSDF(
    const Float3                    &lower,
    const Float3                    &upper,
    ComPtr<ID3D11ShaderResourceView> sdf)
{
    psParamsData_.lower  = lower;
    psParamsData_.upper  = upper;
    psParamsData_.extent = upper - lower;
    sdfSlot_->setShaderResourceView(std::move(sdf));
}

void SDFRenderer::render()
{
    psParams_.update(psParamsData_);

    shader_.bind();
    shaderRscs_.bind();

    deviceContext.setPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    deviceContext.draw(6, 0);

    shaderRscs_.unbind();
    shader_.unbind();
}
