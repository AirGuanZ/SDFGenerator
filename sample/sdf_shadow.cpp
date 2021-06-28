#include "sdf_shadow.h"

void SDFShadowRenderer::initialize()
{
    shader_.initializeStageFromFile<VS>(
        "./asset/sdf_shadow.hlsl", nullptr, "VSMain");
    shader_.initializeStageFromFile<PS>(
        "./asset/sdf_shadow.hlsl", nullptr, "PSMain");

    shaderRscs_ = shader_.createResourceManager();

    sdfSlot_ = shaderRscs_.getShaderResourceViewSlot<PS>("SDF");

    const D3D11_INPUT_ELEMENT_DESC inputElems[] = {
        {
            "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT,
            0, offsetof(Vertex, position),
            D3D11_INPUT_PER_VERTEX_DATA, 0
        },
        {
            "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT,
            0, offsetof(Vertex, normal),
            D3D11_INPUT_PER_VERTEX_DATA, 0
        },
        {
            "COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT,
            0, offsetof(Vertex, color),
            D3D11_INPUT_PER_VERTEX_DATA, 0
        },
    };
    inputLayout_ = InputLayoutBuilder(inputElems).build(shader_);

    vsTransform_.initialize();
    shaderRscs_.getConstantBufferSlot<VS>("VSTransform")
        ->setBuffer(vsTransform_);

    psParams_.initialize();
    shaderRscs_.getConstantBufferSlot<PS>("PSParams")
        ->setBuffer(psParams_);

    auto sdfSampler = device.createSampler(
        D3D11_FILTER_MIN_MAG_MIP_LINEAR,
        D3D11_TEXTURE_ADDRESS_CLAMP,
        D3D11_TEXTURE_ADDRESS_CLAMP,
        D3D11_TEXTURE_ADDRESS_CLAMP);
    shaderRscs_.getSamplerSlot<PS>("SDFSampler")
        ->setSampler(sdfSampler);
}

void SDFShadowRenderer::setCamera(const Camera &camera)
{
    viewProj_ = camera.getViewProj();
}

void SDFShadowRenderer::setSDF(
    const Float3                    &lower,
    const Float3                    &upper,
    ComPtr<ID3D11ShaderResourceView> srv)
{
    psParamsData_.sdfLower = lower;
    psParamsData_.sdfUpper = upper;
    psParamsData_.sdfExtent = upper - lower;
    sdfSlot_->setShaderResourceView(std::move(srv));
}

void SDFShadowRenderer::setLight(
    const Float3 &direction, const Float3 &radiance)
{
    psParamsData_.lightDirection = direction;
    psParamsData_.lightRadiance  = radiance;
}

void SDFShadowRenderer::setTracer(
    float rayOffset, float shadowK, int maxTraceSteps, float absThreshold)
{
    psParamsData_.shadowRayOffset = rayOffset;
    psParamsData_.shadowK         = shadowK;
    psParamsData_.maxTraceSteps   = maxTraceSteps;
    psParamsData_.absThreshold    = absThreshold;
}

void SDFShadowRenderer::render(const VertexBuffer<Vertex> &vertexBuffer)
{
    vsTransform_.update({ viewProj_ });
    psParams_.update(psParamsData_);

    shader_.bind();
    shaderRscs_.bind();
    deviceContext.setInputLayout(inputLayout_);
    vertexBuffer.bind(0);
    deviceContext.setPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    deviceContext.draw(vertexBuffer.getVertexCount(), 0);

    vertexBuffer.unbind(0);
    deviceContext.setInputLayout(nullptr);
    shaderRscs_.unbind();
    shader_.unbind();
}
