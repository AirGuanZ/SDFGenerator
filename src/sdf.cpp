#include <agz-utils/console.h>
#include <agz-utils/thread.h>

#include <d3d11-sdf/bvh.h>
#include <d3d11-sdf/sdf.h>

D3D11_SDF_BEGIN

namespace
{

    template<typename T>
    ComPtr<ID3D11ShaderResourceView> createReadOnlyStructuredBuffer(
        std::span<const T> data)
    {
        D3D11_BUFFER_DESC bufDesc;
        bufDesc.ByteWidth           = static_cast<UINT>(sizeof(T) * data.size());
        bufDesc.Usage               = D3D11_USAGE_IMMUTABLE;
        bufDesc.BindFlags           = D3D11_BIND_SHADER_RESOURCE;
        bufDesc.CPUAccessFlags      = 0;
        bufDesc.MiscFlags           = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
        bufDesc.StructureByteStride = sizeof(T);

        D3D11_SUBRESOURCE_DATA bufSubrscData;
        bufSubrscData.pSysMem          = data.data();
        bufSubrscData.SysMemPitch      = 0;
        bufSubrscData.SysMemSlicePitch = 0;

        auto buf = device.createBuffer(bufDesc, &bufSubrscData);

        D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
        srvDesc.Format              = DXGI_FORMAT_UNKNOWN;
        srvDesc.ViewDimension       = D3D11_SRV_DIMENSION_BUFFER;
        srvDesc.Buffer.FirstElement = 0;
        srvDesc.Buffer.NumElements  = static_cast<UINT>(data.size());

        return device.createSRV(buf, srvDesc);
    }

} // namespace anonymous

std::vector<float> generateSDFData(
    const Float3 *vertices,
    const Float3 *normals,
    size_t        triangleCount,
    int           signRayCount,
    const Float3 &lower,
    const Float3 &upper,
    const Int3   &res)
{
    BVH bvh;
    bvh.build(vertices, normals, triangleCount);

    std::vector<float> result;
    result.resize(res.product());

    const float dx = 1.05f * (upper.x - lower.x) / res.x;

    agz::console::progress_bar_f_t pbar(80, '=');
    pbar.display();

    agz::thread::parallel_forrange(
        0, res.z,
        [&](int threadIdx, int z)
    {
        const float zf = std::lerp(lower.z, upper.z, (z + 0.5f) / res.z);
        for(int y = 0; y < res.y; ++y)
        {
            const float yf = std::lerp(lower.y, upper.y, (y + 0.5f) / res.y);
            for(int x = 0; x < res.x; ++x)
            {
                const size_t idx = z * res.x * res.y + y * res.x + x;

                const float xf = std::lerp(lower.x, upper.x, (x + 0.5f) / res.x);

                const float upperBound =
                    x == 0 ? -1.0f : (std::abs(result[idx - 1]) + dx);
                const float sdf = bvh.sdf(
                    { xf, yf, zf }, signRayCount, upperBound);

                result[idx] = sdf;
            }
        }

        if(threadIdx == 0)
        {
            pbar.set_percent(100.0f * (z + 1) / res.z);
            pbar.display();
        }
    }, -1);

    pbar.done();
    return result;
}

void SDFGenerator::setSignRayCount(int count)
{
    signRayCount_ = count;
}

std::vector<float> SDFGenerator::generate(
    const Float3 *vertices,
    const Float3 *normals,
    size_t        triangleCount,
    const Float3 &lower,
    const Float3 &upper,
    const Int3   &res) const
{
    return generateSDFData(
        vertices, normals, triangleCount,
        signRayCount_, lower, upper, res);
}

SDF SDFGenerator::generateGPU(
    const Float3 *vertices,
    const Float3 *normals,
    size_t        triangleCount,
    const Float3 &lower,
    const Float3 &upper,
    const Int3   &res) const
{
    // prepare shader

    const char *SDF_SRC =
#include <d3d11-sdf/sdf.hlsl>
    ;

    if(!shader_.isAllStageAvailable())
        shader_.initializeStage<CS>(SDF_SRC, nullptr, "CSMain");

    auto shaderRscs = shader_.createResourceManager();

    // upload bvh

    BVH bvh;
    bvh.build(vertices, normals, triangleCount);
    
    auto verticesSRV = createReadOnlyStructuredBuffer(bvh.getVertices());
    auto normalsSRV  = createReadOnlyStructuredBuffer(bvh.getNormals());
    auto nodesSRV    = createReadOnlyStructuredBuffer(bvh.getNodes());

    shaderRscs.getShaderResourceViewSlot<CS>("Vertices")
        ->setShaderResourceView(verticesSRV);
    shaderRscs.getShaderResourceViewSlot<CS>("Normals")
        ->setShaderResourceView(normalsSRV);
    shaderRscs.getShaderResourceViewSlot<CS>("Nodes")
        ->setShaderResourceView(nodesSRV);

    // constant buffer

    struct CSParams
    {
        Float3 lower;  int triangleCount;
        Float3 upper;  int signRayCount;
        Float3 extent; int xBeg;

        int xEnd;
        float pad0;
        float pad1;
        float pad2;
    };

    ConstantBuffer<CSParams> csParams;
    csParams.initialize();
    shaderRscs.getConstantBufferSlot<CS>("CSParams")->setBuffer(csParams);

    // create outputs

    D3D11_TEXTURE3D_DESC outputTexDesc;
    outputTexDesc.Width          = static_cast<UINT>(res.x);
    outputTexDesc.Height         = static_cast<UINT>(res.y);
    outputTexDesc.Depth          = static_cast<UINT>(res.z);
    outputTexDesc.MipLevels      = 1;
    outputTexDesc.Format         = DXGI_FORMAT_R32_FLOAT;
    outputTexDesc.Usage          = D3D11_USAGE_DEFAULT;
    outputTexDesc.BindFlags      = D3D11_BIND_UNORDERED_ACCESS;
    outputTexDesc.CPUAccessFlags = 0;
    outputTexDesc.MiscFlags      = 0;
    auto outputTex = device.createTex3D(outputTexDesc);

    D3D11_UNORDERED_ACCESS_VIEW_DESC outputUAVDesc;
    outputUAVDesc.Format                = DXGI_FORMAT_R32_FLOAT;
    outputUAVDesc.ViewDimension         = D3D11_UAV_DIMENSION_TEXTURE3D;
    outputUAVDesc.Texture3D.FirstWSlice = 0;
    outputUAVDesc.Texture3D.WSize       = static_cast<UINT>(res.z);
    outputUAVDesc.Texture3D.MipSlice    = 0;
    auto outputUAV = device.createUAV(outputTex, outputUAVDesc);

    outputTexDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    auto resultTex = device.createTex3D(outputTexDesc);

    D3D11_SHADER_RESOURCE_VIEW_DESC resultSRVDesc;
    resultSRVDesc.Format                    = DXGI_FORMAT_R32_FLOAT;
    resultSRVDesc.ViewDimension             = D3D11_SRV_DIMENSION_TEXTURE3D;
    resultSRVDesc.Texture3D.MipLevels       = 1;
    resultSRVDesc.Texture3D.MostDetailedMip = 0;
    auto resultSRV = device.createSRV(resultTex, resultSRVDesc);

    shaderRscs.getUnorderedAccessViewSlot<CS>("SDF")
        ->setUnorderedAccessView(outputUAV);

    // dispatch

    shader_.bind();
    shaderRscs.bind();

    constexpr int GROUP_SIZE = 8;
    const int groupCountY = (res.y + GROUP_SIZE - 1) / GROUP_SIZE;
    const int groupCountZ = (res.z + GROUP_SIZE - 1) / GROUP_SIZE;

    constexpr int X_SLICE_SIZE = 32;
    for(int xBeg = 0; xBeg < res.x; xBeg += X_SLICE_SIZE)
    {
        csParams.update({
            lower, static_cast<int>(triangleCount),
            upper, signRayCount_,
            upper - lower, xBeg,
            (std::min)(res.x, xBeg + X_SLICE_SIZE),
            0, 0, 0
            });

        deviceContext.dispatch(1, groupCountY, groupCountZ);
    }

    shaderRscs.unbind();
    shader_.unbind();

    // result

    deviceContext->CopyResource(resultTex.Get(), outputTex.Get());

    SDF result;
    result.tex   = std::move(resultTex);
    result.srv   = std::move(resultSRV);
    result.res   = res;
    result.lower = lower;
    result.upper = upper;

    return result;
}

D3D11_SDF_END
