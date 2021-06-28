#include <iostream>
#include <fstream>

#include <agz-utils/mesh.h>
#include <d3d11-sdf/sdf.h>
#include <cxxopts.hpp>

using namespace d3d11_sdf;

#define INFO(MSG) do { std::cout << (MSG) << std::endl; } while(false)

struct CLIArguments
{
    std::string inputPath;
    std::string outputPath;

    Float3 bboxMin;
    Float3 bboxMax;
    Int3   resolution;

    int signRayCount = 12;

    bool gpu = true;
};

std::optional<CLIArguments> parseCLIArgs(int argc, char *argv[])
{
    cxxopts::Options options("D3D11SDF-CLI");
    options.add_options()
        ("h", "help information")
        ("i", "input filename",                      cxxopts::value<std::string>())
        ("o", "output filename",                     cxxopts::value<std::string>())
        ("r", "resolution       (x,y,z)",            cxxopts::value<std::vector<int>>()->default_value("64"))
        ("q", "quality          (positive integer)", cxxopts::value<int>()->default_value("12"))
        ("s", "bounding box min (x,y,z)",            cxxopts::value<std::vector<float>>())
        ("l", "bounding box max (x,y,z)",            cxxopts::value<std::vector<float>>())
        ("g", "use gpu          (true/false)",       cxxopts::value<std::string>()->default_value("true"));

    auto parseResult = options.parse(argc, argv);

    if(parseResult.count("h"))
    {
        std::cout << options.help() << std::endl;
        return std::nullopt;
    }

    CLIArguments args;
    args.inputPath  = parseResult["i"].as<std::string>();
    args.outputPath = parseResult["o"].as<std::string>();

    const auto bboxMin = parseResult["s"].as<std::vector<float>>();
    if(bboxMin.size() == 1)
        args.bboxMin = Float3(bboxMin[0]);
    else if(bboxMin.size() == 3)
        args.bboxMin = Float3(bboxMin[0], bboxMin[1], bboxMin[2]);
    else
        throw std::runtime_error("invalid number of floats for 'bounding box min'");

    const auto bboxMax = parseResult["l"].as<std::vector<float>>();
    if(bboxMax.size() == 1)
        args.bboxMax = Float3(bboxMax[0]);
    else if(bboxMax.size() == 3)
        args.bboxMax = Float3(bboxMax[0], bboxMax[1], bboxMax[2]);
    else
        throw std::runtime_error("invalid number of floats for 'bounding box max'");

    const auto res = parseResult["r"].as<std::vector<int>>();
    if(res.size() == 1)
        args.resolution = Int3(res[0]);
    else if(res.size() == 3)
        args.resolution = Int3(res[0], res[1], res[2]);
    else
        throw std::runtime_error("invalid number of integers for 'resolution'");

    args.signRayCount = parseResult["q"].as<int>();
    args.gpu          = parseResult["g"].as<std::string>() == "true";

    return args;
}

void run(const CLIArguments &args)
{
    INFO("load mesh from " + args.inputPath);

    const auto tris = agz::mesh::load_from_file(args.inputPath);
    std::vector<Float3> vertices, normals;
    for(auto &tri : tris)
    {
        for(auto &v : tri.vertices)
        {
            vertices.push_back(v.position);
            normals.push_back(v.normal);
        }
    }

    std::vector<float> sdfData;

    if(args.gpu)
    {
        INFO("initialize d3d11");

        ComPtr<ID3D11Device>        d3dDevice;
        ComPtr<ID3D11DeviceContext> d3dDeviceContext;
        const D3D_FEATURE_LEVEL     featureLevel = D3D_FEATURE_LEVEL_11_1;

        if(FAILED(D3D11CreateDevice(
            nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
            D3D11_CREATE_DEVICE_DISABLE_GPU_TIMEOUT,
            &featureLevel, 1, D3D11_SDK_VERSION,
            d3dDevice.GetAddressOf(), nullptr,
            d3dDeviceContext.GetAddressOf())))
        {
            throw std::runtime_error(
                "failed to create d3d1 device & device context");
        }

        agz::d3d11::device.d3dDevice               = d3dDevice.Get();
        agz::d3d11::deviceContext.d3dDeviceContext = d3dDeviceContext.Get();
        AGZ_SCOPE_GUARD({
            agz::d3d11::device.d3dDevice = nullptr;
            agz::d3d11::deviceContext.d3dDeviceContext = nullptr;
        });

        INFO("generate sdf");

        d3d11_sdf::SDFGenerator generator;
        generator.setSignRayCount(args.signRayCount);
        auto sdf = generator.generateGPU(
            vertices.data(), normals.data(), tris.size(),
            args.bboxMin, args.bboxMax, args.resolution);

        INFO("readback data");

        D3D11_TEXTURE3D_DESC texDesc;
        sdf.tex->GetDesc(&texDesc);
        texDesc.BindFlags      = 0;
        texDesc.Usage          = D3D11_USAGE_STAGING;
        texDesc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        auto stagingTex = device.createTex3D(texDesc);

        deviceContext->CopyResource(stagingTex.Get(), sdf.tex.Get());

        D3D11_MAPPED_SUBRESOURCE mappedSubrsc;
        deviceContext->Map(stagingTex.Get(), 0, D3D11_MAP_READ, 0, &mappedSubrsc);

        sdfData.resize(args.resolution.product());
        const size_t rowSize = sizeof(float) * args.resolution.x;
        char *copySrc = static_cast<char*>(mappedSubrsc.pData);
        char *copyDst = reinterpret_cast<char*>(sdfData.data());
    
        for(int z = 0; z < args.resolution.z; ++z)
        {
            for(int y = 0; y < args.resolution.y; ++y)
            {
                auto row = copySrc + mappedSubrsc.RowPitch * y;
                std::memcpy(copyDst, row, rowSize);
                copyDst += rowSize;
            }
            copySrc += mappedSubrsc.DepthPitch;
        }
    }
    else
    {
        INFO("generate sdf");

        d3d11_sdf::SDFGenerator generator;
        generator.setSignRayCount(args.signRayCount);
        sdfData = generator.generate(
            vertices.data(), normals.data(), tris.size(),
            args.bboxMin, args.bboxMax, args.resolution);
    }

    INFO("write to " + args.outputPath);

    std::ofstream fout(args.outputPath, std::ofstream::out);
    if(!fout)
    {
        throw std::runtime_error(
            "failed to open output file: " + args.outputPath);
    }

    fout << args.resolution.x << " "
         << args.resolution.y << " "
         << args.resolution.z << std::endl;

    int i = 0;
    for(int z = 0; z < args.resolution.z; ++z)
    {
        for(int y = 0; y < args.resolution.y; ++y)
        {
            for(int x = 0; x < args.resolution.x; ++x)
                fout << sdfData[i++] << " ";
        }
    }
}

int main(int argc, char *argv[])
{
    try
    {
        const auto optArgs = parseCLIArgs(argc, argv);
        if(optArgs)
            run(*optArgs);
    }
    catch(const std::exception &err)
    {
        std::cerr << err.what() << std::endl;
        return -1;
    }
    catch(...)
    {
        std::cerr << "an unknown error occurred" << std::endl;
        return -1;
    }
}
