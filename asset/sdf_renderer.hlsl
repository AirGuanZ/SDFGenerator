struct VSOutput
{
    float4 position : SV_POSITION;
    float2 texCoord : TEXCOORD;
};

VSOutput VSMain(uint vertexID : SV_VertexID)
{
    VSOutput output;
    output.texCoord = float2((vertexID << 1) & 2, vertexID & 2);
    output.position = float4(output.texCoord * float2(2, -2) + float2(-1, 1), 0.5, 1);
    return output;
}

cbuffer PSParams
{
    float3 FrustumA; int   MaxTraceSteps;
    float3 FrustumB; float AbsThreshold;
    float3 FrustumC;
    float3 FrustumD;

    float3 Eye;

    float3 SDFLower;
    float3 SDFUpper;
    float3 SDFExtent;
}

Texture3D<float> SDF;
SamplerState     SDFSampler;

float max3(float x, float y, float z)
{
    return max(x, max(y, z));
}

float min3(float x, float y, float z)
{
    return min(x, min(y, z));
}

float2 intersectRayBox(float3 o, float3 d)
{
    float3 invD = 1 / d;
    float3 n = invD * (SDFLower - o);
    float3 f = invD * (SDFUpper - o);

    float3 minnf = min(n, f);
    float3 maxnf = max(n, f);

    float t0 = max3(minnf.x, minnf.y, minnf.z);
    float t1 = min3(maxnf.x, maxnf.y, maxnf.z);

    return float2(max(0.0f, t0), t1);
}

float4 PSMain(VSOutput input) : SV_TARGET
{
    float3 o = Eye;
    float3 d = normalize(lerp(
        lerp(FrustumA, FrustumB, input.texCoord.x),
        lerp(FrustumC, FrustumD, input.texCoord.x), input.texCoord.y));

    float2 incts = intersectRayBox(o, d);
    if(incts.x >= incts.y)
        return float4(0, 0, 0, 1);

    float t = incts.x + 0.01;
    int i = 0;

    for(; i < MaxTraceSteps; ++i)
    {
        float3 p = o + t * d;
        float3 uvw = (p - SDFLower) / SDFExtent;
        if(any(saturate(uvw) != uvw))
            break;

        float sdf = SDF.SampleLevel(SDFSampler, uvw, 0);
        float udf = abs(sdf);
        if(udf <= AbsThreshold)
            break;

        t += udf;
    }

    float color = float(i) / (MaxTraceSteps - 1);
    color = pow(color, 1 / 2.2);
    return float4(color.xxx, 1);
}
