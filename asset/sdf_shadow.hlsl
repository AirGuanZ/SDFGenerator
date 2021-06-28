cbuffer VSTransform
{
    float4x4 WorldToClip;
}

struct VSInput
{
    float3 position : POSITION;
    float3 normal   : NORMAL;
    float3 color    : COLOR;
};

struct VSOutput
{
    float4 position      : SV_POSITION;
    float3 worldPosition : WORLD_POSITION;
    float3 worldNormal   : WORLD_NORMAL;
    float3 color         : COLOR;
};

VSOutput VSMain(VSInput input)
{
    VSOutput output;
    output.position      = mul(float4(input.position, 1), WorldToClip);
    output.worldPosition = input.position;
    output.worldNormal   = input.normal;
    output.color         = input.color;
    return output;
}

cbuffer PSParams
{
    float3 LightDirection; float ShadowRayOffset;
    float3 LightRadiance;  float ShadowK;
    float3 SDFLower;       int   MaxTraceSteps;
    float3 SDFUpper;       float AbsThreshold;
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

// see https://www.iquilezles.org/www/articles/rmshadows/rmshadows.htm
float shadowFactor(float3 o, float3 d)
{
    float2 incts = intersectRayBox(o, d);
    if(incts.x >= incts.y)
        return 1;

    float result = 1;
    float ph = 1e20;

    float t = incts.x;
    for(int i = 0; i < MaxTraceSteps; ++i)
    {
        float3 p = o + t * d;
        float3 uvw = (p - SDFLower) / SDFExtent;
        float sdf = SDF.SampleLevel(SDFSampler, uvw, 0);
        float udf = abs(sdf);

        float y = udf * udf / (2.0 * ph);
        float m = sqrt(udf * udf - y * y);
        result = min(result, ShadowK * m / max(0.0f, t - y));
        ph = udf;
        //result = min(result, ShadowK * udf / t);

        if(udf < AbsThreshold)
            return 0;

        t += udf;
        if(t >= incts.y)
            break;
    }

    return result;
}

float4 PSMain(VSOutput input) : SV_TARGET
{
    float3 d = -LightDirection;
    float3 o = input.worldPosition + ShadowRayOffset * input.worldNormal;

    float shadow = shadowFactor(o, d);
    float cosFac = max(0.0f, dot(input.worldNormal, -LightDirection));

    float3 result = LightRadiance * cosFac * shadow;
    return float4(pow(result, 1 / 2.2f), 1);
}
