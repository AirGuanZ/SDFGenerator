#pragma once

#include <span>

#include <d3d11-sdf/common.h>

D3D11_SDF_BEGIN

class BVH
{
public:

    struct Node
    {
        float    bbox[6];
        uint32_t childIndex;
        uint32_t childCount; // childCount is always 0 for interior nodes

        bool isLeaf() const;
    };

    static_assert(sizeof(Node) == sizeof(float) * 8);

    void build(
        const Float3 *vertices, const Float3 *normals, size_t triangleCount);

    std::span<const Node> getNodes() const;

    std::span<const Float3> getVertices() const;

    std::span<const Float3> getNormals() const;

    float estimateUpperBound(const Float3 &p, int precison) const;

    float sdf(const Float3 &p, int signRayCount, float upperBound = -1) const;

private:
    
    size_t traceTriangleIndex(
        const Float3 &a, const Float3 &d, float maxT) const;

    bool containsTriangle(const Float3 &o, float radius2, size_t nodeIndex) const;

    std::pair<size_t, float> udf2(
        const Float3 &p, float u2, size_t nodeIndex) const;

    int triangleCount_ = 0;

    std::vector<Node>   nodes_;
    std::vector<Float3> vertices_;
    std::vector<Float3> normals_;
};

D3D11_SDF_END
