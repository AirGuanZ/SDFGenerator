#include <random>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4068)
#pragma warning(disable: 4141)
#pragma warning(disable: 4267)
#endif

#include <bvh/bvh.hpp>
#include <bvh/leaf_collapser.hpp>
#include <bvh/locally_ordered_clustering_builder.hpp>
#include <bvh/triangle.hpp>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <d3d11-sdf/bvh.h>
#include <d3d11-sdf/ray.h>

D3D11_SDF_BEGIN

namespace
{

    constexpr float FLOAT_INF = std::numeric_limits<float>::infinity();

    float dot2(const Float3 &v)
    {
        return dot(v, v);
    }

    float sign(float v)
    {
        return v < 0 ? -1.0f : (v > 0 ? 1.0f : 0.0f);
    }

    bvh::Vector3<float> convVec3(const Float3 &v)
    {
        return { v.x, v.y, v.z };
    }

    bvh::BoundingBox<float> convBBox(const agz::math::aabb3f &bbox)
    {
        return { convVec3(bbox.low), convVec3(bbox.high) };
    }
    
    bool closestIntersectionWithTriangle(
        const Ray    &r,
        const Float3 &A,
        const Float3 &B_A,
        const Float3 &C_A,
        float        *r_t,
        Float2       *uv) noexcept
    {
        const Float3 s1 = cross(r.d, C_A);
        const float div = dot(s1, B_A);
        if(div == 0.0f)
            return false;
        const float invDiv = 1 / div;

        const Float3 o_A = r.o - A;
        const float alpha = dot(o_A, s1) * invDiv;
        if(alpha < 0)
            return false;

        const Float3 s2 = cross(o_A, B_A);
        const float beta = dot(r.d, s2) * invDiv;
        if(beta < 0 || alpha + beta > 1)
            return false;

        const float t = dot(C_A, s2) * invDiv;
        if(!r.isBetween(t))
            return false;

        *r_t = t;
        *uv  = Float2(alpha, beta);

        return true;
    }
    
    bool intersectRayBox(
        const float     *bbox,
        const Float3    &ori,
        const Float3    &invDir,
        float            t0,
        float            t1)
    {
        const float nx = invDir[0] * (bbox[0] - ori[0]);
        const float ny = invDir[1] * (bbox[1] - ori[1]);
        const float nz = invDir[2] * (bbox[2] - ori[2]);

        const float fx = invDir[0] * (bbox[3] - ori[0]);
        const float fy = invDir[1] * (bbox[4] - ori[1]);
        const float fz = invDir[2] * (bbox[5] - ori[2]);

        t0 = (std::max)(t0, (std::min)(nx, fx));
        t0 = (std::max)(t0, (std::min)(ny, fy));
        t0 = (std::max)(t0, (std::min)(nz, fz));

        t1 = (std::min)(t1, (std::max)(nx, fx));
        t1 = (std::min)(t1, (std::max)(ny, fy));
        t1 = (std::min)(t1, (std::max)(nz, fz));

        return t0 <= t1;
    }

    float udf2Triangle(
        const Float3 &a, const Float3 &b, const Float3 &c, const Float3 &p)
    {
        const Float3 ba = b - a; const Float3 pa = p - a;
        const Float3 cb = c - b; const Float3 pb = p - b;
        const Float3 ac = a - c; const Float3 pc = p - c;
        const Float3 nor = cross(ba, ac);
        
        if(sign(dot(cross(ba, nor), pa)) +
           sign(dot(cross(cb, nor), pb)) +
           sign(dot(cross(ac, nor), pc)) < 2.0)
        {
            return (std::min)((std::min)(
                dot2(ba * agz::math::clamp(dot(ba, pa) / dot2(ba), 0.0f, 1.0f) - pa),
                dot2(cb * agz::math::clamp(dot(cb, pb) / dot2(cb), 0.0f, 1.0f) - pb)),
                dot2(ac * agz::math::clamp(dot(ac, pc) / dot2(ac), 0.0f, 1.0f) - pc));
        }

        return dot(nor, pa) * dot(nor, pa) / dot2(nor);
    }

    bool intersectSphereBox(const float *bbox, const Float3 &o, float r2)
    {
        const Float3 q = {
            agz::math::clamp(o.x, bbox[0], bbox[3]),
            agz::math::clamp(o.y, bbox[1], bbox[4]),
            agz::math::clamp(o.z, bbox[2], bbox[5]),
        };
        const float dist2 = distance2(o, q);
        return dist2 <= r2;
    }

    bool intersectSphereTriangle(
        const Float3 &a, const Float3 &b, const Float3 &c,
        const Float3 &o, float r2)
    {
        return udf2Triangle(a, b, c, o) <= r2;
    }

    constexpr size_t TRI_IDX_NIL = (std::numeric_limits<size_t>::max)();

} // namespace anonymous

bool BVH::Node::isLeaf() const
{
    return childCount > 0;
}

void BVH::build(
    const Float3 *vertices, const Float3 *normals, size_t triangleCount)
{
    // build

    agz::math::aabb3f globalBBox = { Float3(FLOAT_INF), Float3(-FLOAT_INF) };
    std::vector<bvh::BoundingBox<float>> primBBoxes(triangleCount);
    std::vector<bvh::Vector3<float>>     primCenters(triangleCount);
    for(size_t i = 0, j = 0; i < triangleCount; ++i, j += 3)
    {
        agz::math::aabb3f primBBox = { Float3(FLOAT_INF), Float3(-FLOAT_INF) };
        primBBox |= vertices[j];
        primBBox |= vertices[j + 1];
        primBBox |= vertices[j + 2];
        primBBoxes[i] = convBBox(primBBox);

        globalBBox |= primBBox;

        const auto center =
            1.0f / 3 * (vertices[j] + vertices[j + 1] + vertices[j + 2]);
        primCenters[i] = convVec3(center);
    }

    bvh::Bvh<float> tree;
    bvh::LocallyOrderedClusteringBuilder<bvh::Bvh<float>, uint32_t> builder(tree);

    builder.build(
        convBBox(globalBBox),
        primBBoxes.data(),
        primCenters.data(),
        triangleCount);

    bvh::LeafCollapser<bvh::Bvh<float>> leafCollapser(tree);
    leafCollapser.collapse();

    // convert
    
    vertices_.clear();
    normals_.clear();

    nodes_.resize(tree.node_count);

    for(size_t ni = 0; ni < tree.node_count; ++ni)
    {
        auto &node = tree.nodes[ni];
        nodes_[ni].bbox[0] = node.bounds[0];
        nodes_[ni].bbox[1] = node.bounds[2];
        nodes_[ni].bbox[2] = node.bounds[4];
        nodes_[ni].bbox[3] = node.bounds[1];
        nodes_[ni].bbox[4] = node.bounds[3];
        nodes_[ni].bbox[5] = node.bounds[5];

        if(node.is_leaf())
        {
            const size_t primBeg = vertices_.size() / 3;
            const size_t iEnd =
                node.first_child_or_primitive + node.primitive_count;
            for(size_t i = node.first_child_or_primitive;
                i < iEnd; ++i)
            {
                const size_t pi = tree.primitive_indices[i];
                vertices_.push_back(vertices[3 * pi]);
                vertices_.push_back(vertices[3 * pi + 1]);
                vertices_.push_back(vertices[3 * pi + 2]);

                normals_.push_back(normals[3 * pi]);
                normals_.push_back(normals[3 * pi + 1]);
                normals_.push_back(normals[3 * pi + 2]);
            }
            const size_t primEnd = vertices_.size() / 3;

            assert(primEnd != primBeg);
            nodes_[ni].childIndex = static_cast<uint32_t>(primBeg);
            nodes_[ni].childCount = static_cast<uint32_t>(primEnd - primBeg);
        }
        else
        {
            nodes_[ni].childIndex = node.first_child_or_primitive;
            nodes_[ni].childCount = 0;
        }
    }

    triangleCount_ = static_cast<int>(triangleCount);
}

std::span<const BVH::Node> BVH::getNodes() const
{
    return nodes_;
}

std::span<const Float3> BVH::getVertices() const
{
    return vertices_;
}

std::span<const Float3> BVH::getNormals() const
{
    return normals_;
}

float BVH::estimateUpperBound(const Float3 &p, int precison) const
{
    const Float3 lower = { nodes_[0].bbox[0], nodes_[0].bbox[1], nodes_[0].bbox[2] };
    const Float3 upper = { nodes_[0].bbox[3], nodes_[0].bbox[4], nodes_[0].bbox[5] };

    float L = 0, R = distance(0.5f * (lower + upper), p)
                   + distance(lower, upper);

    assert(containsTriangle(p, R * R, 0));

    for(int i = 0; i < precison; ++i)
    {
        const float mid = 0.5f * (L + R);
        if(containsTriangle(p, mid * mid, 0))
            R = mid;
        else
            L = mid;
    }

    return R;
}

float BVH::sdf(const Float3 &p, int signRayCount, float upperBound) const
{
    if(upperBound <= 0)
        upperBound = estimateUpperBound(p, 7);

    assert(containsTriangle(p, upperBound * upperBound, 0));
    const auto [triIdx, udf2Val] = udf2(p, upperBound * upperBound, 0);
    assert(triIdx != TRI_IDX_NIL);
    const float udfVal = std::sqrt(udf2Val);

    thread_local static std::default_random_engine rng{std::random_device()()};
    std::uniform_int_distribution<int> dis(0, triangleCount_ - 1);

    int posCnt = 0;
    for(int i = 0; i < signRayCount; ++i)
    {
        const int rndTriIdx = dis(rng);
        const Float3 &a = vertices_[rndTriIdx * 3 + 0];
        const Float3 &b = vertices_[rndTriIdx * 3 + 1];
        const Float3 &c = vertices_[rndTriIdx * 3 + 2];
        const Float3 dst = 1.0f / 3 * (a + b + c);
        const Float3 dir = dst - p;

        const size_t inctTriIdx = traceTriangleIndex(p, dir, FLOAT_INF);
        if(inctTriIdx == TRI_IDX_NIL)
            continue;

        const Float3 na = normals_[inctTriIdx * 3 + 0];
        const Float3 nb = normals_[inctTriIdx * 3 + 1];
        const Float3 nc = normals_[inctTriIdx * 3 + 2];
        if(dot(dir, na + nb + nc) < 0)
            ++posCnt;
        else
            --posCnt;
    }

    if(posCnt > 0)
        return udfVal;
    if(posCnt < 0)
        return -udfVal;

    const Float3 &a = vertices_[triIdx * 3 + 0];
    const Float3 &b = vertices_[triIdx * 3 + 1];
    const Float3 &c = vertices_[triIdx * 3 + 2];
    assert(udf2Triangle(a, b, c, p) == udf2Val);

    const Float3 &na = normals_[triIdx * 3 + 0];
    const Float3 &nb = normals_[triIdx * 3 + 1];
    const Float3 &nc = normals_[triIdx * 3 + 2];
    
    const int ja = dot(p - a, na) >= 0 ? 1 : -1;
    const int jb = dot(p - b, nb) >= 0 ? 1 : -1;
    const int jc = dot(p - c, nc) >= 0 ? 1 : -1;

    return ja + jb + jc > 0 ? udfVal : -udfVal;
}

size_t BVH::traceTriangleIndex(
    const Float3 &a, const Float3 &d, float maxT) const
{
    constexpr int TRAVERSAL_STACK_SIZE = 128;
    thread_local static uint32_t traversalStack[TRAVERSAL_STACK_SIZE];

    Ray r(a, d, 0, maxT);
    const Float3 invDir = { 1 / r.d.x, 1 / r.d.y, 1 / r.d.z };

    if(!intersectRayBox(
        nodes_[0].bbox, r.o, invDir, r.t0, r.t1))
        return TRI_IDX_NIL;

    int top = 0;
    traversalStack[top++] = 0;

    uint32_t finalIndex = 0;
    Float2 finalUV;
    float finalT = std::numeric_limits<float>::infinity();

    while(top)
    {
        const uint32_t taskNodeIdx = traversalStack[--top];
        const Node &node = nodes_[taskNodeIdx];

        if(node.isLeaf())
        {
            for(uint32_t i = node.childIndex;
                i < node.childIndex + node.childCount; ++i)
            {
                const auto *tri = &vertices_[i * 3];
                if(closestIntersectionWithTriangle(
                    r, tri[0], tri[1] - tri[0], tri[2] - tri[0], &finalT, &finalUV))
                {
                    r.t1 = finalT;
                    finalIndex = i;
                }
            }
        }
        else
        {
            assert(top + 2 < TRAVERSAL_STACK_SIZE);

            const bool addLeft = intersectRayBox(
                nodes_[node.childIndex].bbox, r.o, invDir, r.t0, r.t1);
            if(addLeft)
                traversalStack[top++] = node.childIndex;

            const bool addRight = intersectRayBox(
                nodes_[node.childIndex + 1].bbox, r.o, invDir, r.t0, r.t1);
            if(addRight)
                traversalStack[top++] = node.childIndex + 1;
        }
    }

    if(isinf(finalT))
        return TRI_IDX_NIL;

    return finalIndex;
}

bool BVH::containsTriangle(const Float3 &o, float radius2, size_t nodeIndex) const
{
    auto &node = nodes_[nodeIndex];
    if(!intersectSphereBox(node.bbox, o, radius2))
        return false;

    if(node.isLeaf())
    {
        for(uint32_t i = 0, j = 3 * node.childIndex;
            i < node.childCount; ++i, j += 3)
        {
            if(intersectSphereTriangle(
                vertices_[j], vertices_[j + 1], vertices_[j + 2], o, radius2))
                return true;
        }
        return false;
    }

    return containsTriangle(o, radius2, node.childIndex) ||
           containsTriangle(o, radius2, node.childIndex + 1);
}

std::pair<size_t, float> BVH::udf2(
    const Float3 &p, float u2, size_t nodeIndex) const
{
    auto &node = nodes_[nodeIndex];
    if(!intersectSphereBox(node.bbox, p, u2))
        return { TRI_IDX_NIL, std::numeric_limits<float>::infinity() };

    if(node.isLeaf())
    {
        size_t triIdx = TRI_IDX_NIL;
        float udf2 = std::numeric_limits<float>::infinity();

        for(size_t i = 0, j = 3 * node.childIndex;
            i < node.childCount; ++i, j += 3)
        {
            const Float3 &a = vertices_[j];
            const Float3 &b = vertices_[j + 1];
            const Float3 &c = vertices_[j + 2];

            const float udf2New = udf2Triangle(a, b, c, p);
            if(udf2New < udf2)
            {
                triIdx = i + node.childIndex;
                udf2 = udf2New;
            }
        }

        return { triIdx, udf2 };
    }

    auto left = udf2(p, u2, node.childIndex);
    if(left.first == TRI_IDX_NIL)
        return udf2(p, u2, node.childIndex + 1);

    auto right = udf2(p, (std::min)(left.second, u2), node.childIndex + 1);
    return left.second < right.second ? left : right;
}

D3D11_SDF_END
