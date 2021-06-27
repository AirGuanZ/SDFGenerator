#include "cube.h"

VertexBuffer<Float3> createCubeVertexBuffer()
{
    const Float3 a = { 0, 0, 0 };
    const Float3 b = { 1, 0, 0 };
    const Float3 c = { 1, 0, 1 };
    const Float3 d = { 0, 0, 1 };
    const Float3 e = { 0, 1, 0 };
    const Float3 f = { 1, 1, 0 };
    const Float3 g = { 1, 1, 1 };
    const Float3 h = { 0, 1, 1 };

    const Float3 vertexData[] =
    {
        c, f, g, c, b, f,
        a, h, e, a, d, h,
        h, f, e, h, g, f,
        b, c, d, b, d, a,
        d, g, h, d, c, g,
        b, e, f, b, a, e
    };

    VertexBuffer<Float3> result;
    result.initialize(agz::array_size(vertexData), vertexData);

    return result;
}
