// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

#include <bitset>

namespace epic {
namespace nls {

/**
 * @return twice the oriented area of the triangle a-b-c. The area is positive if the triangle is in counter-clockwise order
 */
template <class T>
T OrientedArea2(const Eigen::Vector2<T> a, const Eigen::Vector2<T> b, const Eigen::Vector2<T> c)
{
    return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
}

/**
 * Rasterizes a triangle by traversing the bounding box and testing each pixel using an inside-out test.
 * It assumes that (0,0) is the pixel-center of the top left corner. The reason is that triangle rasterization then works directly with integer coordinates.
 * Depth and homogenous barycentric coordinates are interpolated linearly.
 * Points that have depth outside the range (0, 1] are clipped.
 */
template <class T>
void RasterizeTriangleInsideOut(const Eigen::Matrix<T, 3, 3>& vertices,
                                const Eigen::Matrix<T, 4, 3>& barycentricCoordinates,
                                int width,
                                int height,
                                const std::function<void(int x, int y, T depth, const Eigen::Vector3<T>& bc)>& drawFunction)
{
    int minx = std::max<int>(0, int(std::ceil(vertices.row(0).minCoeff())));
    int maxx = std::min<int>(width - 1, int(std::floor(vertices.row(0).maxCoeff())));
    int miny = std::max<int>(0, int(std::ceil(vertices.row(1).minCoeff())));
    int maxy = std::min<int>(height - 1, int(std::floor(vertices.row(1).maxCoeff())));

    const Eigen::Vector2<T> v0 = vertices.col(0).template head<2>();
    const Eigen::Vector2<T> v1 = vertices.col(1).template head<2>();
    const Eigen::Vector2<T> v2 = vertices.col(2).template head<2>();

    const T area = OrientedArea2(v0, v1, v2);
    if (area <= 0) {
        return;
    }
    const T invArea = T(1) / area;

    const Eigen::Vector2<T> edge0 = v2 - v1;
    const Eigen::Vector2<T> edge1 = v0 - v2;
    const Eigen::Vector2<T> edge2 = v1 - v0;

    for (int x = minx; x <= maxx; ++x) {
        for (int y = miny; y <= maxy; ++y) {
            const Eigen::Vector2<T> p(x, y);
            T w0 = OrientedArea2(v1, v2, p);
            T w1 = OrientedArea2(v2, v0, p);
            T w2 = OrientedArea2(v0, v1, p);
            if (w0 >= 0 && w1 >= 0 && w2 >= 0) {

                // check the filling rule
                bool topLeftRuleOk = true;
                // topLeftRuleOk &= (w0 > 0) || (edge0[1] < 0) || (edge0[1] == 0 && edge0[0] < 0);
                // topLeftRuleOk &= (w1 > 0) || (edge1[1] < 0) || (edge1[1] == 0 && edge1[0] < 0);
                // topLeftRuleOk &= (w2 > 0) || (edge2[1] < 0) || (edge2[1] == 0 && edge2[0] < 0);
                topLeftRuleOk &= (w0 == 0 ? ((edge0[1] == 0 && edge0[0] > 0) ||  edge0[1] > 0) : true);
                topLeftRuleOk &= (w1 == 0 ? ((edge1[1] == 0 && edge1[0] < 0) ||  edge1[1] < 0) : true);
                topLeftRuleOk &= (w2 == 0 ? ((edge2[1] == 0 && edge2[0] < 0) ||  edge2[1] < 0) : true);

                if (topLeftRuleOk) {
                    w0 *= invArea;
                    w1 *= invArea;
                    w2 *= invArea;

                    const T depth = w0 * vertices(2, 0) + w1 * vertices(2, 1) + w2 * vertices(2, 2);
                    if (depth > 0 && depth <= 1) {
                        // only output valid depth values
                        const Eigen::Vector<T,4> bc = w0 * barycentricCoordinates.col(0) +
                                                    w1 * barycentricCoordinates.col(1) +
                                                    w2 * barycentricCoordinates.col(2);
                        drawFunction(x, y, depth, bc.template head<3>() / bc[3]);
                    }
                }
            }
        }
    }
}


/**
 * Rasterizes a triangle, but culls triangles that fall outside the view volume.
 * The view volume is defined by
 * x: [-1, 1]
 * y: [-1, 1]
 * z: [1, 0]  // reverse with 0 being the furthest depth
 *
 * @param vertices      The homogeneous vertex matrix (vertices are the columns of the matrix)
 * @param width         The width of the output image
 * @param height        The height of the output image
 * @param drawFunction  For every rasterized pixel the drawFunction() is called with the pixel position (origin is center of top left pixel),
 *                      the depth of the vertex [1, 0] and the perspectively correct interpolated barycentric coordinates.
 */
template <class T>
void cullAndRasterizeTriangle(const Eigen::Matrix<T, 4, 3>& vertices,
                              const int width,
                              const int height,
                              const std::function<void(int x, int y, T depth, const Eigen::Vector3<T>& bc)>& drawFunction)
{
    const T w0 = vertices(3, 0);
    const T w1 = vertices(3, 1);
    const T w2 = vertices(3, 2);

    // we discard any triangles where the w is negative i.e. any vertex is behind the viewer
    // note that a proper rendering engine would do proper clipping with external line segments,
    // but we do not care for this in our application
    if (w0 <= 0 || w1 <= 0 || w2 <= 0) {
        return;
    }

    // we perform a simple backface culling test here
    const T area = OrientedArea2(Eigen::Vector2<T>(vertices.col(0).template head<2>()),
                                 Eigen::Vector2<T>(vertices.col(1).template head<2>()),
                                 Eigen::Vector2<T>(vertices.col(2).template head<2>()));
    if (area <= 0) {
        return;
    }

    const T x0 = vertices(0, 0);
    const T y0 = vertices(1, 0);
    const T z0 = vertices(2, 0);
    const T x1 = vertices(0, 1);
    const T y1 = vertices(1, 1);
    const T z1 = vertices(2, 1);
    const T x2 = vertices(0, 2);
    const T y2 = vertices(1, 2);
    const T z2 = vertices(2, 2);

    std::bitset<6> clipMask0, clipMask1, clipMask2;
    clipMask0.set(0, x0 < -w0);
    clipMask0.set(1, x0 >  w0);
    clipMask0.set(2, y0 < -w0);
    clipMask0.set(3, y0 >  w0);
    clipMask0.set(4, z0 <   0);
    clipMask0.set(5, z0 >  w0);

    clipMask1.set(0, x1 < -w1);
    clipMask1.set(1, x1 >  w1);
    clipMask1.set(2, y1 < -w1);
    clipMask1.set(3, y1 >  w1);
    clipMask1.set(4, z1 <   0);
    clipMask1.set(5, z1 >  w1);

    clipMask2.set(0, x2 < -w2);
    clipMask2.set(1, x2 >  w2);
    clipMask2.set(2, y2 < -w2);
    clipMask2.set(3, y2 >  w2);
    clipMask2.set(4, z2 <   0);
    clipMask2.set(5, z2 >  w2);

    if ((clipMask0 & clipMask1 & clipMask2).any()) {
        // the triangle is outside any of the clipping planes
        return;
    } else {
        // Make barycentric coordinates divided by the w component to have perspective correct interpolation of the barycentric coordinates.
        // These can then be used to properly interpolate the vertex attributes in the draw function
        Eigen::Matrix<T, 4, 3> bcs = Eigen::Matrix<T, 4, 3>::Zero();
        bcs(0,0) = T(1) / w0;
        bcs(3,0) = T(1) / w0;
        bcs(1,1) = T(1) / w1;
        bcs(3,1) = T(1) / w1;
        bcs(2,2) = T(1) / w2;
        bcs(3,2) = T(1) / w2;

        // convert vertices to window pixel coordinates with the origin being at the center of the top left corner
        // [-0.5, width-0.5], [-0.5, height-0.5], depth = [0, 1]
        const T scale[3] = { T(width) / T(2), T(height) / T(2), T(1) };
        const T offset[3] = { T(width) / T(2) - T(0.5), T(height) / T(2) - T(0.5), T(0) };

        // divide by w and map to windows coordinates
        Eigen::Matrix<T, 3, 3> projectedVertices;
        for (int i = 0; i < 3; i++) {
            for (int k = 0; k < 3; k++) {
                projectedVertices(k,i) = scale[k] * vertices(k,i) / vertices(3, i) + offset[k];
            }
        }

        RasterizeTriangleInsideOut<T>(projectedVertices, bcs, width, height, drawFunction);
    }
}


} // namespace nls
} //namespace epic
