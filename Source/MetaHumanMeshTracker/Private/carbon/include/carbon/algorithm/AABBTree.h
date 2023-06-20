// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

// from Moa repo: https://github.ol.epicgames.net/Research/Moa/blob/master/extensions/pykdtree/include/aabb.h

#include <carbon/common/Common.h>
CARBON_DISABLE_EIGEN_WARNINGS
#include <Eigen/Dense>
CARBON_RENABLE_WARNINGS

#include <array>
#include <cstdint>
#include <numeric>


namespace epic::carbon {

    // kd-tree class supporting compatibility structures
    template <typename scalar_t, size_t leafSize = 8>
    class AABBTree
    {
    public:
        using point_data_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 3, Eigen::RowMajor>;
        using primitive_data_t = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
        using point_t = Eigen::RowVector<scalar_t, 3>;
        using point2_t = Eigen::RowVector<scalar_t, 2>;
        using primitive_t = Eigen::RowVector<int, 3>;
        using leaf_data_t = std::array<point_t, leafSize>;
        using bbox_t = std::pair<point_t, point_t>;
        static constexpr size_t invalid = std::numeric_limits<size_t>::max();

    private:
        struct node_t {
            std::pair<size_t, size_t> children = { invalid, invalid };
            std::pair<size_t, size_t> range;
            bbox_t bbox;
            size_t dim = invalid;
            scalar_t split = 0.0f;

            node_t(size_t start, size_t end)
                : range({ start, end })
            {
            }
        };

        struct helper_data_t
        {
            point_t edge0;
            point_t edge1;
            point_t normal;
            point_t centroid;
            point_t a;
        };

        // members for storing the data
         const Eigen::Ref<const point_data_t> points;
         const Eigen::Ref<const primitive_data_t> primitives;

        std::vector<helper_data_t> meta;

        // tree structure data
        std::vector<int> indices;
        std::vector<node_t> nodes;

    public:
        AABBTree(const Eigen::Ref<const point_data_t>& points_,
            const Eigen::Ref<const primitive_data_t>& primitives_) :
            points(points_),
            primitives(primitives_)
        {
            CARBON_PRECONDITION(points.rows() > 0, "Point count is zero.");
            CARBON_PRECONDITION(primitives.rows() > 0, "Triangle count is zero.");

            // create indices
            const auto numPrimitives = primitives.rows();
            indices.resize(numPrimitives);
            std::iota(indices.begin(), indices.end(), 0);

            // create centroids for splitting
            meta.resize(numPrimitives);
            for (auto i = 0; i < numPrimitives; i++)
            {
                const auto f = primitives.row(indices[i]);
                meta[i].edge0 = points.row(f[1]) - points.row(f[0]);
                meta[i].edge1 = points.row(f[2]) - points.row(f[0]);
                meta[i].normal = meta[i].edge0.cross(meta[i].edge1);
                meta[i].centroid = (points.row(f[0]) + points.row(f[1]) + points.row(f[2])) * (scalar_t(1.0 / 3.0));
                meta[i].a = point_t(meta[i].edge0.dot(meta[i].edge0),
                    meta[i].edge0.dot(meta[i].edge1),
                    meta[i].edge1.dot(meta[i].edge1));
            }

            // create root node
            nodes.reserve(numPrimitives * 4);
            nodes.emplace_back(0, indices.size());

            // build tree by splitting the root
            splitNode(0);
        }

        std::tuple<int64_t, point_t, scalar_t> getClosestPoint(const Eigen::Ref<const point_t>& point,
            const scalar_t& distance) const
        {
            // find closest compatible point
            int64_t bestIndex = -1;
            point_t bestBarycentric;
            scalar_t bestDistance = distance * distance;

            // use a stack instead of recursion to save time on function calls
            std::array<size_t, 200> stack;
            stack[0] = 0;
            int stackIndex = 0;

            while (stackIndex >= 0)
            {
                // get current node from stack
                const node_t& node = nodes[stack[stackIndex]];
                stackIndex--;

                // check distance from bounding box
                scalar_t bbox_dist = 0.0;
                for (size_t d = 0; d < 3; d++)
                {
                    if (point[d] < node.bbox.first[d])
                        bbox_dist += (point[d] - node.bbox.first[d]) * (point[d] - node.bbox.first[d]);
                    if (point[d] > node.bbox.second[d])
                        bbox_dist += (point[d] - node.bbox.second[d]) * (point[d] - node.bbox.second[d]);
                }
                if (bbox_dist > bestDistance)
                    continue;

                // are we at a leaf node?
                if (node.children.first == invalid)
                {
                    const auto numElements = node.range.second - node.range.first;
                    for (size_t i = 0; i < numElements; i++)
                    {
                        const auto index = indices[node.range.first + i];
                        const auto [pointDistance, bary] = getClosestPointToPrimitive(point, index);
                        if (pointDistance < bestDistance)
                        {
                            bestIndex = index;
                            bestBarycentric = bary;
                            bestDistance = pointDistance;
                        }
                    }
                }
                
                else
                {
                    // no, go to the children if reasonable
                    const scalar_t d = point[node.dim] - node.split;
                    if (d < scalar_t(0))
                    {
                        // we're left of the split, so go to the first child first if it's close enough
                        stackIndex++;
                        stack[stackIndex] = node.children.second;
                        stackIndex++;
                        stack[stackIndex] = node.children.first;
                    }
                    else
                    {
                        // we're right of the split, so go to the first child first if it's close enough
                        stackIndex++;
                        stack[stackIndex] = node.children.first;
                        stackIndex++;
                        stack[stackIndex] = node.children.second;
                    }
                }
            }

            return { bestIndex, bestBarycentric, bestDistance };
        }

        std::tuple<int, point_t, scalar_t> intersectRay(const Eigen::Ref<const point_t>& origin,
            const Eigen::Ref<const point_t>& direction) const
        {
            // find closest compatible point
            int bestIndex = -1;
            point_t bestBarycentric;
            scalar_t bestDistance = std::numeric_limits<scalar_t>::max();

            // use a stack instead of recursion to save time on function calls
            std::array<size_t, 200> stack;
            stack[0] = 0;
            int stackIndex = 0;

            // prepare some other data
            point_t invDir = direction.cwiseInverse();

            while (stackIndex >= 0)
            {
                // get current node from stack
                const node_t& node = nodes[stack[stackIndex]];
                stackIndex--;

                // intersect ray with bounding box
                scalar_t tmin;
                scalar_t tmax;
                if (invDir[0] >= scalar_t(0))
                {
                    tmin = (node.bbox.first[0] - origin[0]) * invDir[0];
                    tmax = (node.bbox.second[0] - origin[0]) * invDir[0];
                }
                else
                {
                    tmin = (node.bbox.second[0] - origin[0]) * invDir[0];
                    tmax = (node.bbox.first[0] - origin[0]) * invDir[0];
                }

                scalar_t tymin;
                scalar_t tymax;
                if (invDir[1] >= scalar_t(0))
                {
                    tymin = (node.bbox.first[1] - origin[1]) * invDir[1];
                    tymax = (node.bbox.second[1] - origin[1]) * invDir[1];
                }
                else
                {
                    tymin = (node.bbox.second[1] - origin[1]) * invDir[1];
                    tymax = (node.bbox.first[1] - origin[1]) * invDir[1];
                }

                if (tmin > tymax || tymin > tmax)
                    continue;
                if (tymin > tmin)
                    tmin = tymin;
                if (tymax < tmax)
                    tmax = tymax;

                scalar_t tzmin;
                scalar_t tzmax;
                if (invDir[2] >= scalar_t(0))
                {
                    tzmin = (node.bbox.first[2] - origin[2]) * invDir[2];
                    tzmax = (node.bbox.second[2] - origin[2]) * invDir[2];
                }
                else
                {
                    tzmin = (node.bbox.second[2] - origin[2]) * invDir[2];
                    tzmax = (node.bbox.first[2] - origin[2]) * invDir[2];
                }

                if (tmin > tzmax || tzmin > tmax)
                    continue;
                if (tzmin > tmin)
                    tmin = tzmin;
                if (tzmax < tmax)
                    tmax = tzmax;

                if (tmin > bestDistance || tmax < 0)
                    continue;

                // are we at a leaf node?
                if (node.children.first == invalid)
                {
                    const auto numElements = node.range.second - node.range.first;
                    for (size_t i = 0; i < numElements; i++)
                    {
                        const auto index = indices[node.range.first + i];
                        const auto [hit, bary, distance] = intersectRayPrimitive(origin, direction, index);
                        if (hit && distance >= 0 && distance < bestDistance)
                        {
                            bestIndex = index;
                            bestBarycentric = bary;
                            bestDistance = distance;
                        }
                    }
                }
                else
                {
                    // no, go to the children if reasonable
                    const scalar_t d = direction[node.dim];
                    if (d < scalar_t(0))
                    {
                        // go to the closer bbox first
                        stackIndex++;
                        stack[stackIndex] = node.children.second;
                        stackIndex++;
                        stack[stackIndex] = node.children.first;
                    }
                    else
                    {
                        // go to the closer bbox first
                        stackIndex++;
                        stack[stackIndex] = node.children.first;
                        stackIndex++;
                        stack[stackIndex] = node.children.second;
                    }
                }
            }

            return { bestIndex, bestBarycentric, bestDistance };
        }

        /**
         * Finds the closest intersection of the ray with the AABB tree in both the positive and the negative direction.
         * @return Tuple containing the triangle index (-1 if there is no intersection), the barycentric coordinates of the intersection,
         *         and the absolute distance from the origin to the intersection point.
         */
        std::tuple<int, point_t, scalar_t> intersectRayBidirectional(const Eigen::Ref<const point_t>& origin,
            const Eigen::Ref<const point_t>& direction) const
        {
            // find closest compatible point
            int bestIndex = -1;
            point_t bestBarycentric;
            scalar_t bestDistance = std::numeric_limits<scalar_t>::max();

            // use a stack instead of recursion to save time on function calls
            std::array<size_t, 200> stack;
            stack[0] = 0;
            int stackIndex = 0;

            // prepare some other data
            point_t invDir = direction.cwiseInverse();

            while (stackIndex >= 0)
            {
                // get current node from stack
                const node_t& node = nodes[stack[stackIndex]];
                stackIndex--;

                // intersect ray with bounding box
                scalar_t tmin;
                scalar_t tmax;
                if (invDir[0] >= scalar_t(0))
                {
                    tmin = (node.bbox.first[0] - origin[0]) * invDir[0];
                    tmax = (node.bbox.second[0] - origin[0]) * invDir[0];
                }
                else
                {
                    tmin = (node.bbox.second[0] - origin[0]) * invDir[0];
                    tmax = (node.bbox.first[0] - origin[0]) * invDir[0];
                }

                scalar_t tymin;
                scalar_t tymax;
                if (invDir[1] >= scalar_t(0))
                {
                    tymin = (node.bbox.first[1] - origin[1]) * invDir[1];
                    tymax = (node.bbox.second[1] - origin[1]) * invDir[1];
                }
                else
                {
                    tymin = (node.bbox.second[1] - origin[1]) * invDir[1];
                    tymax = (node.bbox.first[1] - origin[1]) * invDir[1];
                }

                if (tmin > tymax || tymin > tmax)
                    continue;
                if (tymin > tmin)
                    tmin = tymin;
                if (tymax < tmax)
                    tmax = tymax;

                scalar_t tzmin;
                scalar_t tzmax;
                if (invDir[2] >= scalar_t(0))
                {
                    tzmin = (node.bbox.first[2] - origin[2]) * invDir[2];
                    tzmax = (node.bbox.second[2] - origin[2]) * invDir[2];
                }
                else
                {
                    tzmin = (node.bbox.second[2] - origin[2]) * invDir[2];
                    tzmax = (node.bbox.first[2] - origin[2]) * invDir[2];
                }

                if (tmin > tzmax || tzmin > tmax)
                    continue;
                if (tzmin > tmin)
                    tmin = tzmin;
                if (tzmax < tmax)
                    tmax = tzmax;

                if (tmin > bestDistance || tmax < -bestDistance)
                    continue;

                // are we at a leaf node?
                if (node.children.first == invalid)
                {
                    const auto numElements = node.range.second - node.range.first;
                    for (size_t i = 0; i < numElements; i++)
                    {
                        const auto index = indices[node.range.first + i];
                        const auto [hit, bary, distance] = intersectRayPrimitive(origin, direction, index);
                        const auto absDistance = fabs(distance);
                        if (hit && absDistance < bestDistance)
                        {
                            bestIndex = index;
                            bestBarycentric = bary;
                            bestDistance = absDistance;
                        }
                    }
                }
                else
                {
                    // no, go to the children if reasonable
                    const scalar_t d = direction[node.dim];
                    if (d < scalar_t(0))
                    {
                        // go to the closer bbox first
                        stackIndex++;
                        stack[stackIndex] = node.children.second;
                        stackIndex++;
                        stack[stackIndex] = node.children.first;
                    }
                    else
                    {
                        // go to the closer bbox first
                        stackIndex++;
                        stack[stackIndex] = node.children.first;
                        stackIndex++;
                        stack[stackIndex] = node.children.second;
                    }
                }
            }

            return { bestIndex, bestBarycentric, bestDistance };
        }

    private:
        // --- point-primitive distance + helper functions ---
        std::tuple<scalar_t, point_t> getClosestPointToPrimitive(const Eigen::Ref<const point_t>& point,
            const size_t index) const
        {
            const auto f = primitives.row(index);
            const auto& m = meta[index];
            const auto diff = point - points.row(f[0]);
            const scalar_t b0 = -diff.dot(m.edge0);
            const scalar_t b1 = -diff.dot(m.edge1);
            scalar_t f00 = b0;
            scalar_t f10 = b0 + m.a[0];
            scalar_t f01 = b0 + m.a[1];
            point2_t p0;
            point2_t p1;
            point2_t p;
            scalar_t dt1;
            scalar_t h0;
            scalar_t h1;

            if (f00 >= scalar_t(0))
            {
                if (f01 >= scalar_t(0))
                    getMinEdge02(m.a[2], b1, p);
                else
                {
                    p0[0] = scalar_t(0);
                    p0[1] = f00 / (f00 - f01);
                    p1[0] = f01 / (f01 - f10);
                    p1[1] = scalar_t(1) - p1[0];
                    dt1 = p1[1] - p0[1];
                    h0 = dt1 * (m.a[2] * p0[1] + b1);
                    if (h0 >= scalar_t(0))
                        getMinEdge02(m.a[2], b1, p);
                    else
                    {
                        h1 = dt1 * (m.a[1] * p1[0] + m.a[2] * p1[1] + b1);
                        if (h1 <= scalar_t(0))
                            getMinEdge12(m.a[1], m.a[2], b1, f10, f01, p);
                        else
                            getMinInterior(p0, h0, p1, h1, p);
                    }
                }
            }
            else if (f01 <= scalar_t(0))
            {
                if (f10 <= scalar_t(0))
                    getMinEdge12(m.a[1], m.a[2], b1, f10, f01, p);
                else
                {
                    p0[0] = f00 / (f00 - f10);
                    p0[1] = scalar_t(0);
                    p1[0] = f01 / (f01 - f10);
                    p1[1] = scalar_t(1) - p1[0];
                    h0 = p1[1] * (m.a[1] * p0[0] + b1);
                    if (h0 >= scalar_t(0))
                        p = p0;
                    else
                    {
                        h1 = p1[1] * (m.a[1] * p1[0] + m.a[2] * p1[1] + b1);
                        if (h1 <= scalar_t(0))
                            getMinEdge12(m.a[1], m.a[2], b1, f10, f01, p);
                        else
                            getMinInterior(p0, h0, p1, h1, p);
                    }
                }
            }
            else if (f10 <= scalar_t(0))
            {
                p0[0] = scalar_t(0);
                p0[1] = f00 / (f00 - f01);
                p1[0] = f01 / (f01 - f10);
                p1[1] = scalar_t(1) - p1[0];
                dt1 = p1[1] - p0[1];
                h0 = dt1 * (m.a[2] * p0[1] + b1);
                if (h0 >= scalar_t(0))
                    getMinEdge02(m.a[2], b1, p);
                else
                {
                    h1 = dt1 * (m.a[1] * p1[0] + m.a[2] * p1[1] + b1);
                    if (h1 <= scalar_t(0))
                        getMinEdge12(m.a[1], m.a[2], b1, f10, f01, p);
                    else
                        getMinInterior(p0, h0, p1, h1, p);
                }
            }
            else
            {
                p0[0] = f00 / (f00 - f10);
                p0[1] = scalar_t(0);
                p1[0] = scalar_t(0);
                p1[1] = f00 / (f00 - f01);
                h0 = p1[1] * (m.a[1] * p0[0] + b1);
                if (h0 >= scalar_t(0))
                    p = p0;
                else
                {
                    h1 = p1[1] * (m.a[2] * p1[1] + b1);
                    if (h1 <= scalar_t(0))
                        getMinEdge02(m.a[2], b1, p);
                    else
                        getMinInterior(p0, h0, p1, h1, p);
                }
            }

            const point_t bc{ scalar_t(1) - p[0] - p[1], p[0], p[1] };
            point_t closestPoint = points.row(f[0]) + p[0] * m.edge0 + p[1] * m.edge1;
            return { (point - closestPoint).squaredNorm(), bc };
        }

        inline void getMinEdge02(const scalar_t& a11,
            const scalar_t& b1,
            point2_t& p) const
        {
            p[0] = scalar_t(0);
            if (b1 >= scalar_t(0))
            {
                p[1] = scalar_t(0);
            }
            else if (a11 + b1 <= scalar_t(0))
            {
                p[1] = scalar_t(1);
            }
            else
            {
                p[1] = -b1 / a11;
            }
        }

        inline void getMinEdge12(const scalar_t& a01,
            const scalar_t& a11,
            const scalar_t& b1,
            const scalar_t& f10,
            const scalar_t& f01,
            point2_t& p) const
        {
            const scalar_t h0 = a01 + b1 - f10;
            if (h0 >= scalar_t(0))
            {
                p[1] = scalar_t(0);
            }
            else
            {
                scalar_t h1 = a11 + b1 - f01;
                if (h1 <= scalar_t(0))
                {
                    p[1] = scalar_t(1);
                }
                else
                {
                    p[1] = h0 / (h0 - h1);
                }
            }
            p[0] = scalar_t(1) - p[1];
        }

        inline void getMinInterior(const point2_t& p0,
            const scalar_t& h0,
            const point2_t& p1,
            const scalar_t& h1,
            point2_t& p) const
        {
            const scalar_t z = h0 / (h0 - h1);
            p = (scalar_t(1) - z) * p0 + z * p1;
        }

        // --- ray-primitive intersection function ---
        std::tuple<bool, point_t, scalar_t> intersectRayPrimitive(const Eigen::Ref<const point_t>& origin,
            const Eigen::Ref<const point_t>& direction,
            const size_t& index) const
        {
            const auto f = primitives.row(index);
            const auto& m = meta[index];
            const auto v0 = points.row(f[0]);

            const point_t pvec = direction.cross(m.edge1);
            const scalar_t det = m.edge0.dot(pvec);

            if (det > scalar_t(-1e-7) && det < scalar_t(1e-7))
                return { false, point_t::Zero(), scalar_t(0) };
            const scalar_t inv_det = scalar_t(1) / det;

            const point_t tvec = origin - v0;
            const scalar_t u = tvec.dot(pvec) * inv_det;
            if (u < scalar_t(0) || u > scalar_t(1))
                return { false, point_t::Zero(), scalar_t(0) };

            const point_t qvec = tvec.cross(m.edge0);
            const scalar_t v = direction.dot(qvec) * inv_det;
            if (v < scalar_t(0) || u + v > scalar_t(1))
                return { false, point_t::Zero(), scalar_t(0) };

            const scalar_t t = m.edge1.dot(qvec) * inv_det;
            return { true, point_t(scalar_t(1) - u - v, u, v), t };
        }

        // --- hierarchy construction function and helpers ---
        void splitNode(size_t index)
        {
            // calculate local boundingbox
            auto& node = nodes[index];

            // check if we need to split the node
            const auto numElements = node.range.second - node.range.first;
            if (numElements <= leafSize)
            {
                // nope we're down to a single element, update the bounding box to be that of the primitive
                node.bbox = primitiveBoundingBox(node.range.first, node.range.second);
                return;
            }

            // need to split, calculate centroid bounding box for split decisions
            node.bbox = centroidBoundingBox(node.range.first, node.range.second);

            // calculate the most reasonable axis to split
            const auto size = node.bbox.second - node.bbox.first;
            size.maxCoeff(&node.dim);

            // perform split using a partition of the mean point, this is much faster than the true median
            // and should be ok most of the time
            node.split = (node.bbox.first[node.dim] + node.bbox.second[node.dim]) * 0.5f;
            auto it = std::partition(indices.begin() + node.range.first,
                indices.begin() + node.range.second,
                [&](size_t i)
                {
                    return meta[i].centroid[node.dim] < node.split;
                });

            const auto splitIndex = it - indices.begin();

            // calculate sub boundingboxes to get the right split position
            node.children.first = nodes.size();
            nodes.emplace_back(node.range.first, splitIndex);
            splitNode(node.children.first);

            node.children.second = nodes.size();
            nodes.emplace_back(splitIndex, node.range.second);
            splitNode(node.children.second);

            // update the bounding box to be the union of the child bounding boxes, which already contain the primitive bounding boxes
            node.bbox.first = nodes[node.children.first].bbox.first.cwiseMin(nodes[node.children.second].bbox.first);
            node.bbox.second = nodes[node.children.first].bbox.second.cwiseMax(nodes[node.children.second].bbox.second);
        }

        inline bbox_t centroidBoundingBox(const size_t start, const size_t end) const
        {
            point_t pmin = point_t::Constant(std::numeric_limits<scalar_t>::max());
            point_t pmax = point_t::Constant(-std::numeric_limits<scalar_t>::max());;

            // go over all faces that are part of this list
            for (size_t i = start; i < end; i++)
            {
                const auto& p = meta[indices[i]].centroid;
                pmin = pmin.cwiseMin(p);
                pmax = pmax.cwiseMax(p);
            }

            return { pmin, pmax };
        }

        inline bbox_t primitiveBoundingBox(const size_t start, const size_t end) const
        {
            point_t pmin = point_t::Constant(std::numeric_limits<scalar_t>::max());
            point_t pmax = point_t::Constant(-std::numeric_limits<scalar_t>::max());;

            // go over all faces that are part of this list
            for (size_t i = start; i < end; i++)
            {
                const auto f = primitives.row(indices[i]);
                for (size_t d = 0; d < 3; d++)
                {
                    pmin = pmin.cwiseMin(points.row(f[d]));
                    pmax = pmax.cwiseMax(points.row(f[d]));
                }
            }

            return { pmin, pmax };
        }
    };

} // namespace epic::carbon