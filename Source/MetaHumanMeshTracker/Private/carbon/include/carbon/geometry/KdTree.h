// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

// from Moa repo: https://github.ol.epicgames.net/Research/Moa/blob/master/extensions/pykdtree/include/kdtree.h
// minor extensions to search a list of points

#include <carbon/Common.h>
#include <carbon/common/EigenDenseBackwardsCompatible.h>

#include <array>
#include <numeric>

namespace epic::carbon {
    // some basic compatibility structures that might be useful
    // empty compatibility structure
    template<typename scalar_t>
    struct NoCompatibility {
        NoCompatibility() {
        }

        inline bool isCompatible(const size_t) const {
            return true;
        }
    };


    // compatibility function checking if the normal is pointing in a similar direction
    template<typename scalar_t, size_t dimension = 3>
    struct NormalCompatiblityFunction {
        using data_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, dimension, Eigen::RowMajor>;
        using value_t = Eigen::RowVector<scalar_t, dimension>;

        const Eigen::Ref<const data_t> normals;

        NormalCompatiblityFunction(const Eigen::Ref<const data_t>& normals_) : normals(normals_) {
        }

        inline bool isCompatible(const size_t index, const Eigen::Ref<const value_t>& normal, const scalar_t threshold) const {
            return normal.dot(normals.row(index)) > threshold;
        }
    };


    // kd-tree class supporting compatibility structures
    template<typename scalar_t, typename compatibility_t = NoCompatibility<scalar_t>, size_t dimension = 3, size_t leafSize = 32>
    class KdTree {
        public:
            using data_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, dimension, Eigen::RowMajor>;
            using vector_t = Eigen::RowVector<scalar_t, dimension>;
            using leaf_data_t = std::array<vector_t, leafSize>;
            using bbox_t = std::pair<vector_t, vector_t>;
            static constexpr size_t invalid = std::numeric_limits<size_t>::max();

        private:
            struct node_t {
                std::pair<size_t, size_t> children = {invalid, invalid};
                std::pair<size_t, size_t> range;
                bbox_t bbox;
                size_t dim = invalid;
                scalar_t split = 0.0f;
                leaf_data_t data;

                node_t(size_t start, size_t end)
                    : range({start, end}) {
                }
            };


            // members for storing the data
            const Eigen::Ref<const data_t> points;
            const size_t numPoints;
            compatibility_t comp;

            // tree structure data
            std::vector<size_t> indices;
            std::vector<node_t> nodes;

        public:
            template<typename ... Types>
            KdTree(const Eigen::Ref<const data_t>& points_, Types&& ... args) :
                points(points_),
                numPoints(points.rows()),
                comp(args ...) {
                // create indices
                indices.resize(numPoints);
                std::iota(indices.begin(), indices.end(), 0);

                // create root node
                nodes.reserve((numPoints / leafSize) * 4);
                nodes.emplace_back(0, indices.size());

                // build tree by splitting the root
                splitNode(0);
            }

            vector_t getReferencePoint(const size_t index) const {
                return points.row(index);
            }

            template<typename ... Types>
            std::pair<int64_t, scalar_t> getClosestPoint(const Eigen::Ref<const vector_t>& point,
                                                         const scalar_t& distance,
                                                         Types&& ... args) const {
                // find closest compatible point
                int64_t bestIndex = -1;
                scalar_t bestDistance = distance * distance;

                // use a stack instead of recursion to save time on function calls
                std::array<size_t, 200> stack;
                stack[0] = 0;
                int stackIndex = 0;

                while (stackIndex >= 0) {
                    // get current node from stack
                    const node_t& node = nodes[stack[stackIndex]];
                    stackIndex--;

                    // check distance from bounding box
                    scalar_t bbox_dist = 0.0;
                    for (size_t d = 0; d < dimension; d++) {
                        if (point[d] < node.bbox.first[d]) {
                            bbox_dist += (point[d] - node.bbox.first[d]) * (point[d] - node.bbox.first[d]);
                        }
                        if (point[d] > node.bbox.second[d]) {
                            bbox_dist += (point[d] - node.bbox.second[d]) * (point[d] - node.bbox.second[d]);
                        }
                    }
                    if (bbox_dist > bestDistance) {
                        continue;
                    }

                    // are we at a leaf node?
                    if (node.children.first == invalid) {
                        const auto numElements = node.range.second - node.range.first;
                        for (size_t i = 0; i < numElements; i++) {
                            const scalar_t pointDistance = (node.data[i] - point).squaredNorm();
                            if (pointDistance < bestDistance) {
                                const auto index = indices[node.range.first + i];
                                if (comp.isCompatible(index, args ...)) {
                                    bestIndex = index;
                                    bestDistance = pointDistance;
                                }
                            }
                        }
                    } else {
                        // no, go to the children if reasonable
                        const scalar_t d = point[node.dim] - node.split;
                        if (d < scalar_t(0)) {
                            // we're left of the split, so go to the first child first if it's close enough

                            // only need to deal with the second child if we're within the best distance to the right
                            if (d * d < bestDistance) {
                                stackIndex++;
                                stack[stackIndex] = node.children.second;
                            }
                            stackIndex++;
                            stack[stackIndex] = node.children.first;
                        } else {
                            // we're right of the split, so go to the first child first if it's close enough

                            // only need to deal with the second child if we're within the best distance to the right
                            if (d * d < bestDistance) {
                                stackIndex++;
                                stack[stackIndex] = node.children.first;
                            }
                            stackIndex++;
                            stack[stackIndex] = node.children.second;
                        }
                    }
                }

                return {bestIndex, bestDistance};
            }

            template<typename ... Types>
            std::pair<std::vector<int64_t>, std::vector<scalar_t> > getPointsInRadius(const Eigen::Ref<const vector_t>& point,
                                                                                      const scalar_t& distance,
                                                                                      Types&& ... args) const {
                // find all points within the given radius
                std::pair<std::vector<int64_t>, std::vector<scalar_t> > result;
                const scalar_t squaredDistance = distance * distance;

                // use a stack instead of recursion to save time on function calls
                std::array<size_t, 200> stack;
                stack[0] = 0;
                int stackIndex = 0;

                while (stackIndex >= 0) {
                    // get current node from stack
                    const node_t& node = nodes[stack[stackIndex]];
                    stackIndex--;

                    // check distance from bounding box
                    scalar_t bbox_dist = 0.0;
                    for (size_t d = 0; d < dimension; d++) {
                        if (point[d] < node.bbox.first[d]) {
                            bbox_dist += (point[d] - node.bbox.first[d]) * (point[d] - node.bbox.first[d]);
                        }
                        if (point[d] > node.bbox.second[d]) {
                            bbox_dist += (point[d] - node.bbox.second[d]) * (point[d] - node.bbox.second[d]);
                        }
                    }
                    if (bbox_dist > squaredDistance) {
                        continue;
                    }

                    // are we at a leaf node?
                    if (node.children.first == invalid) {
                        const auto numElements = node.range.second - node.range.first;
                        for (size_t i = 0; i < numElements; i++) {
                            const scalar_t pointDistance = (node.data[i] - point).squaredNorm();
                            if (pointDistance < squaredDistance) {
                                const auto index = indices[node.range.first + i];
                                if (comp.isCompatible(i, args ...)) {
                                    result.first.emplace_back(index);
                                    result.second.emplace_back(pointDistance);
                                }
                            }
                        }
                    } else {
                        // no, go to the children if reasonable
                        const scalar_t d = point[node.dim] - node.split;
                        if (d < scalar_t(0)) {
                            // we're left of the split, so go to the first child first if it's close enough

                            // only need to deal with the second child if we're within the best distance to the right
                            if (d * d < squaredDistance) {
                                stackIndex++;
                                stack[stackIndex] = node.children.second;
                            }
                            stackIndex++;
                            stack[stackIndex] = node.children.first;
                        } else {
                            // we're right of the split, so go to the first child first if it's close enough

                            // only need to deal with the second child if we're within the best distance to the right
                            if (d * d < squaredDistance) {
                                stackIndex++;
                                stack[stackIndex] = node.children.first;
                            }
                            stackIndex++;
                            stack[stackIndex] = node.children.second;
                        }
                    }
                }

                return result;
            }

            void Search(const scalar_t* qrs,
                        int numPts,
                        int* resultIndices,
                        scalar_t* distances = nullptr,
                        const scalar_t maxDistance = std::numeric_limits<scalar_t>::max()) const {
                for (int i = 0; i < numPts; i++) {
                    auto [index, distance] = getClosestPoint(Eigen::Map<const Eigen::RowVector3<scalar_t> >(qrs + 3 * i),
                                                             maxDistance);
                    resultIndices[i] = int(index);
                    if (distances) {
                        distances[i] = distance;
                    }
                }
            }

            Eigen::VectorXi Search(const scalar_t* qrs,
                                   int numPts,
                                   const scalar_t maxDistance = std::numeric_limits<scalar_t>::max()) const {
                Eigen::VectorXi resultIndices(numPts);
                Search(qrs, numPts, resultIndices.data(), nullptr, maxDistance);
                return resultIndices;
            }

        private:
            bbox_t boundingBox(const size_t start, const size_t end) const {
                vector_t pmin = vector_t::Constant(std::numeric_limits<scalar_t>::max());
                vector_t pmax = vector_t::Constant(-std::numeric_limits<scalar_t>::max());

                // go over all indices
                for (size_t i = start; i < end; i++) {
                    const auto& p = points.row(indices[i]);
                    pmin = pmin.cwiseMin(p);
                    pmax = pmax.cwiseMax(p);
                }

                return {pmin, pmax};
            }

            void splitNode(size_t index) {
                // calculate local boundingbox
                auto& node = nodes[index];

                // calculate boundingbox
                node.bbox = boundingBox(node.range.first, node.range.second);

                // check if we need to split the node
                const auto numElements = node.range.second - node.range.first;
                if (numElements <= leafSize) {
                    // nope, we got sufficient small number of nodes in here
                    for (size_t i = 0; i < numElements; i++) {
                        node.data[i] = points.row(indices[node.range.first + i]);
                    }
                    return;
                }

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
                return points.row(i)[node.dim] < node.split;
            });

                const auto splitIndex = it - indices.begin();

                // calculate sub boundingboxes to get the right split position
                node.children.first = nodes.size();
                nodes.emplace_back(node.range.first, splitIndex);
                splitNode(node.children.first);

                node.children.second = nodes.size();
                nodes.emplace_back(splitIndex, node.range.second);
                splitNode(node.children.second);
            }
    };
}  // namespace epic::carbon
