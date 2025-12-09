#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <vector>
#include <algorithm>
#include <numeric>
#include <queue>
#include <glm/vec3.hpp>

struct Photon {
    glm::vec3 throughput;  // power / contribution
    glm::vec3 position;
    glm::vec3 wi;  // incident direction

    // for KdTree Point access
    static constexpr int dim = 3;
    float operator[](int i) const { return position[i]; }

    Photon() {}
    Photon(const glm::vec3& throughput, const glm::vec3& position, const glm::vec3& wi)
        : throughput(throughput), position(position), wi(wi) {}
};

// compute squared distance between points
inline float distance2(const Photon& p1, const glm::vec3& p2) {
    float dist2 = 0;
    for (int i = 0; i < 3; ++i) {
        dist2 += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    }
    return dist2;
}

// k-d tree for k-nearest-neighbor queries
class KdTree {
private:
    struct Node {
        int axis;           // separation axis (x=0, y=1, z=2)
        int idx;            // index of median point
        int leftChildIdx;   // index of left child
        int rightChildIdx;  // index of right child

        Node() : axis(-1), idx(-1), leftChildIdx(-1), rightChildIdx(-1) {}
    };

    std::vector<Node> nodes;  // array of tree nodes
    const Photon* points;     // pointer to array of photons
    int nPoints;              // number of points

    void buildNode(int* indices, int n_points, int depth) {
        if (n_points <= 0) return;

        // choose separation axis
        const int axis = depth % 3;

        // sort indices by coordinate in the separation axis
        std::sort(indices, indices + n_points, [&](const int idx1, const int idx2) {
            return points[idx1][axis] < points[idx2][axis];
        });

        // index of middle element
        const int mid = (n_points - 1) / 2;

        // add node to node array, remember index of current node (parent)
        const int parentIdx = nodes.size();
        Node node;
        node.axis = axis;
        node.idx = indices[mid];
        nodes.push_back(node);

        // add left children
        const int leftChildIdx = nodes.size();
        buildNode(indices, mid, depth + 1);

        // set index of left child on parent node
        if (leftChildIdx == nodes.size()) {
            nodes[parentIdx].leftChildIdx = -1;
        } else {
            nodes[parentIdx].leftChildIdx = leftChildIdx;
        }

        // add right children
        const int rightChildIdx = nodes.size();
        buildNode(indices + mid + 1, n_points - mid - 1, depth + 1);

        // set index of right child on parent node
        if (rightChildIdx == nodes.size()) {
            nodes[parentIdx].rightChildIdx = -1;
        } else {
            nodes[parentIdx].rightChildIdx = rightChildIdx;
        }
    }

    using KNNQueue = std::priority_queue<std::pair<float, int>>;
    
    void searchKNearestNode(int nodeIdx, const glm::vec3& queryPoint, int k,
                            KNNQueue& queue) const {
        if (nodeIdx == -1 || nodeIdx >= nodes.size()) return;

        const Node& node = nodes[nodeIdx];

        // median point
        const Photon& median = points[node.idx];

        // push to queue
        const float dist2 = distance2(median, queryPoint);
        queue.emplace(dist2, node.idx);

        // if size of queue is larger than k, pop queue
        if (queue.size() > k) {
            queue.pop();
        }

        // if query point is lower than median, search left child
        // else, search right child
        const bool isLower = queryPoint[node.axis] < median[node.axis];
        if (isLower) {
            searchKNearestNode(node.leftChildIdx, queryPoint, k, queue);
        } else {
            searchKNearestNode(node.rightChildIdx, queryPoint, k, queue);
        }

        // at leaf node, if queue's largest minimum distance overlaps siblings region, search siblings
        const float dist_to_siblings = median[node.axis] - queryPoint[node.axis];
        if (queue.size() < k || queue.top().first > dist_to_siblings * dist_to_siblings) {
            if (isLower) {
                searchKNearestNode(node.rightChildIdx, queryPoint, k, queue);
            } else {
                searchKNearestNode(node.leftChildIdx, queryPoint, k, queue);
            }
        }
    }

public:
    KdTree() : points(nullptr), nPoints(0) {}

    void setPoints(const Photon* points, int nPoints) {
        this->points = points;
        this->nPoints = nPoints;
    }

    void buildTree() {
        // setup indices of points
        std::vector<int> indices(nPoints);
        std::iota(indices.begin(), indices.end(), 0);

        // build tree recursively
        buildNode(indices.data(), nPoints, 0);
    }

    std::vector<int> searchKNearest(const glm::vec3& queryPoint, int k, float& maxDist2) const {
        KNNQueue queue;
        searchKNearestNode(0, queryPoint, k, queue);

        std::vector<int> ret(queue.size());
        maxDist2 = 0;
        for (int i = 0; i < ret.size(); ++i) {
            const auto& p = queue.top();
            ret[i] = p.second;
            maxDist2 = std::max(maxDist2, p.first);
            queue.pop();
        }

        return ret;
    }
};

#endif
