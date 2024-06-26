#pragma once

#include <fmt/core.h>
#include <fmt/ranges.h>

#include <chrono>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <random>
#include <thread>
#include <tuple>
#include <vector>

#include "../util.h"

bool AStarFindPath(
    const std::pair<int, int>&        start,
    const std::pair<int, int>&        target,
    const std::vector<unsigned char>& map,
    const std::pair<int, int>&        mapSize,
    std::vector<int>&                 outBuffer
);

void debugLogDistanceMap(
    const std::vector<int>&           dist,
    const std::vector<unsigned char>& map,
    const std::pair<int, int>&        mapSize,
    const std::pair<int, int>&        start,
    const std::pair<int, int>&        target
) {
    for (int i = 0; i < mapSize.first * mapSize.second; i++) {
        if (i == start.first + start.second * mapSize.first) {
            fmt::print(" S ");
        } else if (i == target.first + target.second * mapSize.first) {
            fmt::print(" T ");
        } else if (map[i] == 0) {
            fmt::print(" X ");
        } else if (dist[i] == INT_MAX) {
            fmt::print(" _ ");
        } else {
            fmt::print("{:2} ", dist[i]);
        }
        if ((i + 1) % mapSize.first == 0) {
            fmt::print("\n");
        }
    }
}

// Function to find a path using the A* algorithm
bool AStarFindPath(
    const int                         nStartX,
    const int                         nStartY,
    const int                         nTargetX,
    const int                         nTargetY,
    const std::vector<unsigned char>& map,
    const int                         nMapWidth,
    const int                         nMapHeight,
    std::vector<int>&                 outBuffer
) {
    // Lambda to convert 2D coordinates to a 1D index
    auto idx = [nMapWidth](int x, int y) { return x + y * nMapWidth; };

    // Lambda to convert a 1D index to 2D coordinates
    auto idxToXY = [nMapWidth](int idx) -> std::pair<int, int> {
        return std::make_pair(idx % nMapWidth, idx / nMapWidth);
    };

    // Check if the start and target positions are within bounds
    if (nStartX < 0 || nStartX >= nMapWidth || nStartY < 0 ||
        nStartY >= nMapHeight || nTargetX < 0 || nTargetX >= nMapWidth ||
        nTargetY < 0 || nTargetY >= nMapHeight) {
        return false;
    }

    // Check if start position is traversable
    if (!map[idx(nStartX, nStartY)]) {
        fmt::println("Start position is not traversable: {}, {}", nStartX, nStartY);
        std::this_thread::sleep_for(std::chrono::seconds(1000));
        return false;
    }

    // Check if target position is traversable
    if (!map[idx(nTargetX, nTargetY)]) {
        fmt::println("Target position is not traversable: {}, {}", nTargetX, nTargetY);
        std::this_thread::sleep_for(std::chrono::seconds(1000));
        return false;
    }

    // Check if start and target positions are the same
    if (nStartX == nTargetX && nStartY == nTargetY) {
        fmt::println("Start and target positions are the same: {}, {}", nStartX, nStartY);
        std::this_thread::sleep_for(std::chrono::seconds(1000));
        return false;
    }

    // Heuristic function to estimate the distance to the target from a given
    // position
    auto heuristic = [=](int u) -> int {
        auto [x, y] = idxToXY(u);
        return abs(x - nTargetX) + abs(y - nTargetY);
    };

    const int startPos = idx(nStartX, nStartY);  // Start position in 1D index
    const int targetPos =
        idx(nTargetX, nTargetY);           // Target position in 1D index
    const int n = nMapWidth * nMapHeight;  // Total number of cells in the map
    assert(n == map.size());

    int              discovered    = 0;  // Counter for discovered nodes
    int              exploredNodes = 0;  // Counter for explored nodes
    std::vector<int> path(n);            // Vector to store the path

    // Distance vector for each node to the start,
    // initialized to maximum integer value
    std::vector<int> dist(n, INT_MAX);

    // Tuple type for the priority queue with the
    // cost, discovered nodes, and position
    using pq_t = std::tuple<int, int, int>;
    // Priority queue for the A* algorithm, ordered by the cost
    std::priority_queue<pq_t, std::vector<pq_t>, std::greater<pq_t>> pq;

    // Push the start position into the priority queue
    pq.push({0 + heuristic(startPos), 0, startPos});
    dist[startPos] = 0;

    // Main loop of the A* algorithm
    while (!pq.empty()) {
        // Get the position with the lowest cost
        int node = std::get<2>(pq.top());
        pq.pop();
        exploredNodes++;

        // Iterate over the possible moves (right, left, down, up)
        for (int edge : {+1, -1, +nMapWidth, -nMapWidth}) {
            int nextNode = node + edge;  // Calculate the new position
            // Check if the move is valid (not wrapping around the map edges)
            if ((edge == 1 && (nextNode % nMapWidth == 0)) ||
                (edge == -1 && (node % nMapWidth == 0)))
                continue;
            // Check if the new position is within bounds, not visited yet, and
            // is traversable
            if (0 <= nextNode && nextNode < n &&
                dist[nextNode] > dist[node] + 1 && map[nextNode]) {
                path[nextNode] =
                    node;  // Set the predecessor of the new position
                dist[nextNode] =
                    dist[node] + 1;  // Update the distance to the new position
                if (nextNode == targetPos) {
                    // If the target is reached, exit the loop
                    goto end;
                }
                // Push the new position into the priority queue
                pq.push(
                    {dist[nextNode] + heuristic(nextNode), ++discovered,
                     nextNode}
                );
            }
        }
    }

end:
    debugLogDistanceMap(
        dist, map, {nMapWidth, nMapHeight}, {nStartX, nStartY},
        {nTargetX, nTargetY}
    );

    // Check if the target position was reached
    if (dist[targetPos] == INT_MAX) {
        return false;
    }

    outBuffer.resize(dist[targetPos]);  // Resize the output buffer to the path

    // Backtrack from the target to the start to fill the output buffer with the
    // path
    int curr = targetPos;
    for (int i = dist[targetPos] - 1; i >= 0; i--) {
        outBuffer[i] = curr;
        curr         = path[curr];
    }
    return true;
}

bool AStarFindPath(
    const std::pair<int, int>&        start,
    const std::pair<int, int>&        target,
    const std::vector<unsigned char>& map,
    const std::pair<int, int>&        mapSize,
    std::vector<int>&                 outBuffer
) {
    return AStarFindPath(
        start.first, start.second, target.first, target.second, map,
        mapSize.first, mapSize.second, outBuffer
    );
}

void TestSimplePath() {
    std::vector<unsigned char> map = {
        1, 1, 1, 1, 1,  // _
        1, 1, 1, 1, 1,  // _
        1, 1, 1, 1, 1,  // _
        1, 1, 1, 1, 1,  // _
        1, 1, 1, 1, 1   // _
    };

    std::vector<int> path;
    bool             result = AStarFindPath({0, 0}, {4, 4}, map, {5, 5}, path);

    assert(result == true);
    assert(
        path.size() == 8
    );  // The length of the shortest path (Manhattan distance in this case)
    fmt::println("TestSimplePath passed.");
}

void TestPathWithObstacle() {
    std::vector<unsigned char> map = {
        1, 0, 1, 1, 1,  // _
        1, 0, 1, 1, 1,  // _
        1, 0, 1, 0, 1,  // _
        1, 1, 1, 0, 1,  // _
        1, 1, 1, 0, 1   // _
    };

    std::vector<int> path;
    bool             result = AStarFindPath({0, 0}, {4, 4}, map, {5, 5}, path);

    assert(result == true);
    fmt::println("Path with obstacle: {}", path);
    assert(path.size() == 12);
    std::vector<int> expectedPath = {5, 10, 15, 16, 17, 12,
                                     7, 8,  9,  14, 19, 24};
    assert(path == expectedPath);
    fmt::println("TestPathWithObstacle passed.");
}