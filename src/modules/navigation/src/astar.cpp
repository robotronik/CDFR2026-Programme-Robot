#include "navigation/astar.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <queue>

AStar::AStar(double radius)
    : robotRadius(ROBOT_WIDTH / 2.0) {
    session = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    masks[0].obstacles = {{0, -1500, 2000, 50}, {0, 1500, 2000, 50}, {-1000, 0, 50, 3000}, {1000, 0, 50, 3000}};
    if (std::isfinite(radius) && radius > 0) robotRadius = radius;
    for (Mask& mask : masks)
        rebuildMask(mask);
    waypoints.reserve(CELL_COUNT + 2);
}

bool AStar::setRobotRadius(double radius) {
    if (!std::isfinite(radius) || radius <= 0) return false;
    std::lock_guard<std::mutex> lock(mutex);
    if (radius == robotRadius) return true;
    robotRadius = radius;
    for (Mask& mask : masks)
        rebuildMask(mask);
    return true;
}

bool AStar::setObstacles(Layer layer, std::vector<Obstacle> obstacles) {
    const size_t index = static_cast<size_t>(layer);
    if (index == 0 || index >= masks.size()) return false;
    for (const Obstacle& obstacle : obstacles) {
        if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
            !std::isfinite(obstacle.angle) || !std::isfinite(obstacle.radius) ||
            !std::isfinite(obstacle.width) || !std::isfinite(obstacle.height) ||
            obstacle.radius < 0 || (obstacle.radius == 0 && (obstacle.width <= 0 || obstacle.height <= 0)))
            return false;
    }
    std::lock_guard<std::mutex> lock(mutex);
    Mask& mask = masks[index];
    if (mask.obstacles == obstacles) return true;
    mask.obstacles = std::move(obstacles);
    rebuildMask(mask);
    return true;
}

double AStar::inflation(const Mask& mask) const {
    // Cover the whole cell so visibility checks also protect segments between cell centres.
    return std::max(0.0, robotRadius + SCALE * std::sqrt(2.0) / 2 +
                             (&mask == &masks[1] ? GAME_ELEMENT_MARGIN_MM : 0) -
                             (&mask == &masks[0] ? TABLE_BORDER_MARGIN_REDUCTION_MM : 0));
}

void AStar::rebuildMask(Mask& mask) {
    mask.cells.fill(0);
    const double margin = inflation(mask);
    for (const Obstacle& obstacle : mask.obstacles) {
        const double angle = obstacle.angle * std::numbers::pi / 180;
        const double cosine = std::cos(angle), sine = std::sin(angle);
        const double halfWidth = obstacle.width / 2, halfHeight = obstacle.height / 2;
        const double extentX = obstacle.radius > 0 ? obstacle.radius : std::abs(cosine) * halfWidth + std::abs(sine) * halfHeight;
        const double extentY = obstacle.radius > 0 ? obstacle.radius : std::abs(sine) * halfWidth + std::abs(cosine) * halfHeight;
        const double minRow = std::clamp((obstacle.x - extentX - margin + 1000) / SCALE, 0.0, double(ROWS - 1));
        const double maxRow = std::clamp((obstacle.x + extentX + margin + 1000) / SCALE, 0.0, double(ROWS - 1));
        const double minCol = std::clamp((obstacle.y - extentY - margin + 1500) / SCALE, 0.0, double(COLS - 1));
        const double maxCol = std::clamp((obstacle.y + extentY + margin + 1500) / SCALE, 0.0, double(COLS - 1));
        for (int row = int(std::floor(minRow)); row <= int(std::ceil(maxRow)); ++row) {
            for (int col = int(std::floor(minCol)); col <= int(std::ceil(maxCol)); ++col) {
                if (mask.cells[row * COLS + col]) continue;
                const double dx = row * SCALE - 1000 - obstacle.x;
                const double dy = col * SCALE - 1500 - obstacle.y;
                double distanceSquared;
                double limit = margin;
                if (obstacle.radius > 0) {
                    distanceSquared = dx * dx + dy * dy;
                    limit += obstacle.radius;
                } else {
                    const double outsideX = std::max(std::abs(dx * cosine + dy * sine) - halfWidth, 0.0);
                    const double outsideY = std::max(std::abs(-dx * sine + dy * cosine) - halfHeight, 0.0);
                    distanceSquared = outsideX * outsideX + outsideY * outsideY;
                }
                if (distanceSquared <= limit * limit) mask.cells[row * COLS + col] = 1;
            }
        }
    }
    ++mask.version;
    encodeMask(mask);
}

void AStar::encodeMask(Mask& mask) {
    json runs = json::array();
    for (int index = 0; index < CELL_COUNT;) {
        if (!mask.cells[index]) {
            ++index;
            continue;
        }
        const int start = index;
        while (index < CELL_COUNT && mask.cells[index])
            ++index;
        runs.push_back({start, index - start});
    }
    json obstacles = json::array();
    for (const Obstacle& obstacle : mask.obstacles) {
        // Lidar points already travel in /get_global; exporting them twice would dominate the mask payload.
        if (&mask == &masks[static_cast<size_t>(Layer::Opponent)]) break;
        if (obstacle.radius > 0)
            obstacles.push_back({{"x", obstacle.x}, {"y", obstacle.y}, {"radius", obstacle.radius}});
        else
            obstacles.push_back({{"x", obstacle.x}, {"y", obstacle.y}, {"width", obstacle.width}, {"height", obstacle.height}, {"angle", obstacle.angle}});
    }
    mask.encoded = {{"version", mask.version}, {"cells", std::move(runs)}, {"obstacles", std::move(obstacles)}};
}

int AStar::cost(int row, int col) const {
    if (row < 0 || row >= ROWS || col < 0 || col >= COLS) return -1;
    const int index = row * COLS + col;
    return 1 + GAME_ELEMENT_COST * masks[1].cells[index] + OPPONENT_COST * (masks[0].cells[index] + masks[2].cells[index]);
}

int AStar::cellIndex(position_t position) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.a) ||
        position.x < -1000 || position.x > 1000 || position.y < -1500 || position.y > 1500) return -1;
    return int(std::round((position.x + 1000) / SCALE)) * COLS + int(std::round((position.y + 1500) / SCALE));
}

position_t AStar::cellPosition(int index, double angle) {
    return {double(index / COLS * SCALE - 1000), double(index % COLS * SCALE - 1500), angle};
}

bool AStar::pointClear(position_t point) const {
    const int index = cellIndex(point);
    if (index < 0) return false;
    const int row = index / COLS, col = index % COLS;
    const bool edgeX = std::abs((point.x + 1000) / SCALE - (row - 0.5)) < 1e-10;
    const bool edgeY = std::abs((point.y + 1500) / SCALE - (col - 0.5)) < 1e-10;
    return cost(row, col) >= 0 && (!edgeX || cost(row - 1, col) >= 0) && (!edgeY || cost(row, col - 1) >= 0) &&
           (!edgeX || !edgeY || cost(row - 1, col - 1) >= 0);
}

double AStar::lineCost(position_t from, position_t to) const {
    const double infinity = std::numeric_limits<double>::infinity();
    const int first = cellIndex(from), last = cellIndex(to);
    if (first < 0 || last < 0 || !pointClear(from) || !pointClear(to)) return infinity;
    int row = first / COLS, col = first % COLS;
    const int endRow = last / COLS, endCol = last % COLS;
    const double dx = (to.x - from.x) / SCALE, dy = (to.y - from.y) / SCALE;
    const int stepX = dx > 0 ? 1 : -1, stepY = dy > 0 ? 1 : -1;
    const double deltaX = dx == 0 ? infinity : 1 / std::abs(dx);
    const double deltaY = dy == 0 ? infinity : 1 / std::abs(dy);
    const double x = (from.x + 1000) / SCALE, y = (from.y + 1500) / SCALE;
    const bool alongEdgeX = dx == 0 && std::abs(x - (row - 0.5)) < 1e-10;
    const bool alongEdgeY = dy == 0 && std::abs(y - (col - 0.5)) < 1e-10;
    double nextX = dx == 0 ? infinity : (row + stepX * 0.5 - x) / dx;
    double nextY = dy == 0 ? infinity : (col + stepY * 0.5 - y) / dy;
    const double distance = std::hypot(to.x - from.x, to.y - from.y);
    double total = 0, previous = 0;
    while (true) {
        int weight = cost(row, col);
        if (weight < 0 || (alongEdgeX && cost(row - 1, col) < 0) || (alongEdgeY && cost(row, col - 1) < 0)) return infinity;
        if (alongEdgeX) weight = std::max(weight, cost(row - 1, col));
        if (alongEdgeY) weight = std::max(weight, cost(row, col - 1));
        const double next = std::min({nextX, nextY, 1.0});
        total += std::max(0.0, next - previous) * distance * weight;
        previous = next;
        if (row == endRow && col == endCol) return total;
        if (std::abs(nextX - nextY) < 1e-10) {
            if (cost(row + stepX, col) < 0 || cost(row, col + stepY) < 0) return infinity;
            row += stepX;
            col += stepY;
            nextX += deltaX;
            nextY += deltaY;
        } else if (nextX < nextY) {
            row += stepX;
            nextX += deltaX;
        } else {
            col += stepY;
            nextY += deltaY;
        }
    }
}

int AStar::findPath(position_t start, position_t goal, std::span<position_t> path, double& length) {
    length = std::numeric_limits<double>::infinity();
    const int first = cellIndex(start), last = cellIndex(goal);
    if (first < 0 || last < 0 || path.empty()) return 0;
    std::lock_guard<std::mutex> lock(mutex);
    if (!pointClear(start) || !pointClear(goal)) return 0;
    // shortcut: Same-cell moves stay direct; split endpoint states if sub-cell detours are needed.
    if (first == last || lineCost(start, goal) <= std::hypot(goal.x - start.x, goal.y - start.y) + 1e-8) {
        const int count = start.x == goal.x && start.y == goal.y ? 1 : 2;
        if (path.size() < size_t(count)) return 0;
        path[0] = count == 1 ? goal : start;
        path[count - 1] = goal;
        length = std::hypot(goal.x - start.x, goal.y - start.y);
        return count;
    }
    for (Node& node : nodes)
        node = {std::numeric_limits<double>::infinity(), -1, false};
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    const auto heuristic = [&](int index) {
        const int dx = std::abs(index / COLS - last / COLS), dy = std::abs(index % COLS - last % COLS);
        return SCALE * (std::max(dx, dy) + (std::sqrt(2.0) - 1) * std::min(dx, dy));
    };
    nodes[first].g = 0;
    if (cost(first / COLS, first % COLS) == 1) {
        open.push({heuristic(first), first});
    } else {
        // Price the actual straight exit before choosing it, so it remains a single target.
        for (int index = 0; index < CELL_COUNT; ++index) {
            if (index == first) continue;
            const int row = index / COLS, col = index % COLS;
            bool boundary = false;
            if (cost(row, col) == 1) {
                for (int dr = -1; dr <= 1 && !boundary; ++dr)
                    for (int dc = -1; dc <= 1; ++dc)
                        if (cost(row + dr, col + dc) > 1) boundary = true;
            }
            if (!boundary && index != last) continue;
            const double g = lineCost(start, cellPosition(index, goal.a));
            if (!std::isfinite(g)) continue;
            nodes[index].g = g;
            nodes[index].parent = first;
            open.push({g + heuristic(index), index});
        }
    }
    while (!open.empty()) {
        const int current = open.top().second;
        open.pop();
        if (nodes[current].closed) continue;
        nodes[current].closed = true;
        if (current == last) break;
        const int row = current / COLS, col = current % COLS;
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                if ((dr == 0 && dc == 0) || row + dr < 0 || row + dr >= ROWS || col + dc < 0 || col + dc >= COLS) continue;
                const int next = (row + dr) * COLS + col + dc;
                const int weight = cost(row + dr, col + dc);
                if (nodes[next].closed || weight < 0) continue;
                if (dr && dc && (cost(row + dr, col) < 0 || cost(row, col + dc) < 0)) continue;
                const double nextG = nodes[current].g + SCALE * (dr && dc ? std::sqrt(2.0) : 1) * (cost(row, col) + weight) / 2.0;
                if (nextG >= nodes[next].g) continue;
                nodes[next].g = nextG;
                nodes[next].parent = current;
                open.push({nextG + heuristic(next), next});
            }
        }
    }
    if (!nodes[last].closed) return 0;
    waypoints.clear();
    for (int index = last; index != -1; index = nodes[index].parent)
        waypoints.push_back(cellPosition(index, goal.a));
    std::reverse(waypoints.begin(), waypoints.end());
    if (cost(first / COLS, first % COLS) == 1 && (start.x != waypoints.front().x || start.y != waypoints.front().y))
        waypoints.insert(waypoints.begin(), start);
    else
        waypoints.front() = start;
    if (goal.x != waypoints.back().x || goal.y != waypoints.back().y)
        waypoints.push_back(goal);
    else
        waypoints.back() = goal;

    std::vector<double> accumulated(waypoints.size(), 0);
    for (size_t i = 1; i < waypoints.size(); ++i)
        accumulated[i] = accumulated[i - 1] + lineCost(waypoints[i - 1], waypoints[i]);

    size_t count = 0, current = 0;
    path[count++] = waypoints.front();
    while (current + 1 < waypoints.size()) {
        size_t next = current + 1;
        const int index = cellIndex(waypoints[current]);
        if (current != 0 || cost(index / COLS, index % COLS) == 1) {
            for (size_t candidate = waypoints.size() - 1; candidate > current; --candidate) {
                if (lineCost(waypoints[current], waypoints[candidate]) <= accumulated[candidate] - accumulated[current] + 1e-8) {
                    next = candidate;
                    break;
                }
            }
        }
        if (!std::isfinite(lineCost(waypoints[current], waypoints[next]))) return 0;
        if (count == path.size()) return 0;
        path[count++] = waypoints[next];
        if (current == 0 && cost(index / COLS, index % COLS) > 1) path[count - 1].a = start.a;
        current = next;
    }
    if (path[count - 1].a != goal.a) {
        if (count == path.size()) return 0;
        path[count++] = goal;
    }
    length = 0;
    for (size_t i = 1; i < count; ++i)
        length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    return int(count);
}

json AStar::mapJson(const std::string& knownSession, const std::array<uint64_t, 3>& knownVersions) const {
    std::lock_guard<std::mutex> lock(mutex);
    json layers = json::object();
    constexpr std::array<const char*, 3> names = {"border", "game", "opponent"};
    for (size_t i = 0; i < masks.size(); ++i) {
        if (knownSession != session || knownVersions[i] != masks[i].version) layers[names[i]] = masks[i].encoded;
    }
    return {{"session", session}, {"scale", SCALE}, {"rows", ROWS}, {"cols", COLS}, {"origin", {{"x", -1000}, {"y", -1500}}}, {"robot_radius_mm", robotRadius}, {"layers", std::move(layers)}};
}

json AStar::costmapJson() const {
    std::lock_guard<std::mutex> lock(mutex);
    json cells = json::array();
    for (int row = 0; row < ROWS; ++row)
        for (int col = 0; col < COLS; ++col)
            if (cost(row, col) != 1) cells.push_back({{"x", col}, {"y", row}, {"cost", cost(row, col) < 0 ? -1 : cost(row, col) - 1}});
    return cells;
}
