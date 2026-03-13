# ============================================================
# Hungarian Task Allocator | Phân nhiệm tối ưu (Hungarian / Munkres)
# Dùng cho tổ hợp: A* + Hungarian
# ============================================================
"""
Thuật toán Hungarian (Munkres) giải bài toán phân công tối ưu:
  - Xây ma trận chi phí robot×pickup dựa trên Manhattan distance
  - Xét tất cả robot IDLE cùng lúc để phân công toàn cục tối ưu
  - Tránh pickup đang bị chiếm bởi robot khác (NOT IDLE)
  - Trả kết quả cho robot gọi hàm (cùng interface với PSO_TaskAllocator)
"""
import random

from config.settings import PICKUP_NODES, CONFIG_DELIVERY_LOCATIONS


def _manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _hungarian_solve(cost_matrix):
    """Thuật toán Hungarian (Munkres) cho ma trận chi phí NxM.
    Trả list of (row, col) assignments tối thiểu tổng chi phí.
    Hoạt động với ma trận không vuông (N != M)."""
    import copy

    n = len(cost_matrix)
    m = len(cost_matrix[0]) if n > 0 else 0
    if n == 0 or m == 0:
        return []

    # Pad thành ma trận vuông nếu cần
    size = max(n, m)
    C = []
    for i in range(size):
        row = []
        for j in range(size):
            if i < n and j < m:
                row.append(cost_matrix[i][j])
            else:
                row.append(0)
        C.append(row)

    # Step 1: Row reduction
    for i in range(size):
        min_val = min(C[i])
        for j in range(size):
            C[i][j] -= min_val

    # Step 2: Column reduction
    for j in range(size):
        min_val = min(C[i][j] for i in range(size))
        for i in range(size):
            C[i][j] -= min_val

    # Augmenting path method
    INF = float('inf')
    u = [0] * (size + 1)
    v = [0] * (size + 1)
    p = [0] * (size + 1)
    way = [0] * (size + 1)

    # Build potentials from scratch using cost_matrix
    C_orig = []
    for i in range(size):
        row = []
        for j in range(size):
            if i < n and j < m:
                row.append(cost_matrix[i][j])
            else:
                row.append(0)
        C_orig.append(row)

    u = [0] * (size + 1)
    v = [0] * (size + 1)
    p = [0] * (size + 1)  # p[j] = row assigned to column j

    for i in range(1, size + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (size + 1)
        used = [False] * (size + 1)

        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = -1
            for j in range(1, size + 1):
                if not used[j]:
                    cur = C_orig[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(size + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break

        while j0:
            p[j0] = p[way[j0]]
            j0 = way[j0]

    # Extract assignments
    result = []
    for j in range(1, size + 1):
        if p[j] > 0 and p[j] - 1 < n and j - 1 < m:
            result.append((p[j] - 1, j - 1))
    return result


class HungarianAllocator:
    """Phân nhiệm dùng Hungarian algorithm — tối ưu toàn cục.
    Cùng interface với PSO_TaskAllocator: __init__ + run()."""

    def __init__(self, current_pos, current_heading, app_instance, caller_name=None):
        self.app = app_instance
        self.start_pos = current_pos  # (x, y, heading)
        self.caller_name = caller_name
        self.pickup_options = list(PICKUP_NODES)

    def _is_pickup_occupied(self, pickup):
        """Kiểm tra pickup đang bị robot khác chiếm."""
        for rname, r in self.app.robots.items():
            if rname == self.caller_name:
                continue
            if (r.x, r.y) == pickup:
                return True
            if r.target_node == pickup and r.state in ["TO_PICKUP", "PICKING"]:
                return True
        return False

    def _best_delivery_cost(self, pickup):
        """Chi phí delivery tối thiểu từ pickup đến delivery gần nhất (theo đơn hàng tập trung)."""
        l_id = self.app.pickup_orders.get(pickup, 1)
        candidates = CONFIG_DELIVERY_LOCATIONS.get(l_id, [])
        min_cost = 99999
        for delivery_phys in candidates:
            cost = _manhattan(pickup, delivery_phys)
            if cost < min_cost:
                min_cost = cost
        return min_cost, l_id

    def run(self):
        """Chạy Hungarian assignment.

        Thu thập tất cả robot IDLE → xây cost matrix → giải Hungarian
        → trả allocation cho robot caller."""
        robot_xy = (self.start_pos[0], self.start_pos[1])

        # Thu thập robot idle (bao gồm cả caller)
        idle_robots = []
        for rname, r in self.app.robots.items():
            if r.state == "IDLE":
                idle_robots.append((rname, (r.x, r.y)))

        # Nếu chỉ có caller (hoặc không tìm thấy), fallback 1-robot
        if not idle_robots:
            idle_robots = [(self.caller_name, robot_xy)]

        # Lọc pickup khả dụng (không bị chiếm)
        available_pickups = []
        occupied_pickups = []
        for p in self.pickup_options:
            if self._is_pickup_occupied(p):
                occupied_pickups.append(p)
            else:
                available_pickups.append(p)

        # Nếu tất cả bị chiếm, dùng tất cả pickup
        if not available_pickups:
            available_pickups = list(self.pickup_options)

        # Xây ma trận chi phí: rows=robots, cols=pickups
        n_robots = len(idle_robots)
        n_pickups = len(available_pickups)
        cost_matrix = []
        pickup_delivery_map = {}  # pickup_idx → best logical_id

        for i, (rname, rpos) in enumerate(idle_robots):
            row = []
            for j, pickup in enumerate(available_pickups):
                dist_to_pickup = _manhattan(rpos, pickup)
                delivery_cost, best_lid = self._best_delivery_cost(pickup)
                pickup_delivery_map[j] = best_lid
                total = dist_to_pickup + delivery_cost
                row.append(total)
            cost_matrix.append(row)

        # Giải Hungarian
        assignments = _hungarian_solve(cost_matrix)

        # Tìm assignment cho caller
        caller_idx = None
        for i, (rname, _) in enumerate(idle_robots):
            if rname == self.caller_name:
                caller_idx = i
                break

        if caller_idx is not None:
            for row_idx, col_idx in assignments:
                if row_idx == caller_idx:
                    pickup = available_pickups[col_idx]
                    l_id = pickup_delivery_map[col_idx]
                    return {'pickup': pickup, 'logical_id': l_id}

        # Fallback: chọn best greedy
        best_cost = float('inf')
        best_pickup = available_pickups[0]
        best_lid = self.app.pickup_orders.get(available_pickups[0], 1)
        for j, pickup in enumerate(available_pickups):
            cost = _manhattan(robot_xy, pickup)
            d_cost, l_id = self._best_delivery_cost(pickup)
            total = cost + d_cost
            if total < best_cost:
                best_cost = total
                best_pickup = pickup
                best_lid = l_id
        return {'pickup': best_pickup, 'logical_id': best_lid}
