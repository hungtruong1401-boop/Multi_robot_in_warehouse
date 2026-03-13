# ============================================================
# PSO Basic Task Allocator | PSO cơ bản (Manhattan distance)
# Dùng cho tổ hợp: A* + PSO (không dùng BFS heuristic của CA*)
# ============================================================
import random

from config.settings import PICKUP_NODES, CONFIG_DELIVERY_LOCATIONS


def _manhattan(a, b):
    """Khoảng cách Manhattan giữa 2 điểm."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class PSO_BasicAllocator:
    """PSO phân bổ nhiệm vụ cơ bản — dùng Manhattan distance thay vì
    BFS heuristic từ CooperativePlanner. Cùng interface với PSO_TaskAllocator."""

    def __init__(self, current_pos, current_heading, app_instance, caller_name=None):
        self.app = app_instance
        self.start_pos = current_pos  # (x, y, heading)
        self.caller_name = caller_name
        self.pickup_options = PICKUP_NODES
        self.scenarios = []
        for p_node in self.pickup_options:
            logical_id = self.app.pickup_orders.get(p_node, 1)
            self.scenarios.append({'pickup': p_node, 'logical_id': logical_id})
        self.num_particles = 15
        self.iterations = 5
        self.w = 0.5
        self.c1 = 1.5
        self.c2 = 1.5
        self.min_val = 0
        self.max_val = len(self.pickup_options) - 0.01
        self.particles = []
        for _ in range(self.num_particles):
            pos = random.uniform(self.min_val, self.max_val)
            self.particles.append({
                'pos': pos, 'vel': random.uniform(-1, 1),
                'best_pos': pos, 'best_cost': float('inf'),
                'current_cost': float('inf')
            })
        self.global_best_pos = self.particles[0]['pos']
        self.global_best_cost = float('inf')
        self.cost_cache = {}

    def calculate_fitness(self, float_index):
        idx = int(float_index)
        if idx < 0: idx = 0
        if idx >= len(self.pickup_options): idx = len(self.pickup_options) - 1
        if idx in self.cost_cache: return self.cost_cache[idx]
        scenario = self.scenarios[idx]
        pickup = scenario['pickup']
        l_id = scenario['logical_id']
        occupied_penalty = 0
        for rname, r in self.app.robots.items():
            if rname == self.caller_name:
                continue
            if (r.x, r.y) == pickup:
                occupied_penalty = 99999
                break
            if r.target_node == pickup and r.state in ["TO_PICKUP", "PICKING"]:
                occupied_penalty = 99999
                break
        # === KHÁC BIỆT: Dùng Manhattan distance thay vì BFS heuristic ===
        robot_xy = (self.start_pos[0], self.start_pos[1])
        path1_len = _manhattan(robot_xy, pickup)
        candidates = CONFIG_DELIVERY_LOCATIONS[l_id]
        min_path2 = 99999
        for delivery_phys in candidates:
            p_len = _manhattan(pickup, delivery_phys)
            if p_len < min_path2:
                min_path2 = p_len
        total_cost = path1_len + min_path2 + occupied_penalty
        self.cost_cache[idx] = total_cost
        return total_cost

    def run(self):
        for i in range(self.iterations):
            for p in self.particles:
                cost = self.calculate_fitness(p['pos'])
                p['current_cost'] = cost
                if cost < p['best_cost']:
                    p['best_cost'] = cost
                    p['best_pos'] = p['pos']
                if cost < self.global_best_cost:
                    self.global_best_cost = cost
                    self.global_best_pos = p['pos']
            for p in self.particles:
                r1 = random.random()
                r2 = random.random()
                vel_cognitive = self.c1 * r1 * (p['best_pos'] - p['pos'])
                vel_social = self.c2 * r2 * (self.global_best_pos - p['pos'])
                p['vel'] = self.w * p['vel'] + vel_cognitive + vel_social
                p['pos'] += p['vel']
                if p['pos'] < self.min_val: p['pos'] = self.min_val
                if p['pos'] > self.max_val: p['pos'] = self.max_val
        best_idx = int(self.global_best_pos)
        if best_idx < 0: best_idx = 0
        if best_idx >= len(self.pickup_options): best_idx = len(self.pickup_options) - 1
        return self.scenarios[best_idx]
