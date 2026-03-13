# ============================================================
# PSO Task Allocator | Batch PSO — Phân bổ nhiệm vụ tối ưu toàn cục
# Thuật toán: Single-Objective Batch PSO với Crossover + Structured Mutation
# ============================================================
import random

from config.settings import PICKUP_NODES, CONFIG_DELIVERY_LOCATIONS


class PSO_TaskAllocator:
    """
    Batch PSO Allocator — Phân bổ TẤT CẢ robot IDLE cùng lúc.

    Fitness: F(X) = w1*Total + w2*Max + beta_dup*Dup + gamma_cong*Cong
    - w1*Total  : Tổng quãng đường tất cả robots (hiệu suất)
    - w2*Max    : Quãng đường robot vất vả nhất (cân bằng tải)
    - beta_dup  : Phạt nặng nếu 2 robot cùng nhắm 1 pickup
    - gamma_cong: Phạt tắc nghẽn tại pickup dựa trên trạng thái thực

    Interface:
        __init__(app)           — app = FleetManager instance
        run(idle_robots, now_t) — trả dict {robot_name: scenario}
    """
    def __init__(self, app):
        self.app = app
        self.coop = app.coop_manager

        self.num_particles = 20
        self.iterations = 20
        self.mutation_rate = 0.20

        # Trọng số Congestion / Penalty
        self.beta_dup_pickup = 5000
        self.gamma_congestion = 8
        self.horizon = 8

        # Trọng số Fitness (w1 + w2 = 1.0)
        self.w1 = 0.6  # Ưu tiên tổng quãng đường
        self.w2 = 0.4  # Ưu tiên cân bằng tải (Max distance)

    def build_scenarios(self):
        scenarios = []
        for p in PICKUP_NODES:
            logical_id = self.app.pickup_orders.get(p, 1)
            scenarios.append({'pickup': p, 'logical_id': logical_id})
        return scenarios

    def _min_delivery_dist(self, pickup, logical_id):
        candidates = CONFIG_DELIVERY_LOCATIONS.get(logical_id, [])
        if not candidates:
            return 9999
        sim_start = (pickup[0], pickup[1], 'N')
        return min(self.coop.get_heuristic(sim_start, d) for d in candidates)

    def _congestion_score(self, pickup, now_t, exclude_name=None):
        px, py = pickup
        c = 0
        for name, r in self.app.robots.items():
            if name == exclude_name:
                continue
            if (r.x, r.y) == pickup:
                c += 30
            # Distance-weighted: robot gần pickup gây tắc nhiều hơn
            if r.target_node == pickup and r.state in ["TO_PICKUP", "PICKING"]:
                dist = abs(r.x - px) + abs(r.y - py)
                if dist <= 3:
                    c += 25
                elif dist <= 6:
                    c += 15
                else:
                    c += 5
            if getattr(r, 'current_job_pickup', None) == pickup:
                c += 20
        # Delivery congestion: kiểm tra delivery nodes cùng logical_id
        logical_id = self.app.pickup_orders.get(pickup, 1)
        delivery_locs = CONFIG_DELIVERY_LOCATIONS.get(logical_id, [])
        for d in delivery_locs:
            for name, r in self.app.robots.items():
                if name == exclude_name:
                    continue
                if (r.x, r.y) == d and r.state in ["TO_DELIVERY", "DROPPING"]:
                    c += 10
        # Reservation table tương lai
        for dt in range(1, self.horizon + 1):
            key = (px, py, now_t + dt)
            if key in self.coop.reservations and self.coop.reservations[key] != exclude_name:
                c += 2
        return c

    def _robot_cost(self, robot, scenario, now_t):
        pickup = scenario['pickup']
        logical_id = scenario['logical_id']
        d1 = self.coop.get_heuristic((robot.x, robot.y, robot.heading), pickup)
        if d1 >= 9999:
            return 1e9, 999999  # unreachable
        d2 = self._min_delivery_dist(pickup, logical_id)
        cong = self._congestion_score(pickup, now_t, robot.name)
        return d1 + d2, cong

    def _calculate_fitness(self, assignment, robots, scenarios, now_t):
        """
        Single-Objective Fitness:
        F = w1*Total + w2*Max + beta_dup*Dup + gamma_cong*Cong_sum
        """
        pickups = []
        cong_sum = 0
        total = 0
        max_cost = 0

        for i, s_idx in enumerate(assignment):
            base_cost, cong = self._robot_cost(robots[i], scenarios[s_idx], now_t)
            cong_sum += cong
            total += base_cost
            if base_cost > max_cost:
                max_cost = base_cost
            pickups.append(scenarios[s_idx]['pickup'])

        dup = len(pickups) - len(set(pickups))

        return (self.w1 * total) + (self.w2 * max_cost) + \
               (self.beta_dup_pickup * dup) + (self.gamma_congestion * cong_sum)

    def _repair(self, assign, scenarios_count):
        for i in range(len(assign)):
            if assign[i] < 0: assign[i] = 0
            if assign[i] >= scenarios_count: assign[i] = scenarios_count - 1
        return assign

    # ---------------------------------------------------------
    # MAIN RUN (Single-Objective Batch PSO)
    # ---------------------------------------------------------
    def run(self, idle_robots, now_t):
        scenarios = self.build_scenarios()
        if not idle_robots or not scenarios:
            return {}

        robots = sorted(idle_robots, key=lambda r: r.name)
        k = min(len(robots), len(PICKUP_NODES))
        robots = robots[:k]
        S = len(scenarios)

        # ❺ Fast-path: 1 robot → greedy (không cần PSO)
        if k == 1:
            best_idx = 0
            best_cost = float('inf')
            for i in range(S):
                cost, cong = self._robot_cost(robots[0], scenarios[i], now_t)
                total = cost + self.gamma_congestion * cong
                if total < best_cost:
                    best_cost = total
                    best_idx = i
            return {robots[0].name: scenarios[best_idx]}

        # Nhóm scenarios theo pickup node
        scenarios_by_pickup = {}
        for idx, sc in enumerate(scenarios):
            p = sc['pickup']
            if p not in scenarios_by_pickup:
                scenarios_by_pickup[p] = []
            scenarios_by_pickup[p].append(idx)
        unique_pickups = list(scenarios_by_pickup.keys())

        # Global Best
        gbest_a = None
        gbest_fit = float('inf')
        particles = []

        # 1. Init Population
        for _ in range(self.num_particles):
            if k <= len(unique_pickups):
                chosen = random.sample(unique_pickups, k)
                a = [random.choice(scenarios_by_pickup[p]) for p in chosen]
                random.shuffle(a)
            else:
                a = random.sample(range(S), k) if S >= k else [random.randrange(S) for _ in range(k)]

            fit = self._calculate_fitness(a, robots, scenarios, now_t)
            if fit < gbest_fit:
                gbest_fit = fit
                gbest_a = a[:]

            particles.append({
                'a': a,
                'pbest_a': a[:],
                'pbest_fit': fit,
            })

        # 2. Main Loop
        for _ in range(self.iterations):
            for p in particles:
                a = p['a'][:]
                pbest_a = p['pbest_a']

                # Crossover: kéo về pbest / gbest
                for i in range(k):
                    if random.random() < 0.4:
                        a[i] = pbest_a[i]
                    elif random.random() < 0.4:
                        a[i] = gbest_a[i]

                # Structured Mutation
                if random.random() < self.mutation_rate:
                    mut_type = random.random()
                    if mut_type < 0.4:
                        # RETARGET: đổi delivery cho cùng pickup
                        idx = random.randrange(k)
                        curr_p = scenarios[a[idx]]['pickup']
                        options = scenarios_by_pickup.get(curr_p, [])
                        if len(options) > 1:
                            a[idx] = random.choice(options)
                    elif mut_type < 0.8:
                        # SWAP: hoán đổi task giữa 2 robots
                        i1, i2 = random.randrange(k), random.randrange(k)
                        a[i1], a[i2] = a[i2], a[i1]
                    else:
                        # RANDOM RESET: khám phá ngẫu nhiên
                        a[random.randrange(k)] = random.randrange(S)

                a = self._repair(a, S)
                new_fit = self._calculate_fitness(a, robots, scenarios, now_t)
                p['a'] = a

                # Cập nhật Personal Best
                if new_fit < p['pbest_fit']:
                    p['pbest_fit'] = new_fit
                    p['pbest_a'] = a[:]
                    # Cập nhật Global Best
                    if new_fit < gbest_fit:
                        gbest_fit = new_fit
                        gbest_a = a[:]

        # 3. Trả về giải pháp tốt nhất
        if gbest_a is None:
            return {}
        return {robots[i].name: scenarios[gbest_a[i]] for i in range(k)}
