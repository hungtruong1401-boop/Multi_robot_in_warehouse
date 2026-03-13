#UET | NCKH 2026 | Mô phỏng kiểm thử thuật toán cho hệ thống Robot kho hàng vận hành tự động
import tkinter as tk
from tkinter import ttk, messagebox
import time
import random
import heapq
import math 
from collections import deque
# ============================================================
# CẤU HÌNH TỌA ĐỘ VÀ BẢN ĐỒ
# ============================================================
PICKUP_NODES= [(0,0), (3,0), (6,0), (9,0), (1,10), (4,10), (7,10), (10,10)]
# CẤU HÌNH 12 addresses giao hàng theo thầy góp ý
CONFIG_DELIVERY_LOCATIONS = {
    1:  [(2,1), (8,4), (2,7)], 
    2:  [(1,2), (9,5), (1,8)], 
    3:  [(3,2), (7,5), (3,8)], 
    4:  [(2,3), (8,6), (2,9)],
    5:  [(5,1), (5,4), (5,7)], 
    6:  [(4,2), (6,5), (4,8)],
    7:  [(6,2), (4,5), (6,8)],  
    8:  [(5,3), (5,6), (5,9)],  
    9:  [(8,1), (2,4), (8,7)],  
    10: [(7,2), (3,5), (7,8)], 
    11: [(9,2), (1,5), (9,8)], 
    12: [(8,3), (2,6), (8,9)]   
}
DELIVERY_NODES = []
for locs in CONFIG_DELIVERY_LOCATIONS.values():
    DELIVERY_NODES.extend(locs)
#Tạm bỏ để thống nhất màu cho 12 địa chỉ 
#DELIVERY_PALETTE = {
 #   1: "#FFB3BA", 2: "#FFDFBA", 3: "#FFFFBA", 4: "#BAFFC9",
 #   5: "#BAE1FF", 6: "#E6B3FF", 7: "#FFB3E6", 8: "#D4F0F0",
 #   9: "#E6E6FA", 10: "#FFC0CB", 11: "#98FB98", 12: "#F0E68C"
#}

REMOVE_NODES = set()
BLUE_NODES = set(PICKUP_NODES)
# Cấu hình các đường không link
SKIP_COLS = {2,5,8}
SKIP_ROWS = {2,5,8}
# ============================================================
#PSO OPTIMIZER(thuật toán của Đăng_chưa update fitness)
# ============================================================
class PSO_TaskAllocator:
    """
    MODIFIED: Single-Objective Batch PSO
    Fitness Function: F(X) = w1*Total + w2*Max + beta_dup*Dup + gamma_cong*Cong
    - w1*Total  : Tổng quãng đường tất cả robots (hiệu suất)
    - w2*Max    : Quãng đường robot vất vả nhất (cân bằng tải / makespan)
    - beta_dup  : Phạt nặng nếu 2 robot cùng nhắm 1 pickup
    - gamma_cong: Phạt tắc nghẽn tại pickup dựa trên trạng thái thực
    """
    def __init__(self, app):
        self.app = app
        self.coop = app.coop_manager

        self.num_particles = 40
        self.iterations = 35
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
            if r.target_node == pickup and r.state in ["TO_PICKUP", "PICKING"]:
                c += 15
            if getattr(r, 'current_job_pickup', None) == pickup:
                c += 20
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

class BatchPSO_SPV_Allocator:
    """
    Batch PSO với Random Key Encoding (Smallest Position Value).
    Particle = vector số thực [0,1], sort để tạo thứ tự pickup cho robots.
    """
    def __init__(self, app):
        self.app = app
        self.coop = app.coop_manager
        
        # Cấu hình PSO
        self.num_particles = 30  
        self.iterations = 50     
        self.w = 0.5    # Quán tính
        self.c1 = 1.5   # Học tập cá nhân
        self.c2 = 1.5   # Học tập bầy đàn
        
        # Giới hạn không gian tìm kiếm (cho Random Key)
        self.min_pos = 0.0
        self.max_pos = 1.0

    def _decode_assignment(self, position_vector, robots, pickups):
        """
        Random Key Encoding (Smallest Position Value):
        1. Tạo cặp (value, original_index) cho mỗi pickup
        2. Sort cặp theo value tăng dần
        3. Robot[i] nhận Pickup có thứ tự thứ i sau sort
        
        Ví dụ:
        position = [0.7, 0.3, 0.9, 0.1]
        → sorted = [(0.1,3), (0.3,1), (0.7,0), (0.9,2)]
        → Robot[0]→Pickup[3], Robot[1]→Pickup[1], Robot[2]→Pickup[0], Robot[3]→Pickup[2]
        """
        k = len(robots)
        m = len(pickups)
        n = min(k, m)  # Số lượng task tối đa

        # 1. Tạo vector mapping: (value, pickup_index)
        mapping = [(position_vector[i], i) for i in range(len(position_vector))]
            
        # 2. Sort theo value (số thực)
        mapping.sort(key=lambda x: x[0])
        
        # 3. Gán task & tính fitness
        assignment = {}
        total_cost = 0
        
        for i in range(n):
            robot = robots[i]
            pickup_idx = mapping[i][1]  # Lấy original index
            
            if pickup_idx >= m: 
                continue

            pickup = pickups[pickup_idx]
            assignment[robot.name] = pickup
            
            # --- Tính Cost ---
            # Cost 1: Robot → Pickup
            d1 = self.coop.get_heuristic((robot.x, robot.y, robot.heading), pickup)
            
            # Cost 2: Pickup → Delivery (ước lượng)
            l_id = self.app.pickup_orders.get(pickup, list(CONFIG_DELIVERY_LOCATIONS.keys())[0])
            cands = CONFIG_DELIVERY_LOCATIONS.get(l_id, [])
            d2 = 9999
            if cands:
                sim_start = (pickup[0], pickup[1], 'N')
                d2 = min(self.coop.get_heuristic(sim_start, d) for d in cands)
            
            total_cost += (d1 + d2)
            
        return assignment, total_cost

    def run(self, idle_robots, pickups):
        """
        Tối ưu assignment bằng Random Key PSO.
        
        Returns: dict {robot_name: pickup_node}
        """
        # 1. Chuẩn bị
        robots = sorted(idle_robots, key=lambda r: r.name)
        dim = len(pickups)  # Số chiều = số pickup
        
        if not robots or not pickups:
            return {}

        # 2. Khởi tạo swarm
        particles = []
        for _ in range(self.num_particles):
            # Vector số thực ngẫu nhiên [0, 1]
            pos = [random.uniform(self.min_pos, self.max_pos) for _ in range(dim)]
            vel = [random.uniform(-0.1, 0.1) for _ in range(dim)]
            
            # Tính fitness ban đầu
            _, cost = self._decode_assignment(pos, robots, pickups)
            
            particles.append({
                'pos': pos,
                'vel': vel,
                'pbest_pos': pos[:],
                'pbest_cost': cost
            })
            
        # Tìm Global Best ban đầu
        gbest_pos = particles[0]['pbest_pos'][:]
        gbest_cost = particles[0]['pbest_cost']
        for p in particles:
            if p['pbest_cost'] < gbest_cost:
                gbest_cost = p['pbest_cost']
                gbest_pos = p['pbest_pos'][:]
                
        # 3. Main Loop
        for _ in range(self.iterations):
            for p in particles:
                # --- Cập nhật Vận tốc & Vị trí ---
                new_pos = []
                new_vel = []
                
                for i in range(dim):
                    r1 = random.random()
                    r2 = random.random()
                    
                    # PSO velocity update
                    v = (self.w * p['vel'][i] + 
                         self.c1 * r1 * (p['pbest_pos'][i] - p['pos'][i]) + 
                         self.c2 * r2 * (gbest_pos[i] - p['pos'][i]))
                    
                    # Giới hạn vận tốc
                    v = max(-0.2, min(0.2, v))
                    
                    # Position update
                    x = p['pos'][i] + v
                    
                    # Boundary handling
                    x = max(self.min_pos, min(self.max_pos, x))
                    
                    new_vel.append(v)
                    new_pos.append(x)
                
                p['pos'] = new_pos
                p['vel'] = new_vel
                
                # --- Đánh giá Fitness ---
                _, current_cost = self._decode_assignment(p['pos'], robots, pickups)
                
                # Update PBest
                if current_cost < p['pbest_cost']:
                    p['pbest_cost'] = current_cost
                    p['pbest_pos'] = p['pos'][:]
                    
                    # Update GBest
                    if current_cost < gbest_cost:
                        gbest_cost = current_cost
                        gbest_pos = p['pos'][:]
                        
        # 4. Decode gbest thành assignment cuối cùng
        final_assignment, _ = self._decode_assignment(gbest_pos, robots, pickups)
        return final_assignment

class ExtendedCentralPSOAllocator:
    """
    MODIFIED: Specific Crowding MOPSO (Algorithm 2 + 3-Objective)
    Replaces the original Single-Objective Extended PSO.
    """
    def __init__(self, app):
        self.app = app
        self.coop = app.coop_manager

        self.num_particles = 40
        self.iterations = 35
        self.mutation_rate = 0.20

        # Trọng số cho Congestion/Penalty (vẫn dùng để tính f3)
        self.alpha_max = 0.9         
        self.beta_dup_pickup = 5000  
        self.gamma_congestion = 8    
        self.horizon = 8             

        self.k_scen_per_pickup = 3   

        # MOPSO Config
        self.archive = []       
        self.archive_size = 20
        self.num_objectives = 3 # Efficiency, Balance, Penalty

    def build_scenarios(self):
        """
        Tạo scenarios dựa trên mapping cố định pickup_orders.
        Mỗi pickup CHỈ có 1 logical_id cố định.
        """
        scenarios = []
        for p in PICKUP_NODES:
            # Lấy logical_id từ mapping cố định
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
        # robot đang ở pickup / đang nhắm tới pickup => tắc
        for name, r in self.app.robots.items():
            if name == exclude_name:
                continue
            if (r.x, r.y) == pickup:
                c += 30
            if r.target_node == pickup and r.state in ["TO_PICKUP", "PICKING"]:
                c += 15
            if r.current_job_pickup == pickup:
                c += 20
        # reservation trong tương lai gần tại pickup
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

        base = d1 + d2
        return base, cong

    def _fitness_vector(self, assignment, robots, scenarios, now_t):
        pickups = []
        cong_sum = 0
        total = 0
        max_cost = 0

        for i, s_idx in enumerate(assignment):
            rb = robots[i]
            sc = scenarios[s_idx]
            base_cost, cong = self._robot_cost(rb, sc, now_t)
            
            cong_sum += cong
            c = base_cost
            total += c
            if c > max_cost: max_cost = c
            pickups.append(sc['pickup'])

        dup = len(pickups) - len(set(pickups))
        
        # 3 Objectives (Minimize all)
        f1 = total
        f2 = max_cost
        f3 = self.beta_dup_pickup * dup + self.gamma_congestion * cong_sum
        
        return (f1, f2, f3)

    def _repair(self, assign, scenarios_count):
        for i in range(len(assign)):
            if assign[i] < 0: assign[i] = 0
            if assign[i] >= scenarios_count: assign[i] = scenarios_count - 1
        return assign

    # ---------------------------------------------------------
    # 2. ALGORITHM 2: CROWDED VALUE CALCULATION
    # ---------------------------------------------------------
    def calculate_crowded_value_alg2(self):
        """
        Thực hiện chính xác Algorithm 2.
        """
        n = len(self.archive)
        if n < 3:
            for p in self.archive: p['c'] = 1.0
            return

        # Normalization
        f_min = [min(p['fit'][i] for p in self.archive) for i in range(self.num_objectives)]
        f_max = [max(p['fit'][i] for p in self.archive) for i in range(self.num_objectives)]
        ranges = [max(1e-9, f_max[i] - f_min[i]) for i in range(self.num_objectives)]

        normalized_fits = []
        for p in self.archive:
            norm_vec = [(p['fit'][i] - f_min[i]) / ranges[i] for i in range(self.num_objectives)]
            normalized_fits.append(norm_vec)

        # Calculate cx
        for i in range(n):
            current_vec = normalized_fits[i]
            distances = []
            for j in range(n):
                if i == j: continue
                # Euclidean distance
                dist = math.sqrt(sum((current_vec[k] - normalized_fits[j][k])**2 for k in range(self.num_objectives)))
                distances.append((dist, j)) 
            
            distances.sort(key=lambda x: x[0])
            
            if len(distances) >= 2:
                idx_a = distances[0][1]
                idx_b = distances[1][1]
                vec_a = normalized_fits[idx_a]
                vec_b = normalized_fits[idx_b]
                diffs = [abs(vec_a[k] - vec_b[k]) for k in range(self.num_objectives)]
                cx = min(diffs)
                self.archive[i]['c'] = cx
            else:
                self.archive[i]['c'] = 0.0

    # ---------------------------------------------------------
    # 3. SELECT LEADER
    # ---------------------------------------------------------
    def select_leader(self):
        if not self.archive: return None
        total_c = sum(p['c'] for p in self.archive)
        if total_c == 0: return random.choice(self.archive)

        r = random.uniform(0, total_c)
        current_sum = 0
        for p in self.archive:
            current_sum += p['c']
            if current_sum >= r:
                return p
        return self.archive[-1]

    def check_dominance(self, fit_a, fit_b):
        better_one = False
        for va, vb in zip(fit_a, fit_b):
            if va > vb: return False
            if va < vb: better_one = True
        return better_one

    def update_archive_generic(self, archive, archive_limit, assignment, fit):
        """Hàm update dùng chung cho cả Global Archive và Private Archive"""
        # 1. Kiểm tra Dominance
        for member in archive:
            if self.check_dominance(member['fit'], fit):
                return archive # Bị dominate bởi cái cũ -> Không thêm

        # 2. Lọc bỏ cái cũ bị cái mới dominate
        new_archive = [m for m in archive if not self.check_dominance(fit, m['fit'])]
        
        # 3. Thêm cái mới
        new_archive.append({
            'a': assignment[:], 
            'fit': fit,
            'c': 0.0 # Sẽ tính lại khi cần dùng
        })
        
        # 4. Tinh lọc (Pruning) nếu tràn bộ nhớ
        # (Lưu ý: Private Archive thường nhỏ, ví dụ size=5 hoặc 10)
        if len(new_archive) > archive_limit:
            # Tính sơ bộ crowded value (đơn giản hóa bằng khoảng cách Manhattan để nhanh)
            # Hoặc gọi hàm calculate_crowded_value_alg2 riêng cho tập này nếu muốn chính xác tuyệt đối
            # Ở đây ta random xóa để tiết kiệm CPU cho Private Set, hoặc xóa cái cũ nhất
            new_archive.pop(0) 
            
        return new_archive

    def update_archive(self, assignment, fit):
        self.archive = self.update_archive_generic(self.archive, self.archive_size, assignment, fit)

    # ---------------------------------------------------------
    # MAIN RUN
    # ---------------------------------------------------------
    def run(self, idle_robots, now_t):
        scenarios = self.build_scenarios()
        if not idle_robots or not scenarios: return {}

        robots = sorted(idle_robots, key=lambda r: r.name)
        k = min(len(robots), len(PICKUP_NODES))
        robots = robots[:k]
        S = len(scenarios)

        # Helper: Map Scenarios by Pickup for quick lookups
        # scenarios_by_pickup[pickup_node] = [list of indices in 'scenarios']
        scenarios_by_pickup = {}
        for idx, sc in enumerate(scenarios):
            p = sc['pickup']
            if p not in scenarios_by_pickup: scenarios_by_pickup[p] = []
            scenarios_by_pickup[p].append(idx)
        
        unique_pickups = list(scenarios_by_pickup.keys())

        # 1. Init Population (Smart Initialization)
        particles = []
        self.archive = [] 
        
        for _ in range(self.num_particles):
            # Try to assign unique pickups if possible
            if k <= len(unique_pickups):
                # Pick k unique pickups
                chosen_pickups = random.sample(unique_pickups, k)
                a = []
                for p in chosen_pickups:
                    # For each pickup, choose a random valid scenario (variant)
                    sc_idx = random.choice(scenarios_by_pickup[p])
                    a.append(sc_idx)
                # Shuffle the assignment vector so different robots get different pickups
                random.shuffle(a)
            else:
                # Fallback if more robots than pickups
                if S >= k: a = random.sample(range(S), k)
                else: a = [random.randrange(S) for _ in range(k)]
            
            fit = self._fitness_vector(a, robots, scenarios, now_t)
            
            # CẤU TRÚC MỚI CỦA PARTICLE
            p_archive = []
            p_archive = self.update_archive_generic(p_archive, 5, a, fit) # Private size = 5
            
            particles.append({
                'a': a, 
                'private_archive': p_archive, 
                'fit': fit
            })
            self.update_archive(a, fit)

        # 2. Main Loop
        for it in range(self.iterations):
            self.calculate_crowded_value_alg2()
            
            if len(self.archive) > self.archive_size:
                self.archive.sort(key=lambda x: x['c'])
                diff = len(self.archive) - self.archive_size
                self.archive = self.archive[diff:]

            for p in particles:
                # A. Chọn Leader từ Global Archive (Social Component)
                global_leader = self.select_leader() 
                gbest_a = global_leader['a'] if global_leader else p['a']

                # B. Chọn PBest từ Private Archive (Cognitive Component)
                # Thay vì 1 pbest cố định, ta chọn ngẫu nhiên 1 cái từ tập Private của chính nó
                if p['private_archive']:
                    personal_leader = random.choice(p['private_archive'])
                    pbest_a = personal_leader['a']
                else:
                    pbest_a = p['a']

                # C. Di chuyển hạt
                a = p['a'][:]
                
                # Crossover (Bias toward leader)
                # For unique pickup constraint, simple crossover is risky, but we rely on mutation to fix
                for i in range(k):
                    r1 = random.random()
                    r2 = random.random()
                    
                    # Crossover lai ghép với cả Personal Best và Global Best
                    if r1 < 0.4: 
                        a[i] = pbest_a[i] # Kéo về kinh nghiệm bản thân
                    elif r2 < 0.4:
                        a[i] = gbest_a[i] # Kéo về kinh nghiệm bầy đàn
                
                # Structured Mutation
                if random.random() < self.mutation_rate:
                    mut_type = random.random()
                    
                    if mut_type < 0.4: 
                        # A. RETARGET (Change delivery for same pickup)
                        # Pick a gene, find its pickup, pick another scenario with same pickup
                        idx = random.randrange(k)
                        curr_sc_idx = a[idx]
                        curr_p = scenarios[curr_sc_idx]['pickup']
                        options = scenarios_by_pickup.get(curr_p, [])
                        if len(options) > 1:
                            a[idx] = random.choice(options)
                            
                    elif mut_type < 0.8:
                        # B. SWAP (Exchange tasks between 2 robots)
                        # Keeps the set of pickups identical, just swaps who does what
                        i1 = random.randrange(k); i2 = random.randrange(k)
                        a[i1], a[i2] = a[i2], a[i1]
                        
                    else:
                        # C. RANDOM RESET (Small exploration)
                        idx = random.randrange(k)
                        a[idx] = random.randrange(S)
                
                # No repair needed for Swap/Retarget usually, but good safety
                a = self._repair(a, S)
                
                # D. Đánh giá & Update Private Archive
                new_fit = self._fitness_vector(a, robots, scenarios, now_t)
                p['a'] = a
                p['fit'] = new_fit

                # Update Private Archive của hạt
                p['private_archive'] = self.update_archive_generic(
                    p['private_archive'], 5, a, new_fit
                )

                # Update Archive
                self.update_archive(a, new_fit)

        # 3. Select best solution
        if not self.archive: return {}
        # Prioritize solution with 0 duplicate penalty first
        valid_sols = [sol for sol in self.archive if sol['fit'][2] < 1000] # f3 is penalty
        if valid_sols:
             best_sol = min(valid_sols, key=lambda x: x['fit'][0]) # Minimal Total Distance
        else:
             best_sol = min(self.archive, key=lambda x: x['fit'][2]) # Minimal Penalty

        out = {}
        for i in range(k):
            out[robots[i].name] = scenarios[best_sol['a'][i]]
        return out
# ============================================================
#QUẢN LÝ GIAO THÔNG
# ============================================================
class TrafficManager:
    def __init__(self, grid_n, obstacles):
        self.grid_n = grid_n
        self.obstacles = obstacles
        #LUẬT GIAO THÔNG
        self.COLS_DOWN = {0, 3, 6, 9}
        self.COLS_UP = {1, 4, 7, 10}
        self.ROWS_RIGHT = {0, 3, 6, 9}
        self.ROWS_LEFT = {1, 4, 7, 10}

    def is_valid_move(self, curr_pos, next_pos):
        cx, cy = curr_pos
        nx, ny = next_pos
        if not (0 <= nx < self.grid_n and 0 <= ny < self.grid_n) or next_pos in self.obstacles:
            return False, 0
        # Xác định hướng di chuyển
        dirs = {(0,1): 'N', (0,-1): 'S', (1,0): 'E', (-1,0): 'W'}
        direction = dirs.get((nx-cx, ny-cy))
        if not direction: return False, 0
        # Kiểm tra cột/hàng bị chặn
        if direction in 'NS' and cx in SKIP_COLS: return False, 0
        if direction in 'EW' and cy in SKIP_ROWS: return False, 0
        # Kiểm tra luật giao thông
        allowed = {'N': cx in self.COLS_UP, 'S': cx in self.COLS_DOWN,
                   'E': cy in self.ROWS_RIGHT, 'W': cy in self.ROWS_LEFT}
        return (True, 1) if allowed[direction] else (False, 0)   
            
    def get_valid_neighbors(self, cx, cy):
        valid = []
        moves = {'N': (0,1), 'S':(0,-1), 'E':(1,0), 'W':(-1,0)}
        for d, (dx, dy) in moves.items():
            nx, ny = cx + dx, cy + dy
            allowed, _ = self.is_valid_move((cx, cy), (nx, ny))
            if allowed:
                valid.append(((nx, ny), d))
        return valid
# ============================================================
# COOPERATIVE A* MANAGER (replace ban A* truoc do)
# ============================================================
class CooperativePlanner:
    def __init__(self, grid_n, traffic_mgr):
        self.grid_n = grid_n
        self.traffic_mgr = traffic_mgr
        # Reservation Table: Key=(x, y, t), Value=RobotName
        self.reservations = {}
        # Cache BFS: Key=(start_tuple, end_tuple), Value=Distance
        # Để tiết kiệm mem, chỉ cache các điểm quan trọng hoặc tính lazy
        self.bfs_cache = {} 
        self.all_nodes = [(x,y) for x in range(grid_n) for y in range(grid_n)]
        # Pre-compute BFS cho toàn map (vì map nhỏ 11x11)
        self.precompute_all_pairs()

    def precompute_all_pairs(self):
        # Tính khoảng cách thực tế từ mọi ô đến mọi ô dựa trên luật giao thông dùng tính heuristic
        for start_node in self.all_nodes:
            self._bfs_from_source(start_node)
            
    def _bfs_from_source(self, start_node):
        q = deque([(start_node, 0)])
        visited = {start_node: 0}
        while q:
            (cx, cy), dist = q.popleft()
            self.bfs_cache[(start_node, (cx, cy))] = dist
            # Lấy hàng xóm hợp lệ theo luật giao thông
            neighbors = self.traffic_mgr.get_valid_neighbors(cx, cy)
            for (nx, ny), _ in neighbors:
                if (nx, ny) not in visited:
                    visited[(nx, ny)] = dist + 1
                    q.append(((nx, ny), dist + 1))
        # Fill các điểm không đến được bằng vô cực
        for target in self.all_nodes:
            if (start_node, target) not in self.bfs_cache:
                self.bfs_cache[(start_node, target)] = 9999

    def get_heuristic(self, start_pose, target_xy):
        # start_pose có thể là (x,y,h) hoặc (x,y)
        sx, sy = start_pose[0], start_pose[1]
        return self.bfs_cache.get(((sx, sy), target_xy), abs(sx - target_xy[0]) + abs(sy - target_xy[1]))

    def clear_reservation(self, robot_name, keep_current_time=None, keep_pos=None):
        # Xóa các đặt chỗ cũ của robot này để replan, giữ lại reservation cho vị trí hiện tại nếu được chỉ định
        keys_to_remove = [k for k, v in self.reservations.items() if v == robot_name]
        for k in keys_to_remove:
            if keep_current_time is not None and keep_pos is not None:
                # Giữ lại reservation cho vị trí hiện tại và các tick kế tiếp (đang di chuyển)
                if k[0] == keep_pos[0] and k[1] == keep_pos[1] and k[2] >= keep_current_time:
                    continue
            del self.reservations[k]

    def reserve_path(self, robot_name, path_states, start_time, start_pos=None):
        #Đặt chỗ cho vị trí xuất phát nếu được cung cấp
        if start_pos:
            sx, sy, sh, st = start_pos
            self.reservations[(sx, sy, st)] = robot_name
        
        #path_states: List of (x, y, heading, t)
        for (x, y, h, t) in path_states:
            self.reservations[(x, y, t)] = robot_name
        
        #Đặt chỗ cho thời gian thao tác (Dừng 1s = 2 ticks)
        if path_states:
            last_x, last_y, _, last_t = path_states[-1]
            STEPS_WAIT = 2 
            for i in range(1, STEPS_WAIT + 1):
                self.reservations[(last_x, last_y, last_t + i)] = robot_name

    def find_path_space_time(self, start_pose, target_xy, current_time, robot_name, max_depth=100):
        sx, sy, sh = start_pose
        
        # Nếu đích đến không thể tiếp cận (do luật giao thông), trả về None ngay
        if self.get_heuristic((sx, sy), target_xy) >= 9999:
            return None, None

        # Priority Queue: (f_score, g_score, x, y, heading, t, path_cmds, path_states)
        # path_states dùng để reserve sau này
        queue = [(0, 0, sx, sy, sh, current_time, [], [])]
        
        # Visited: (x, y, heading, t)-> g_score
        visited = {} 
        DIR = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
        LEFT = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
        RIGHT = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}
        best_path = None
        min_h = float('inf')

        while queue:
            f, g, cx, cy, ch, ct, path, states = heapq.heappop(queue)
            # Giới hạn độ sâu tìm kiếm
            if g > max_depth: 
                continue
            # Check đích
            if (cx, cy) == target_xy:
                #Kiểm tra xem đích có trống không?
                is_safe = True
                for i in range(1, 5):
                    if (cx, cy, ct + i) in self.reservations and self.reservations.get((cx, cy, ct + i)) != robot_name:
                        is_safe = False; break
                if is_safe:
                    return path, states

            state_key = (cx, cy, ch, ct)
            if state_key in visited and visited[state_key] <= g:
                continue
            visited[state_key] = g

            # --- OPTION 1: DI CHUYỂN (MOVE) ---
            dx, dy = DIR[ch]
            nx, ny = cx + dx, cy + dy
            
            # Check luật giao thông cơ bản
            allowed, penalty = self.traffic_mgr.is_valid_move((cx, cy), (nx, ny))
            
            if allowed:
                # [COOPERATIVE CHECK] Check reservation table
                # Kiểm tra 1: Ô đích (nx, ny) phải trống tại t+1
                # Kiểm tra 2: Ô hiện tại (cx, cy) không bị robot khác chiếm tại t+1 (robot khác đang đi vào)
                dest_free = (nx, ny, ct + 1) not in self.reservations or self.reservations.get((nx, ny, ct + 1)) == robot_name
                curr_safe = (cx, cy, ct + 1) not in self.reservations or self.reservations.get((cx, cy, ct + 1)) == robot_name
                if dest_free and curr_safe:
                    new_g = g + penalty
                    h = self.get_heuristic((nx, ny), target_xy)
                    new_f = new_g + h
                    new_cmds = path + ['F']
                    new_states = states + [(nx, ny, ch, ct + 1)]
                    heapq.heappush(queue, (new_f, new_g, nx, ny, ch, ct + 1, new_cmds, new_states))

            # --- OPTION 2: XOAY (TURN) ---
            # Xoay tốn 1 nhịp thời gian
            for turn_cmd, next_h in [('L', LEFT[ch]), ('R', RIGHT[ch])]:
                if (cx, cy, ct + 1) not in self.reservations or self.reservations.get((cx, cy, ct + 1)) == robot_name:
                    new_g = g + 2 # Penalty xoay
                    h = self.get_heuristic((cx, cy), target_xy)
                    new_f = new_g + h
                    new_cmds = path + [turn_cmd]
                    new_states = states + [(cx, cy, next_h, ct + 1)]
                    heapq.heappush(queue, (new_f, new_g, cx, cy, next_h, ct + 1, new_cmds, new_states))
            # --- OPTION 3: ĐỨNG CHỜ (WAIT) ---
            # Nếu phía trước tắc, robot có thể chọn đứng yên
            if (cx, cy, ct + 1) not in self.reservations or self.reservations.get((cx, cy, ct + 1)) == robot_name:
                new_g = g + 1 
                h = self.get_heuristic((cx, cy), target_xy)
                # Phạt nhẹ lệnh WAIT để ưu tiên di chuyển nếu có thể
                new_f = new_g + h + 0.5 
                new_cmds = path + ['WAIT']
                new_states = states + [(cx, cy, ch, ct + 1)]
                heapq.heappush(queue, (new_f, new_g, cx, cy, ch, ct + 1, new_cmds, new_states))
        return None, None
# ============================================================
# Trạng thái Robot | RobotState
# ============================================================
class RobotState:
    def __init__(self, name, color, sub=6):
        self.name = name
        self.color = color
        self.x = 0; self.y = 0; self.heading = 'N'
        self.SUB = sub
        self.sx = self.x * self.SUB
        self.sy = self.y * self.SUB
        self.step_cnt = 0
        self.step_backlog = 0 
        self.state = "IDLE" 
        self.target_node = None
        self.future_delivery_logical_id = None
        self.current_job_pickup = None 
        self.path_queue = []
        self.wait_timer = 0
        self.paused = False
        self.manual_override = False
        self.delivered_count = 0 
        self.total_distance = 0 # Track distance 
        #Theo dõi xem robot có đang theo plan CA* không
        self.has_planned = False
        #check robot kẹt để phát hiện nghẽn
        self.stuck_counter = 0
        self.last_position = None
        self.MAX_STUCK = 100 #chộ này quy định thời gian tắc bao lâu thì replan
        
        # phần thêm vào để xử lí IDLE
        self.idle_since_tick = None

    def sync_sub_from_grid(self):
        self.sx = self.x * self.SUB
        self.sy = self.y * self.SUB
    def reset(self):
        self.x = 0; self.y = 0; self.heading = 'N'
        self.sx = 0; self.sy = 0
        self.step_backlog = 0
        self.path_queue.clear()
        self.state = "IDLE"
        self.target_node = None
        self.future_delivery_logical_id = None
        self.current_job_pickup = None 
        self.wait_timer = 0
        self.paused = False
        self.manual_override = False
        self.delivered_count = 0
        self.has_planned = False
        self.stuck_counter = 0
        self.last_position = None

        # Phần thêm vào xử lí IDLE
        self.idle_since_tick = None
# ============================================================
# CHƯƠNG TRÌNH CHÍNH
# ============================================================
class RobotSimulationApp:

    # --- hàm buffered group cho việc IDLE ---
    def _dispatch_idle_buffered_pso(self):
        if self.USE_EXTENDED_PSO:
            return

        # gom robot IDLE đủ điều kiện
        idle = []
        for rb in self.robots.values():
            if (rb.state == "IDLE" and not rb.path_queue and rb.wait_timer == 0
                and rb.step_backlog == 0 and rb.target_node is None
                and not rb.paused and not rb.manual_override):
                if rb.idle_since_tick is None:
                    rb.idle_since_tick = self.tick_counter
                idle.append(rb)

        if not idle:
            return

        now = self.tick_counter
        max_wait = max(now - (rb.idle_since_tick or now) for rb in idle)

        # điều kiện dispatch:
        #  - đủ >= 2 robot IDLE => dispatch ngay
        #  - hoặc chờ quá IDLE_WAIT_TICKS => dispatch cho số hiện có
        should_dispatch = (len(idle) >= 2) or (max_wait >= self.IDLE_WAIT_TICKS)
        if not should_dispatch:
            return

        # cooldown tránh bắn liên tục mỗi tick
        if (now - self.last_dispatch_tick) < self.DISPATCH_COOLDOWN_TICKS:
            return

        # nếu >8 robot IDLE thì ưu tiên những con chờ lâu nhất
        idle.sort(key=lambda rb: (-(now - (rb.idle_since_tick or now)), rb.name))

        # phân lô robot IDLE để dispatch
        for i in range(0, len(idle), self.BATCH_TARGET):
            group = idle[i:i + self.BATCH_TARGET]

            # chạy PSO_TaskAllocator (class bạn muốn)
            assignment = self.pso_batch_allocator.run(group, now)
            if not assignment:
                continue

            # gán nhiệm vụ + lập kế hoạch CA* cho từng robot trong nhóm
            for rb in group:
                sc = assignment.get(rb.name)
                if not sc:
                    continue

                pickup = sc['pickup']
                logical_id = sc['logical_id']

                rb.target_node = pickup
                rb.future_delivery_logical_id = logical_id
                rb.current_job_pickup = pickup

                # reset timer IDLE vì đã được giao việc
                rb.idle_since_tick = None

                # lập kế hoạch đường đi (CA*) cho robot này
                self.coop_manager.clear_reservation(rb.name, now, (rb.x, rb.y))
                path, states = self.coop_manager.find_path_space_time(
                    (rb.x, rb.y, rb.heading), pickup, now, rb.name
                )

                if path is not None:
                    rb.path_queue = path
                    start_pos = (rb.x, rb.y, rb.heading, now)
                    self.coop_manager.reserve_path(rb.name, states, 0, start_pos)
                    rb.state = "TO_PICKUP"

                    # sinh đơn mới cho pickup vừa được assign (giữ stream đơn liên tục cho lô tiếp theo)
                    if pickup in PICKUP_NODES:
                        self.pickup_orders[pickup] = random.choice(self.logical_ids)
                else:
                    # plan fail => giữ IDLE không đổi để tick sau tính tiếp, thử giữ reservation dài hạn
                    rb.target_node = None
                    rb.future_delivery_logical_id = None
                    rb.current_job_pickup = None
                    rb.state = "IDLE"
                    # Không reset r.idle_since_tick = now mà để nguyên để nó liên tục được xử lý,
                    # HOẶC phạt nhẹ 2 tick để nhường loop
                    rb.idle_since_tick = now - self.IDLE_WAIT_TICKS + 2 
                    for t_offset in range(16):
                        self.coop_manager.reservations[(rb.x, rb.y, now + t_offset)] = rb.name

        self.last_dispatch_tick = now

    def __init__(self, root):

        # --- Cấu hình chiến lược IDLE ---
        self.LOGIC_MS = 50
        # ===== Buffered Batch Dispatch Config =====
        self.BATCH_TARGET = 8              # tối đa 8 robot trong 1 batch PSO
        self.IDLE_WAIT_SEC = 0.5           # hoặc 1.0
        self.IDLE_WAIT_TICKS = max(1, int(self.IDLE_WAIT_SEC * 1000 / self.LOGIC_MS))

        self.last_dispatch_tick = -999
        self.DISPATCH_COOLDOWN_TICKS = 1   # tránh dispatch liên tục mỗi tick

        self.root = root
        self.root.title("Mô phỏng hệ thống Robot kho hàng vận hành tự động | UET NCKH2026")
        self.root.state("zoomed")
        self.SUB = 10 
        self.grid_n = 11
        
        self.traffic_mgr = TrafficManager(self.grid_n, REMOVE_NODES)
        #Khởi tạo Cooperative Planner
        self.coop_manager = CooperativePlanner(self.grid_n, self.traffic_mgr)
        self.is_running = False 
        self.robots = {}
        self.tick_counter = 0 # Thời gian toàn cục cho CA*
        self.is_benchmark_mode = False
        self.start_time = None
        self.TEST_DURATION = 60

        # ====== Order map: mỗi PICKUP có 1 logical_id ổn định tới khi được phục vụ ======
        self.logical_ids = list(CONFIG_DELIVERY_LOCATIONS.keys())
        self.logical_ids = list(CONFIG_DELIVERY_LOCATIONS.keys())
        # self.pickup_orders = {p: random.choice(self.logical_ids) for p in PICKUP_NODES}
        # Fixed assignments as requested
        self.pickup_orders = {
            (0,0): 1,
            (3,0): 2,
            (6,0): 3,
            (9,0): 4,
            (1,10): 5,
            (4,10): 6,
            (7,10): 7,
            (10,10): 8
        }

        # Bật/tắt thuật toán Extended (central)
        self.USE_EXTENDED_PSO = False
        self.ext_allocator = ExtendedCentralPSOAllocator(self)
        self.pso_batch_allocator = PSO_TaskAllocator(self)

        # ================= GUI LAYOUT =================
        self.main_pane = tk.PanedWindow(root, orient=tk.HORIZONTAL, sashwidth=6)
        self.main_pane.pack(fill="both", expand=True)
        # MAP
        self.frame_map = tk.Frame(self.main_pane, bg="white")
        self.main_pane.add(self.frame_map, minsize=600)
        self.cell = 55
        self.MARGIN = 40
        size = (self.grid_n - 1) * self.cell + 2 * self.MARGIN
        self.canvas = tk.Canvas(self.frame_map, width=size, height=size, bg="white")
        self.canvas.pack(padx=10, pady=10)

        # Bảng giám sát điểm nhận hàng 
        self.frame_pickup_info = tk.LabelFrame(self.frame_map, text="Giám sát Trạm Nhận Hàng", bg="white", padx=5, pady=5)
        self.frame_pickup_info.pack(fill="x", padx=10, pady=(0, 10))
        
        pk_cols = ("id", "coord", "status", "delivery_coord", "assigned_robot")
        self.tree_pickup = ttk.Treeview(self.frame_pickup_info, columns=pk_cols, show="headings", height=8)
        self.tree_pickup.heading("id", text="ID Trạm")
        self.tree_pickup.heading("coord", text="Tọa độ")
        self.tree_pickup.heading("status", text="Đơn hàng (Tới Điểm trả)")
        self.tree_pickup.heading("delivery_coord", text="Tọa độ trả")
        self.tree_pickup.heading("assigned_robot", text="Robot thực hiện")
        self.tree_pickup.column("id", width=60, anchor="center")
        self.tree_pickup.column("coord", width=80, anchor="center")
        self.tree_pickup.column("status", width=120, anchor="center")
        self.tree_pickup.column("delivery_coord", width=80, anchor="center")
        self.tree_pickup.column("assigned_robot", width=100, anchor="center")
        self.tree_pickup.pack(fill="both", expand=True)

        # ĐIỀU KHIỂN
        self.frame_right = tk.Frame(self.main_pane)
        self.main_pane.add(self.frame_right, minsize=400)
        tk.Label(self.frame_right, text="Trung tâm điều phối robot | NCKH2026", font=("Arial", 14, "bold"), fg="#333").pack(pady=10)
        self.lbl_timer = tk.Label(self.frame_right, text="Thời gian: 00:00:00", font=("Arial", 12, "bold"), fg="red")
        self.lbl_timer.pack(pady=2)
        self.frame_config = tk.LabelFrame(self.frame_right, text="Cấu hình hệ thống", padx=5, pady=5)
        self.frame_config.pack(fill="x", padx=10, pady=5)
        f_cfg_row1 = tk.Frame(self.frame_config)
        f_cfg_row1.pack(fill="x", pady=2)
        tk.Label(f_cfg_row1, text="Số lượng Robot (1-72): ").pack(side="left")
        self.spin_count = tk.Spinbox(f_cfg_row1, from_=1, to=72, width=5)
        self.spin_count.delete(0, "end")
        self.spin_count.insert(0, 5) 
        self.spin_count.pack(side="left", padx=5)
        tk.Button(f_cfg_row1, text="Tạo Robot", bg="#2196F3", fg="white", command=self.setup_robots_ui).pack(side="left", padx=5)
        f_btn_row = tk.Frame(self.frame_config)
        f_btn_row.pack(fill="x", pady=5)
        self.btn_start = tk.Button(f_btn_row, text="KHỞI ĐỘNG HỆ THỐNG", bg="#4CAF50", fg="white", font=("Arial", 10, "bold"), height=2, command=self.toggle_system_start)
        self.btn_start.pack(side="left", fill="x", expand=True, padx=(0,5))
        self.btn_test_60s = tk.Button(f_btn_row, text="CHẠY TEST (60s)", bg="#FF9800", fg="white", font=("Arial", 10, "bold"), height=2, command=self.start_benchmark_60s)
        self.btn_test_60s.pack(side="left", padx=0)
        self.frame_pos = tk.LabelFrame(self.frame_right, text="Giám sát trạng thái", padx=5, pady=5)
        self.frame_pos.pack(fill="both", expand=True, padx=10, pady=5)
        self.lbl_total_stats = tk.Label(self.frame_pos, text="Tổng hàng đã giao: 0", font=("Arial", 11, "bold"), fg="blue")
        self.lbl_total_stats.pack(anchor="w", padx=5, pady=(0, 5))
        columns = ("name", "status", "pos", "target", "next_zone", "count")
        self.tree_monitor = ttk.Treeview(self.frame_pos, columns=columns, show="headings", height=12)
        self.tree_monitor.heading("name", text="Robot")
        self.tree_monitor.heading("status", text="Trạng thái")
        self.tree_monitor.heading("pos", text="Vị trí")
        self.tree_monitor.heading("target", text="Đích đến")
        self.tree_monitor.heading("next_zone", text="Điểm Trả") 
        self.tree_monitor.heading("count", text="Đã giao")
        self.tree_monitor.column("name", width=50, anchor="center")
        self.tree_monitor.column("status", width=90, anchor="w")
        self.tree_monitor.column("pos", width=60, anchor="center")
        self.tree_monitor.column("target", width=60, anchor="center")
        self.tree_monitor.column("next_zone", width=60, anchor="center")
        self.tree_monitor.column("count", width=50, anchor="center")
        scrollbar = ttk.Scrollbar(self.frame_pos, orient=tk.VERTICAL, command=self.tree_monitor.yview)
        self.tree_monitor.configure(yscroll=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.tree_monitor.pack(fill="both", expand=True)
        self.frame_manual = tk.LabelFrame(self.frame_right, text="Điều khiển thủ công", padx=10, pady=10)
        self.frame_manual.pack(fill="x", padx=10, pady=10)
        frame_sel = tk.Frame(self.frame_manual)
        frame_sel.pack(fill="x", pady=5)
        tk.Label(frame_sel, text="Chọn Robot:").pack(side="left")
        self.combo_robots = ttk.Combobox(frame_sel, values=[], state="readonly", width=10)
        self.combo_robots.pack(side="left", padx=5)
        self.btn_pause = tk.Button(frame_sel, text="Dừng/Tiếp", bg="yellow", command=self.toggle_robot_pause)
        self.btn_pause.pack(side="left", padx=5)
        frame_tgt = tk.Frame(self.frame_manual)
        frame_tgt.pack(fill="x", pady=5)
        tk.Label(frame_tgt, text="Điểm nhận (x,y):").pack(side="left")
        self.ent_x = tk.Entry(frame_tgt, width=5); self.ent_x.pack(side="left", padx=2)
        self.ent_y = tk.Entry(frame_tgt, width=5); self.ent_y.pack(side="left", padx=2)
        tk.Button(frame_tgt, text="Di chuyển", bg="#4CAF50", fg="white", command=self.manual_set_target).pack(side="left", padx=5)
        # ================= LOOP =================
        self.setup_robots(5) 
        self.draw_map()
        self.draw_robot()
        self.ANIM_MS = 10 #này là tốc độ fps của robot, có thể sửa để tăng/giảm hoạt ảnh 
        self.LOGIC_MS = 50 #này là tốc độ xử lý logic của robot, có thể sửa để tăng/giảm tốc độ xử lý 
        self.root.after(self.ANIM_MS, self._anim_tick)
        self.root.after(self.LOGIC_MS, self._server_logic_tick)
        self.root.after(200, self._refresh_status_panel)
    def setup_robots_ui(self):
        try:
            n = int(self.spin_count.get())
            if n < 1: n= 1
            if n > 72: n= 72
        except:
            n= 5
        self.is_running = False
        self.is_benchmark_mode = False
        self.btn_start.config(text="KHỞI ĐỘNG HỆ THỐNG", bg="#4CAF50")
        self.setup_robots(n)
        self.draw_map()
        self.draw_robot()
    def setup_robots(self, count):
        self.robots.clear()
        self.tick_counter = 0
        self.coop_manager.reservations.clear()
        valid_spawns = []
        for x in range(self.grid_n):
            for y in range(self.grid_n):
                if (x, y) not in REMOVE_NODES:
                    if x not in SKIP_COLS or y not in SKIP_ROWS:
                        valid_spawns.append((x, y))
        random.shuffle(valid_spawns)
        # Fixed start positions if count is 8
        fixed_starts = [
            (1, 1), (1, 4), 
            (4, 1), (4, 4),
            (7, 1), (7, 4),
            (10, 1), (10, 4)
        ]

        for i in range(count):
            if count == 8 and i < len(fixed_starts):
                sx, sy = fixed_starts[i]
            else:
                if not valid_spawns: break
                sx, sy = valid_spawns.pop()
            name = f"ESP{i+1:02d}" 
            color = "#1D86FE"
            heading = 'N'
            if sx in self.traffic_mgr.COLS_UP: heading = 'N'
            elif sx in self.traffic_mgr.COLS_DOWN: heading = 'S'
            elif sy in self.traffic_mgr.ROWS_RIGHT: heading = 'E'
            elif sy in self.traffic_mgr.ROWS_LEFT: heading = 'W'
            r = RobotState(name, color, sub=self.SUB)
            r.x, r.y, r.heading = sx, sy, heading
            r.sync_sub_from_grid()
            # Khởi tạo reservation tại vị trí ban đầu để tránh robot khác spawn đè lên
            self.coop_manager.reservations[(sx, sy, 0)] = name
            self.robots[name] = r
        self.combo_robots['values'] = list(self.robots.keys())
        if self.robots:
            self.combo_robots.current(0)
    def toggle_system_start(self):
        self.is_benchmark_mode = False 
        self.is_running = not self.is_running
        if self.is_running:
            self.start_time = time.time()  # Lưu thời gian bắt đầu
            self.btn_start.config(text="ĐANG CHẠY (NHẤN ĐỂ DỪNG)", bg="#F44336")
        else:
            self.lbl_timer.config(text="Thời gian: --")
            self.btn_start.config(text="KHỞI ĐỘNG HỆ THỐNG", bg="#4CAF50")
    def start_benchmark_60s(self):
        for r in self.robots.values():
            r.delivered_count = 0
        self.is_benchmark_mode = True
        self.start_time = time.time()
        self.is_running = True
        self.btn_start.config(text="ĐANG TEST 60s...", bg="#FF9800")
    # ============================================================
    # VẼ MAP | DRAWING 
    # ============================================================
    def draw_map(self):
        self.canvas.delete("all")
        off = self.MARGIN
        # Vẽ Lưới Dọc
        for x in range(self.grid_n):
            if x not in SKIP_COLS:
                cx = off + x * self.cell
                self.canvas.create_line(cx, off, cx, off + (self.grid_n - 1) * self.cell, width=2, fill="#e0e0e0")
                if x in self.traffic_mgr.COLS_DOWN: 
                     self.canvas.create_text(cx, off - 15, text="↓", fill="gray", font=("Arial", 10, "bold"))
                elif x in self.traffic_mgr.COLS_UP: 
                     self.canvas.create_text(cx, off + (self.grid_n - 1) * self.cell + 15, text="↑", fill="gray", font=("Arial", 10, "bold"))
        # Vẽ Lưới Ngang
        for y in range(self.grid_n):
            if y not in SKIP_ROWS:
                cy = off + (self.grid_n - 1 - y) * self.cell
                self.canvas.create_line(off, cy, off + (self.grid_n - 1) * self.cell, cy, width=2, fill="#e0e0e0")
                if y in self.traffic_mgr.ROWS_LEFT:
                    self.canvas.create_text(off + (self.grid_n - 1) * self.cell + 15, cy, text="←", fill="gray", font=("Arial", 10, "bold"))
                elif y in self.traffic_mgr.ROWS_RIGHT: 
                    self.canvas.create_text(off - 15, cy, text="→", fill="gray", font=("Arial", 10, "bold"))
        # Vẽ Nodes (Điểm trả)
        rr = 7
        for l_id, coords in CONFIG_DELIVERY_LOCATIONS.items():
            color = "#AAF4B7" 
            for (dx, dy) in coords:
                cx = off + dx * self.cell
                cy = off + (self.grid_n - 1 - dy) * self.cell
                self.canvas.create_rectangle(cx-7, cy-7, cx+7, cy+7, fill=color, outline="green", width=1)
                self.canvas.create_text(cx, cy, text=str(l_id), font=("Arial", 7), fill="black")
        # Vẽ Nodes đặc biệt và PICKUP NODES với số
        for x in range(self.grid_n):
            for y in range(self.grid_n):
                cx = off + x * self.cell
                cy = off + (self.grid_n - 1 - y) * self.cell
                
                if (x, y) in BLUE_NODES:
                    self.canvas.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, fill="#4287f5", outline="blue")
                    try:
                        idx = PICKUP_NODES.index((x,y)) + 1
                        self.canvas.create_text(cx, cy, text=str(idx), font=("Arial", 7, "bold"), fill="white")
                    except:
                        pass
                elif (x,y) not in DELIVERY_NODES and (x not in SKIP_COLS or y not in SKIP_ROWS):
                     self.canvas.create_oval(cx - 2, cy - 2, cx + 2, cy + 2, fill="black") 

    def draw_robot(self):
        for name in self.robots:
            self.canvas.delete(f"robot_{name}")
        for name, r in self.robots.items():
            x = r.sx / r.SUB
            y = r.sy / r.SUB
            cx = self.MARGIN + x * self.cell
            cy = self.MARGIN + (self.grid_n - 1 - y) * self.cell
            size = 14
            outline_w = 0; outline_color = "black"; stipple = ""
            if r.state in ["PICKING", "DROPPING"]:
                outline_color = "#41CEC4" 
                outline_w = 4               
            if r.paused:
                outline_w = 4; stipple = "gray50"; outline_color = "black"
            # Vẽ thân Robot
            self.canvas.create_rectangle(
                cx - size, cy - size, cx + size, cy + size,
                fill=r.color, outline=outline_color, width=outline_w, stipple=stipple, tags=f"robot_{name}"
            )
            if r.state in ["TO_DELIVERY", "DROPPING"]:
                cargo_sz = 9
                self.canvas.create_rectangle(
                    cx - cargo_sz, cy - cargo_sz, cx + cargo_sz, cy + cargo_sz,
                    fill="#E1720A", outline="black", tags=f"robot_{name}"
                )
            short_name = name.replace("ESP", "")
            self.canvas.create_text(cx, cy, text=short_name, font=("Arial", 9, "bold"), fill="white", tags=f"robot_{name}")
            tri = {
                'N': [cx, cy - size - 8, cx - 6, cy - size, cx + 6, cy - size],
                'E': [cx + size + 8, cy, cx + size, cy - 6, cx + size, cy + 6],
                'S': [cx, cy + size + 8, cx - 6, cy + size, cx + 6, cy + size],
                'W': [cx - size - 8, cy, cx - size, cy - 6, cx - size, cy + 6],
            }
            self.canvas.create_polygon(tri[r.heading], fill="blue", outline="blue", tags=f"robot_{name}")
    # ============================================================
    #Lộ phí (Dùng BFS Cache cho PSO)
    # ============================================================
    def get_path_cost(self, start_pose, target_xy):
        return self.coop_manager.get_heuristic(start_pose, target_xy)
    # ============================================================
    # CHỌN ĐIỂM TỐI ƯU (1 địa chỉ 3 điểm giao, chọn điểm gần nhất)
    # ============================================================
    def resolve_best_delivery_node(self, logical_id, robot_instance):
        candidates = CONFIG_DELIVERY_LOCATIONS.get(logical_id, [])
        if not candidates:
            return None 
        occupied_targets = set()
        for name, r in self.robots.items():
            if name == robot_instance.name: continue
            # Them ca vi tri hien tai cua robot
            occupied_targets.add((r.x, r.y))
            if r.target_node:
                occupied_targets.add(r.target_node)
        # Uu tien cac diem chua ai chon lam dich va chua co robot o do
        available = [pos for pos in candidates if pos not in occupied_targets]
        if not available: available = candidates
        best_node = None
        min_cost = float('inf')
        start_pose = (robot_instance.x, robot_instance.y, robot_instance.heading)
        for pos in available:
            # Dung BFS heuristic tinh nhanh
            cost = self.coop_manager.get_heuristic(start_pose, pos)
            if cost < min_cost:
                min_cost = cost
                best_node = pos
        return best_node if best_node else candidates[0]
    def _estimate_blocking_time(self, x, y, heading, robot_name):
        """Ước tính thời gian chờ robot đang block phía trước"""
        DIR = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
        dx, dy = DIR.get(heading, (0, 0))
        front_x, front_y = x + dx, y + dy
        # Tìm robot đang ở vị trí phía trước
        for name, r in self.robots.items():
            if name == robot_name:
                continue
            if (r.x, r.y) == (front_x, front_y):
                # Nếu robot đang PICKING/DROPPING, trả về wait_timer còn lại
                if r.state in ["PICKING", "DROPPING"]:
                    return r.wait_timer + 2  # +2 cho buffer di chuyển
                # Nếu robot đang di chuyển hoặc chờ, ước tính ngắn
                elif r.path_queue:
                    return min(len(r.path_queue), 5)  # Tối đa 5 tick
                else:
                    return 3  # Default ngắn
        # Không có robot block trực tiếp, kiểm tra reservation
        for t_offset in range(1, 10):
            key = (front_x, front_y, self.tick_counter + t_offset)
            if key not in self.coop_manager.reservations:
                return t_offset  # Thời gian đến khi ô trống
        return 15  # Default nếu không xác định được
    # ============================================================
    # LOGIC TRUNG TÂM ĐIỀU PHỐI (cho CA*)
    # ============================================================

    def _central_dispatch_idle_extended(self):
        if not self.USE_EXTENDED_PSO:
            return

        idle = []
        for r in self.robots.values():
            if r.paused or r.manual_override:
                continue
            if r.state == "IDLE" and (not r.path_queue) and r.wait_timer == 0 and r.step_backlog == 0 and r.target_node is None:
                idle.append(r)

        if not idle:
            return

        for i in range(0, len(idle), self.BATCH_TARGET):
            group = idle[i:i + self.BATCH_TARGET]

            assignment = self.ext_allocator.run(group, self.tick_counter)
            if not assignment:
                continue

            for r in group:
                sc = assignment.get(r.name)
                if not sc:
                    continue

                pickup = sc['pickup']
                l_id = sc['logical_id']

                r.target_node = pickup
                r.future_delivery_logical_id = l_id
                r.current_job_pickup = pickup

                # plan tới pickup bằng CA*
                self.coop_manager.clear_reservation(r.name, self.tick_counter, (r.x, r.y))
                path, states = self.coop_manager.find_path_space_time(
                    (r.x, r.y, r.heading), pickup, self.tick_counter, r.name
                )
                if path is not None:
                    r.path_queue = path
                    start_pos = (r.x, r.y, r.heading, self.tick_counter)
                    self.coop_manager.reserve_path(r.name, states, 0, start_pos)
                    r.state = "TO_PICKUP"

                    # sinh đơn mới cho pickup vừa được assign (giữ stream đơn liên tục)
                    if pickup in PICKUP_NODES:
                        self.pickup_orders[pickup] = random.choice(self.logical_ids)
                else:
                    # không plan được thì trả lại IDLE
                    r.target_node = None
                    r.future_delivery_logical_id = None
                    r.current_job_pickup = None
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = r.name
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = r.name

    def _server_logic_tick(self):
        if not self.is_running:
             self.root.after(self.LOGIC_MS, self._server_logic_tick)
             return
        self.tick_counter += 1
        if self.is_benchmark_mode:
            elapsed = time.time() - self.start_time
            remaining = max(0, self.TEST_DURATION - elapsed)
            self.lbl_timer.config(text=f"Thời gian: {int(remaining)}s / {self.TEST_DURATION}s")
            if elapsed >= self.TEST_DURATION:
                self.is_running = False
                self.btn_start.config(text="KHỞI ĐỘNG HỆ THỐNG", bg="#4CAF50")
                total_delivered = sum(r.delivered_count for r in self.robots.values())
                self.lbl_timer.config(text=f"⏱ KẾT THÚC TEST 60s!")
                # Popup thông báo kết quả
                messagebox.showinfo("KẾT QUẢ TEST 60s", f"TỔNG HÀNG ĐÃ GIAO: {total_delivered}")
                self.root.after(self.LOGIC_MS, self._server_logic_tick)
                return
        else:
            # Check completion condition: 8 orders delivered
            total_delivered = sum(r.delivered_count for r in self.robots.values())
            if total_delivered >= 8 and not hasattr(self, 'completion_reported'):
                self.completion_reported = True
                total_dist = sum(r.total_distance for r in self.robots.values())
                max_dist = max(r.total_distance for r in self.robots.values())
                comp_time = time.time() - self.start_time
                
                print(f"=== COMPLETION METRICS ===")
                print(f"Total Distance: {total_dist}")
                print(f"Max Single Dist: {max_dist}")
                print(f"Completion Time: {comp_time:.2f}s")
                print("--- Individual Robot Distances ---")
                dist_details = ""
                for name, r in self.robots.items():
                    d_str = f"{name}: {r.total_distance}"
                    print(d_str)
                    dist_details += d_str + "\n"
                
                msg = (f"HOÀN THÀNH 8 ĐƠN HÀNG!\n"
                       f"Tổng quãng đường: {total_dist}\n"
                       f"Max quãng đường 1 robot: {max_dist}\n"
                       f"Thời gian: {comp_time:.2f}s\n\n"
                       f"Chi tiết:\n{dist_details}")
                messagebox.showinfo("KẾT QUẢ", msg)

            #Timer tăng dần cho chế độ chạy bình thường
            elapsed = time.time() - self.start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.lbl_timer.config(text=f"Thời gian: {hours:02d}:{minutes:02d}:{seconds:02d}")
        #Dọn dẹp reservation cũ MỖI TICK để giải quyết tắc nghẽn nhanh hơn
        old_threshold = self.tick_counter - 2
        keys_to_remove = [k for k in self.coop_manager.reservations if k[2] < old_threshold]
        for k in keys_to_remove:
            del self.coop_manager.reservations[k]
        # [DEBUG] Kiem tra va cham: phat hien robot trung toa do
        positions = {}
        for name, r in self.robots.items():
            grid_pos = (r.x, r.y)
            if grid_pos in positions:
                other_name = positions[grid_pos]
                # Collision detected - chi log ASCII de tranh loi encoding
                pass  
            else:
                positions[grid_pos] = name
        # Sắp xếp robot để xử lý: 
        # 1. Ưu tiên robot đang giao hàng (TO_DELIVERY) để giải phóng đường
        # 2. Ưu tiên robot có số ID thấp hơn (ESP01 > ESP02 > ...)
        def get_robot_priority(item):
            name, r = item
            state_priority = 0 if r.state == "TO_DELIVERY" else (1 if r.state == "DROPPING" else 2)
            # Lấy số từ tên robot (ESP01 -> 1, ESP02 -> 2, ...)
            try:
                robot_num = int(name.replace("ESP", ""))
            except:
                robot_num = 99
            return (state_priority, robot_num)
        sorted_robots = sorted(self.robots.items(), key=get_robot_priority)
        # ===== Central dispatch cho các robot IDLE (Extended PSO) =====
        self._central_dispatch_idle_extended()
        for name, r in sorted_robots:
            if r.paused or r.step_backlog > 0: continue
            if r.wait_timer > 0:
                #Duy tri reservation cho robot dang cho (PICKING/DROPPING)
                # Dam bao khong co robot khac den vi tri nay trong thoi gian cho
                for t_offset in range(r.wait_timer + 2):
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_offset)] = name
                r.wait_timer -= 1
                continue
            #Kiểm tra robot có bị stuck không
            current_pos = (r.x, r.y)
            if r.last_position == current_pos:
                r.stuck_counter +=1
            else:
                r.stuck_counter = 0
                r.last_position = current_pos
            
            # Nếu robot bị stuck quá lâu và có path_queue, force replan
            if r.stuck_counter >= r.MAX_STUCK and r.path_queue:
                # Xóa path cũ và xóa reservation
                r.path_queue.clear()
                self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                # Giữ reservation vị trí hiện tại
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                r.stuck_counter = 0
                # Tiếp tục để state machine xử lý việc replan
            
            # Nếu stuck > 25 ticks mà không có path:
            # - TO_DELIVERY: CHỈ clear reservation để replan, KHÔNG mất hàng
            # - TO_PICKUP: reset về IDLE để lấy mục tiêu mới
            if r.stuck_counter >= 25 and not r.path_queue and r.state not in ["IDLE", "PICKING", "DROPPING"]:
                if r.state == "TO_DELIVERY":
                    # Giữ nguyên hàng (target_node, logical_id), chỉ kích replan
                    self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                    r.stuck_counter = 0
                    # Không continue — để state machine TO_DELIVERY replan ngay tick này
                else:
                    # TO_PICKUP stuck → cho phép đổi mục tiêu
                    r.state = "IDLE"
                    r.target_node = None
                    r.current_job_pickup = None
                    r.future_delivery_logical_id = None
                    self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                    r.stuck_counter = 0
            # 1. Xử lý di chuyển theo kế hoạch CA*
            if r.path_queue:
                cmd = r.path_queue.pop(0)
                if cmd == 'WAIT':
                    # Đếm số WAIT liên tiếp ở đầu queue
                    # Nếu có quá nhiều WAIT, xóa path và replan ngay
                    wait_count = 1
                    for next_cmd in r.path_queue:
                        if next_cmd == 'WAIT':
                            wait_count += 1
                        else:
                            break
                    # Nếu có >= 2 WAIT liên tiếp, replan ngay thay vì chờ (nhanh hơn cho nhiều robot)
                    if wait_count >= 2:
                        r.path_queue.clear()
                        self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                        # Giữ reservation vị trí hiện tại
                        self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                        self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                        # Không continue - để rơi xuống state machine replan
                    else:
                        # Robot dung yen 1 nhip, dam bao reservation
                        self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                        self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                        continue
                else:
                    self._execute_command(r, cmd, name)
                    continue
            # 2. State Machine 
            if r.manual_override:
                # Manual override logic giữ nguyên
                continue
            # TRANG THAI: IDLE (Tim hang)
            if r.state == "IDLE":
                # đánh dấu thời điểm bắt đầu IDLE để chờ gom nhóm
                if r.idle_since_tick is None:
                    r.idle_since_tick = self.tick_counter

                # giữ chỗ đứng hiện tại trong lúc chờ dispatch nhóm
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                continue
            # TRẠNG THÁI: TO_PICKUP
            elif r.state == "TO_PICKUP":
                # Đã đến đích (hết path_queue)
                if (r.x, r.y) == r.target_node:
                    r.state = "PICKING"
                    r.wait_timer = 20 
                else:
                    # Nếu hết path mà chưa đến đích -> replan
                    self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                    path, states = self.coop_manager.find_path_space_time(
                        (r.x, r.y, r.heading), r.target_node, self.tick_counter, name
                    )
                    if path is not None:
                        r.path_queue = path
                        start_pos = (r.x, r.y, r.heading, self.tick_counter)
                        self.coop_manager.reserve_path(name, states, 0, start_pos)
                    else:
                        #Ước tính thời gian chờ robot đang block
                        blocking_time = self._estimate_blocking_time(r.x, r.y, r.heading, name)
                        
                        # Tìm alternative pickup tốt nhất
                        best_alt_path = None
                        best_alt_pickup = None
                        best_alt_states = None
                        best_alt_len = float('inf')
                        
                        for alt_pickup in PICKUP_NODES:
                            if alt_pickup == r.target_node:
                                continue
                            alt_path, alt_states = self.coop_manager.find_path_space_time(
                                (r.x, r.y, r.heading), alt_pickup, self.tick_counter, name
                            )
                            if alt_path is not None and len(alt_path) < best_alt_len:
                                best_alt_len = len(alt_path)
                                best_alt_path = alt_path
                                best_alt_pickup = alt_pickup
                                best_alt_states = alt_states
                        
                        # Chờ nếu blocking_time <= alternative path length
                        # Ưu tiên chờ nếu chỉ cần đợi ngắn
                        if blocking_time <= best_alt_len and blocking_time <= 10:
                            # Chờ - giữ reservation
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                        elif best_alt_path is not None:
                            # Alternative ngắn hơn - đổi target
                            r.path_queue = best_alt_path
                            r.target_node = best_alt_pickup
                            r.current_job_pickup = best_alt_pickup
                            start_pos = (r.x, r.y, r.heading, self.tick_counter)
                            self.coop_manager.reserve_path(name, best_alt_states, 0, start_pos)
                        else:
                            # Không có alternative, chờ
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
            # TRẠNG THÁI: PICKING -> Chuyển sang TO_DELIVERY
            elif r.state == "PICKING":
                l_id = r.future_delivery_logical_id
                if l_id is None: l_id = random.choice(list(CONFIG_DELIVERY_LOCATIONS.keys()))
                real_delivery_node = self.resolve_best_delivery_node(l_id, r)
                # [CA* PLANNING] Giữ lại reservation vị trí hiện tại khi lập kế hoạch
                self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                path, states = self.coop_manager.find_path_space_time(
                    (r.x, r.y, r.heading), real_delivery_node, self.tick_counter, name
                )
                if path is not None:
                    r.path_queue = path
                    # Đặt chỗ bao gồm vị trí xuất phát
                    start_pos = (r.x, r.y, r.heading, self.tick_counter)
                    self.coop_manager.reserve_path(name, states, 0, start_pos)
                    r.target_node = real_delivery_node 
                    r.state = "TO_DELIVERY"
                else:
                    # Không tìm được đường ngay, VẪN chuyển TO_DELIVERY để replan hoạt động
                    r.target_node = real_delivery_node
                    r.state = "TO_DELIVERY"
                    # Giữ reservation vị trí hiện tại
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
            # TRẠNG THÁI: TO_DELIVERY
            elif r.state == "TO_DELIVERY":
                if (r.x, r.y) == r.target_node:
                    r.state = "DROPPING"
                    r.wait_timer = 20 
                else:
                    # Nếu hết path mà chưa đến đích, replan
                    self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                    path, states = self.coop_manager.find_path_space_time(
                        (r.x, r.y, r.heading), r.target_node, self.tick_counter, name
                    )
                    if path is not None:
                        r.path_queue = path
                        start_pos = (r.x, r.y, r.heading, self.tick_counter)
                        self.coop_manager.reserve_path(name, states, 0, start_pos)
                    else:
                        #Thử tìm delivery node thay thế cùng logical ID
                        found_alternative = False
                        l_id = r.future_delivery_logical_id
                        if l_id and l_id in CONFIG_DELIVERY_LOCATIONS:
                            for alt_delivery in CONFIG_DELIVERY_LOCATIONS[l_id]:
                                if alt_delivery == r.target_node:
                                    continue
                                alt_path, alt_states = self.coop_manager.find_path_space_time(
                                    (r.x, r.y, r.heading), alt_delivery, self.tick_counter, name
                                )
                                if alt_path is not None:
                                    r.path_queue = alt_path
                                    r.target_node = alt_delivery
                                    start_pos = (r.x, r.y, r.heading, self.tick_counter)
                                    self.coop_manager.reserve_path(name, alt_states, 0, start_pos)
                                    found_alternative = True
                                    break
                        
                        # Nếu vẫn không tìm được, thử các logical_id khác
                        if not found_alternative:
                            for other_l_id in CONFIG_DELIVERY_LOCATIONS.keys():
                                if other_l_id == l_id:
                                    continue
                                for alt_delivery in CONFIG_DELIVERY_LOCATIONS[other_l_id]:
                                    alt_path, alt_states = self.coop_manager.find_path_space_time(
                                        (r.x, r.y, r.heading), alt_delivery, self.tick_counter, name
                                    )
                                    if alt_path is not None:
                                        r.path_queue = alt_path
                                        r.target_node = alt_delivery
                                        r.future_delivery_logical_id = other_l_id
                                        start_pos = (r.x, r.y, r.heading, self.tick_counter)
                                        self.coop_manager.reserve_path(name, alt_states, 0, start_pos)
                                        found_alternative = True
                                        break
                                if found_alternative:
                                    break
                        
                        if not found_alternative:
                            # Giữ reservation và đợi, stuck_counter sẽ tăng ở đầu loop
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
            # TRẠNG THÁI: DROPPING
            elif r.state == "DROPPING":
                r.delivered_count += 1
                r.state = "IDLE"
                r.target_node = None
                r.future_delivery_logical_id = None
                r.current_job_pickup = None

                # Xóa reservation cũ nhưng giữ vị trí hiện tại
                self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
            
            # TRẠNG THÁI: FINISHED (removed)

        # Sau khi xử lý state machine của từng robot xong:
        self._dispatch_idle_buffered_pso()

        self.root.after(self.LOGIC_MS, self._server_logic_tick)
    def _execute_command(self, r, cmd, robot_name):
        if cmd == 'F':
            dx, dy = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}[r.heading]
            new_x, new_y = r.x + dx, r.y + dy
            #Kiem tra truc tiep xem vi tri moi co robot khac khong
            for other_name, other_r in self.robots.items():
                if other_name == robot_name:
                    continue
                # Kiem tra vi tri grid cua robot khac
                if (other_r.x, other_r.y) == (new_x, new_y):
                    # Có robot khác ở vị trí mới
                    # Thay vì chờ, xóa path để force replan NGAY LẬP TỨC
                    # Điều này giúp robot tìm đường tránh hoặc target khác trong < 1s
                    r.path_queue.clear()
                    # Giữ reservation tại chỗ cho tick hiện tại để tránh bị đè
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = robot_name
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = robot_name
                    return False
            r.x = new_x; r.y = new_y; r.step_backlog = r.SUB
            # Only count distance for task-related movements
            if r.state in ["TO_PICKUP", "TO_DELIVERY"]:
                r.total_distance += 1 
            # Cap nhat reservation cho vi tri moi
            self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = robot_name
            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = robot_name
            return True
        elif cmd in ['L', 'R']:
            if cmd == 'L': r.heading = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[r.heading]
            else: r.heading = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[r.heading]
            r.step_backlog = 3
            return True
        return True
    # ============================================================
    # ANIMATION và UI
    # ============================================================
    def _anim_tick(self):
        any_change = False
        for name, r in self.robots.items():
            if r.step_backlog > 0:
                r.step_backlog -= 1
                target_sx = r.x * r.SUB; target_sy = r.y * r.SUB
                if r.sx < target_sx: r.sx += 1
                elif r.sx > target_sx: r.sx -= 1
                if r.sy < target_sy: r.sy += 1
                elif r.sy > target_sy: r.sy -= 1
                any_change = True
        if any_change: self.draw_robot()
        self.root.after(self.ANIM_MS, self._anim_tick)

    def _refresh_status_panel(self):
        # 1. Update bảng trạng thái Robot bên phải
        for item in self.tree_monitor.get_children():
            self.tree_monitor.delete(item)

        STATE_MAP = {
            "IDLE": "Chờ lệnh",
            "TO_PICKUP": "Đi lấy hàng",
            "PICKING": "Đang bốc...",
            "TO_DELIVERY": "Đi giao hàng",
            "DROPPING": "Đang trả...",
            "MANUAL_MOVE": "Thủ công"
        }
        total_delivered = 0
        sorted_robots= sorted(self.robots.keys())
        
        active_pickup_map = {} 

        for name in sorted_robots:
            r= self.robots[name]
            st= STATE_MAP.get(r.state, r.state)
            
            if r.paused: st = "TẠM DỪNG"
            elif r.wait_timer > 0: st += f" ({r.wait_timer})"
            
            total_delivered += r.delivered_count
            pos_str = f"({r.x}, {r.y})"
            tgt_str = f"({r.target_node[0]}, {r.target_node[1]})" if r.target_node else "--"
            del_str = f"Điểm trả {r.future_delivery_logical_id}" if r.future_delivery_logical_id else "--"

            self.tree_monitor.insert("", "end", values=(name, st, pos_str, tgt_str, del_str, r.delivered_count))
            
            if r.current_job_pickup:
                active_pickup_map[r.current_job_pickup] = {
                    'robot': name,
                    'zone': f"Điểm trả {r.future_delivery_logical_id}" if r.future_delivery_logical_id else "?",
                    'delivery_coord': str(r.target_node) if (r.state in ["TO_DELIVERY", "DROPPING"] and r.target_node) else "--"
                }

        self.lbl_total_stats.config(text=f"Tổng hàng đã giao: {total_delivered}")
        # 2. Update bảng giám sát điểm nhận hàng
        for item in self.tree_pickup.get_children():
            self.tree_pickup.delete(item)
        for i, node in enumerate(PICKUP_NODES):
            p_id = f"Trạm {i+1}"
            coord_str = f"{node}"
            data = active_pickup_map.get(node)
            status_str = data['zone'] if data else "Đợi hàng"
            robot_str = data['robot'] if data else "Đang trống"
            delivery_coord_str = data['delivery_coord'] if data else "--"
            self.tree_pickup.insert("", "end", values=(p_id, coord_str, status_str, delivery_coord_str, robot_str))
        self.root.after(200, self._refresh_status_panel)
    def toggle_robot_pause(self):
        name = self.combo_robots.get()
        if name in self.robots:
            self.robots[name].paused = not self.robots[name].paused; self.draw_robot()
    def manual_set_target(self):
        name = self.combo_robots.get()
        try: tx = int(self.ent_x.get()); ty = int(self.ent_y.get())
        except: messagebox.showerror("Error", "Lỗi nhập liệu"); return
        if not (0 <= tx < self.grid_n and 0 <= ty < self.grid_n): return
        if name in self.robots:
            r = self.robots[name]
            r.manual_override = True; r.state = "Đi thủ công"; r.path_queue.clear(); r.target_node = (tx, ty)
            # Dùng logic cũ cho manual để không ảnh hưởng reservation system
            path = self.find_path_old((r.x, r.y, r.heading), (tx, ty))
            if path: r.path_queue = path; r.paused = False
            else: messagebox.showwarning("Warning", "Không tìm thấy đường")    
    def find_path_old(self, start_pose, target_xy):
        # Backup hàm A* cũ cho chế độ Manual
        sx, sy, sh = start_pose
        tx, ty = target_xy
        if (sx, sy) == (tx, ty): return []
        queue = []
        heapq.heappush(queue, (0, 0, sx, sy, sh, []))
        visited_costs = {} 
        DIR = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
        LEFT = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
        RIGHT = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}
        while queue:
            _, cost, cx, cy, ch, path = heapq.heappop(queue)
            if (cx, cy)== (tx, ty): return path
            state = (cx, cy, ch)
            if state in visited_costs and visited_costs[state] <= cost: continue
            visited_costs[state] = cost
            dx, dy = DIR[ch]
            nx, ny = cx + dx, cy + dy
            allowed, penalty = self.traffic_mgr.is_valid_move((cx, cy), (nx, ny))
            if allowed:
                new_cost = cost + penalty
                heuristic = (abs(nx - tx) + abs(ny - ty)) 
                heapq.heappush(queue, (new_cost + heuristic, new_cost, nx, ny, ch, path + ['F']))
            nh = RIGHT[ch]; new_cost = cost + 2 
            heapq.heappush(queue, (new_cost + abs(cx-tx)+abs(cy-ty), new_cost, cx, cy, nh, path + ['R']))
            nh = LEFT[ch]; new_cost = cost + 2
            heapq.heappush(queue, (new_cost + abs(cx-tx)+abs(cy-ty), new_cost, cx, cy, nh, path + ['L']))
        return None
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulationApp(root)
    root.mainloop()