# ============================================================
# Fleet Manager | Bộ não trung tâm điều phối (MÔ PHỎNG)
# Tách từ RobotSimulationApp._server_logic_tick()
# ============================================================
import random
import heapq
import time

from config.settings import (
    PICKUP_NODES, CONFIG_DELIVERY_LOCATIONS, DELIVERY_NODES,
    REMOVE_NODES, SKIP_COLS, SKIP_ROWS,
    WAIT_TIMER_PICK, WAIT_TIMER_DROP, TEST_DURATION,
)
from core.robot_state import RobotState
from algorithms.pso_allocator import PSO_TaskAllocator


class FleetManager:
    """Bộ não trung tâm: quản lý trạng thái robot + state machine điều phối."""

    def __init__(self, grid_n, traffic_mgr, coop_planner, robot_proxy, sub=10,
                 algorithm_mode="ca_pso", allocator_class=None):
        self.grid_n = grid_n
        self.traffic_mgr = traffic_mgr
        self.coop_manager = coop_planner
        self.proxy = robot_proxy
        self.SUB = sub
        self.robots = {}
        self.tick_counter = 0
        self.is_running = False
        self.is_benchmark_mode = False
        self.start_time = None
        self.TEST_DURATION = TEST_DURATION
        self.algorithm_mode = algorithm_mode
        self.allocator_class = allocator_class  # None → dùng PSO mặc định

        # Hệ thống đơn hàng tập trung: mỗi pickup có 1 logical_id cố định
        # Tất cả allocator đọc từ đây thay vì tự tính
        self.pickup_orders = {
            (0,0): 1,  (3,0): 2,  (6,0): 3,  (9,0): 4,
            (1,10): 5, (4,10): 6, (7,10): 7, (10,10): 8
        }

    # ============================================================
    # SETUP ROBOTS
    # ============================================================
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
        for i in range(count):
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
            r.visual_heading = heading  # Khởi tạo hướng hiển thị
            r.sync_sub_from_grid()
            self.coop_manager.reservations[(sx, sy, 0)] = name
            self.robots[name] = r

    def setup_robots_deterministic(self, count):
        """Spawn robot tại vị trí cố định gần pickup nodes.
        Cùng N → cùng vị trí mọi lần chạy, bất kể thuật toán."""
        self.robots.clear()
        self.tick_counter = 0
        self.coop_manager.reservations.clear()

        # Tạo danh sách vị trí spawn cố định gần pickup nodes
        # Mỗi pickup node có 1 nhóm ô lân cận (sắp xếp cố định)
        spawn_pool = self._build_deterministic_spawn_pool()

        for i in range(min(count, len(spawn_pool))):
            sx, sy = spawn_pool[i]
            name = f"ESP{i+1:02d}"
            color = "#1D86FE"
            heading = 'N'
            if sx in self.traffic_mgr.COLS_UP: heading = 'N'
            elif sx in self.traffic_mgr.COLS_DOWN: heading = 'S'
            elif sy in self.traffic_mgr.ROWS_RIGHT: heading = 'E'
            elif sy in self.traffic_mgr.ROWS_LEFT: heading = 'W'
            r = RobotState(name, color, sub=self.SUB)
            r.x, r.y, r.heading = sx, sy, heading
            r.visual_heading = heading
            r.sync_sub_from_grid()
            self.coop_manager.reservations[(sx, sy, 0)] = name
            self.robots[name] = r

    def _build_deterministic_spawn_pool(self):
        """Xây danh sách vị trí spawn cố định, phân bổ đều quanh pickup nodes.
        Danh sách luôn có thứ tự cố định → deterministic."""
        # Tất cả ô hợp lệ (không phải obstacle, không phải skip)
        valid_cells = set()
        for x in range(self.grid_n):
            for y in range(self.grid_n):
                if (x, y) not in REMOVE_NODES:
                    if x not in SKIP_COLS or y not in SKIP_ROWS:
                        valid_cells.add((x, y))

        # Loại bỏ các ô là delivery nodes (tránh spawn trên ô giao hàng)
        delivery_set = set(DELIVERY_NODES)
        valid_cells -= delivery_set

        # Tính khoảng cách Manhattan từ mỗi ô tới pickup node gần nhất
        def min_dist_to_pickup(pos):
            return min(abs(pos[0] - p[0]) + abs(pos[1] - p[1]) for p in PICKUP_NODES)

        # Sắp xếp theo: khoảng cách tới pickup gần nhất → tọa độ x → tọa độ y
        # (đảm bảo thứ tự cố định với cùng khoảng cách)
        sorted_cells = sorted(valid_cells, key=lambda pos: (min_dist_to_pickup(pos), pos[0], pos[1]))

        # Phân bổ round-robin: chia đều quanh 8 pickup nodes
        # Nhóm theo pickup gần nhất
        groups = {p: [] for p in PICKUP_NODES}
        assigned = set()
        for cell in sorted_cells:
            if cell in PICKUP_NODES:
                continue  # Không spawn ngay trên pickup node
            nearest_pickup = min(PICKUP_NODES,
                                  key=lambda p: (abs(cell[0]-p[0]) + abs(cell[1]-p[1]), p[0], p[1]))
            groups[nearest_pickup].append(cell)

        # Build spawn pool: round-robin qua 8 nhóm pickup
        spawn_pool = []
        pickup_list = sorted(PICKUP_NODES)  # Thứ tự cố định
        max_group_size = max(len(g) for g in groups.values()) if groups else 0
        for idx in range(max_group_size):
            for p in pickup_list:
                if idx < len(groups[p]):
                    cell = groups[p][idx]
                    if cell not in assigned:
                        spawn_pool.append(cell)
                        assigned.add(cell)

        return spawn_pool

    # ============================================================
    # UTILITY
    # ============================================================
    def get_path_cost(self, start_pose, target_xy):
        return self.coop_manager.get_heuristic(start_pose, target_xy)

    def resolve_best_delivery_node(self, logical_id, robot_instance):
        candidates = CONFIG_DELIVERY_LOCATIONS.get(logical_id, [])
        if not candidates: return None
        occupied_targets = set()
        for name, r in self.robots.items():
            if name == robot_instance.name: continue
            occupied_targets.add((r.x, r.y))
            if r.target_node:
                occupied_targets.add(r.target_node)
        available = [pos for pos in candidates if pos not in occupied_targets]
        if not available: available = candidates
        best_node = None
        min_cost = float('inf')
        start_pose = (robot_instance.x, robot_instance.y, robot_instance.heading)
        for pos in available:
            cost = self.coop_manager.get_heuristic(start_pose, pos)
            if cost < min_cost:
                min_cost = cost
                best_node = pos
        return best_node if best_node else candidates[0]

    def _estimate_blocking_time(self, x, y, heading, robot_name):
        DIR = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
        dx, dy = DIR.get(heading, (0, 0))
        front_x, front_y = x + dx, y + dy
        for name, r in self.robots.items():
            if name == robot_name: continue
            if (r.x, r.y) == (front_x, front_y):
                if r.state in ["PICKING", "DROPPING"]:
                    return r.wait_timer + 2
                elif r.path_queue:
                    return min(len(r.path_queue), 5)
                else:
                    return 3
        for t_offset in range(1, 10):
            key = (front_x, front_y, self.tick_counter + t_offset)
            if key not in self.coop_manager.reservations:
                return t_offset
        return 15

    def _has_free_neighbor(self, cx, cy, check_time):
        moves = [(0,1), (0,-1), (1,0), (-1,0)]
        for dx, dy in moves:
            nx, ny = cx + dx, cy + dy
            allowed, _ = self.traffic_mgr.is_valid_move((cx, cy), (nx, ny))
            if allowed:
                if (nx, ny, check_time) not in self.coop_manager.reservations:
                    return True
        return False

    def _is_adjacent_blocked_by_working(self, r, robot_name):
        """Kiểm tra: đích đến CÓ ngay sát bên VÀ đang bị robot PICKING/DROPPING chắn.
        Nếu True → ĐỨNG YÊN CHỜ — KHÔNG replan, KHÔNG xoay.
        Returns: int - Số tick cần chờ (0 nếu không bị chắn)
        """
        if not r.target_node:
            return 0
        tx, ty = r.target_node
        if abs(r.x - tx) + abs(r.y - ty) > 2:
            return 0
        for other_name, other_r in self.robots.items():
            if other_name == robot_name:
                continue
            if (other_r.x, other_r.y) == (tx, ty) and other_r.state in ["PICKING", "DROPPING"]:
                return other_r.wait_timer + 2
        return 0

    def _is_front_blocked(self, r, robot_name):
        """Kiểm tra ô phía trước có robot nào đang đứng không.
        Nếu có → nên chờ thay vì xoay/reroute (robot thật đi chậm).
        Returns: int - Số tick nên chờ (0 nếu trống)
        """
        DIR = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
        dx, dy = DIR.get(r.heading, (0, 0))
        fx, fy = r.x + dx, r.y + dy
        for other_name, other_r in self.robots.items():
            if other_name == robot_name:
                continue
            if (other_r.x, other_r.y) == (fx, fy):
                if other_r.state in ["PICKING", "DROPPING"]:
                    return other_r.wait_timer + 3
                elif other_r.path_queue:
                    return min(len(other_r.path_queue) + 1, 8)
                else:
                    return 3
        return 0

    def _attempt_escape(self, r, robot_name):
        moves = [(0,1), (0,-1), (1,0), (-1,0)]
        random.shuffle(moves)
        for dx, dy in moves:
            nx, ny = r.x + dx, r.y + dy
            allowed, penalty = self.traffic_mgr.is_valid_move((r.x, r.y), (nx, ny))
            if allowed:
                if (nx, ny, self.tick_counter + 1) not in self.coop_manager.reservations:
                    if dx == 0 and dy == 1: head = 'N'
                    elif dx == 0 and dy == -1: head = 'S'
                    elif dx == 1 and dy == 0: head = 'E'
                    else: head = 'W'
                    cmds = []
                    if r.heading != head:
                        if (r.heading == 'N' and head == 'S') or \
                           (r.heading == 'S' and head == 'N') or \
                           (r.heading == 'E' and head == 'W') or \
                           (r.heading == 'W' and head == 'E'):
                            cmds = ['L', 'L']
                        elif (r.heading == 'N' and head == 'W') or \
                             (r.heading == 'W' and head == 'S') or \
                             (r.heading == 'S' and head == 'E') or \
                             (r.heading == 'E' and head == 'N'):
                            cmds = ['L']
                        else:
                            cmds = ['R']
                    cmds.append('F')
                    r.path_queue = cmds
                    self.coop_manager.clear_reservation(robot_name, self.tick_counter, (r.x, r.y))
                    t = self.tick_counter
                    self.coop_manager.reservations[(r.x, r.y, t)] = robot_name
                    t += 1
                    for _ in range(len(cmds)-1):
                        self.coop_manager.reservations[(r.x, r.y, t)] = robot_name
                        t += 1
                    self.coop_manager.reservations[(nx, ny, t)] = robot_name
                    r.stuck_counter = 0
                    return True
        return False

    # ============================================================
    # UPDATE TICK — Logic trung tâm điều phối
    # ============================================================
    def update_tick(self):
        self.tick_counter += 1

        # Benchmark timer
        if self.is_benchmark_mode:
            from config.settings import BENCHMARK_TICKS_LIMIT
            if self.tick_counter >= BENCHMARK_TICKS_LIMIT:
                self.is_running = False
                total_delivered = sum(r.delivered_count for r in self.robots.values())
                elapsed = time.time() - self.start_time
                return {'event': 'benchmark_done', 'total': total_delivered,
                        'elapsed': elapsed, 'duration': elapsed}

        # Dọn dẹp reservation cũ
        self.coop_manager.clear_old_reservations(self.tick_counter - 2)

        # Sắp xếp robot theo ưu tiên
        def get_robot_priority(item):
            name, r = item
            state_priority = 0 if r.state == "TO_DELIVERY" else (1 if r.state == "DROPPING" else 2)
            try:
                robot_num = int(name.replace("ESP", ""))
            except:
                robot_num = 99
            return (state_priority, robot_num)

        sorted_robots = sorted(self.robots.items(), key=get_robot_priority)

        # === Batch PSO: phân bổ TẤT CẢ robot IDLE cùng lúc ===
        batch_assignments = {}
        if self.allocator_class is None or self.allocator_class is PSO_TaskAllocator:
            idle_list = []
            for rname, rb in sorted_robots:
                if rb.paused or rb.wait_timer > 0:
                    continue
                if rb.state == "IDLE" and not rb.manual_override and not rb.path_queue:
                    idle_list.append(rb)
            if idle_list:
                batch_pso = PSO_TaskAllocator(self)
                batch_assignments = batch_pso.run(idle_list, self.tick_counter)

        for name, r in sorted_robots:
            if r.paused: continue
            if r.wait_timer > 0:
                for t_offset in range(r.wait_timer + 2):
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_offset)] = name
                r.wait_timer -= 1
                continue

            # Stuck detection
            current_pos = (r.x, r.y)
            if r.last_position == current_pos:
                r.stuck_counter += 1
            else:
                r.stuck_counter = 0
                r.last_position = current_pos

            if r.stuck_counter >= r.MAX_STUCK and r.path_queue:
                r.path_queue.clear()
                self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                r.stuck_counter = 0
                r.total_stuck_events += 1  # Metric: deadlock event

            if r.stuck_counter >= 25 and not r.path_queue and r.state not in ["IDLE", "PICKING", "DROPPING", "TO_DELIVERY"]:
                r.state = "IDLE"
                r.target_node = None
                r.current_job_pickup = None
                r.future_delivery_logical_id = None
                self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                r.stuck_counter = 0

            # 1. Execute path queue
            if r.path_queue:
                if r.target_node and (r.x, r.y) == r.target_node:
                    r.path_queue.clear()
                    self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                else:
                    cmd = r.path_queue.pop(0)
                    if cmd == 'WAIT':
                        r.total_wait_ticks += 1  # Metric: wait tick
                        wait_count = 1
                        for next_cmd in r.path_queue:
                            if next_cmd == 'WAIT': wait_count += 1
                            else: break
                        if wait_count >= 20:
                            r.path_queue.clear()
                            self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                        else:
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                            continue
                    else:
                        if cmd == 'F':
                            r.total_distance += 1  # Metric: distance
                        self.proxy.execute_command(r, cmd, name, self.robots, self.tick_counter, self.coop_manager)
                        continue

            # 2. State Machine
            if r.manual_override:
                continue

            if r.state == "IDLE":
                # Lấy kết quả từ batch PSO (nếu có), hoặc fallback per-robot
                if name in batch_assignments:
                    best_scenario = batch_assignments[name]
                elif self.allocator_class and self.allocator_class is not PSO_TaskAllocator:
                    # Allocator khác (Hungarian, Random, ...) vẫn chạy per-robot
                    allocator = self.allocator_class((r.x, r.y, r.heading), r.heading, self, name)
                    best_scenario = allocator.run()
                else:
                    # Fallback greedy: robot vừa IDLE nhưng không có batch assignment
                    best_cost = float('inf')
                    best_scenario = None
                    for p in PICKUP_NODES:
                        occupied = False
                        for rn, rb in self.robots.items():
                            if rn == name: continue
                            if rb.target_node == p and rb.state in ["TO_PICKUP", "PICKING"]:
                                occupied = True; break
                        if occupied: continue
                        lid = self.pickup_orders.get(p, 1)
                        cost = self.coop_manager.get_heuristic((r.x, r.y, r.heading), p)
                        if cost < best_cost:
                            best_cost = cost
                            best_scenario = {'pickup': p, 'logical_id': lid}
                    if best_scenario is None:
                        continue
                pickup = best_scenario['pickup']
                future_l_id = best_scenario['logical_id']
                if pickup:
                    r.target_node = pickup
                    r.future_delivery_logical_id = future_l_id
                    r.current_job_pickup = pickup
                    if (r.x, r.y) == pickup:
                        r.state = "PICKING"
                        r.wait_timer = 0
                    else:
                        self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                        path, states = self.coop_manager.find_path_space_time(
                            (r.x, r.y, r.heading), pickup, self.tick_counter, name)
                        if path is not None:
                            r.path_queue = path
                            start_pos = (r.x, r.y, r.heading, self.tick_counter)
                            self.coop_manager.reserve_path(name, states, 0, start_pos)
                            r.state = "TO_PICKUP"
                        else:
                            found_alt = False
                            has_exit = self._has_free_neighbor(r.x, r.y, self.tick_counter + 1)
                            if has_exit:
                                best_alt_path = None
                                best_alt_pickup = None
                                best_alt_states = None
                                best_alt_len = float('inf')
                                for alt_pickup in PICKUP_NODES:
                                    if alt_pickup == pickup: continue
                                    alt_path, alt_states = self.coop_manager.find_path_space_time(
                                        (r.x, r.y, r.heading), alt_pickup, self.tick_counter, name)
                                    if alt_path is not None and len(alt_path) < best_alt_len:
                                        best_alt_len = len(alt_path)
                                        best_alt_path = alt_path
                                        best_alt_pickup = alt_pickup
                                        best_alt_states = alt_states
                                if best_alt_path is not None:
                                    r.path_queue = best_alt_path
                                    r.target_node = best_alt_pickup
                                    r.current_job_pickup = best_alt_pickup
                                    start_pos = (r.x, r.y, r.heading, self.tick_counter)
                                    self.coop_manager.reserve_path(name, best_alt_states, 0, start_pos)
                                    r.state = "TO_PICKUP"
                                    found_alt = True
                            if not found_alt:
                                r.path_queue = ['WAIT'] * 8
                                for t_off in range(10):
                                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_off)] = name

            elif r.state == "TO_PICKUP":
                if (r.x, r.y) == r.target_node:
                    r.state = "PICKING"
                    r.wait_timer = WAIT_TIMER_PICK
                else:
                    adj_wait = self._is_adjacent_blocked_by_working(r, name)
                    if adj_wait > 0:
                        wait_ticks = min(adj_wait, 25)
                        r.path_queue = ['WAIT'] * wait_ticks
                        for t_off in range(wait_ticks + 2):
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_off)] = name
                        continue
                    front_wait = self._is_front_blocked(r, name)
                    if front_wait > 0:
                        wait_ticks = min(front_wait, 15)
                        r.path_queue = ['WAIT'] * wait_ticks
                        for t_off in range(wait_ticks + 2):
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_off)] = name
                        continue
                    self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                    path, states = self.coop_manager.find_path_space_time(
                        (r.x, r.y, r.heading), r.target_node, self.tick_counter, name)
                    if path is not None:
                        r.path_queue = path
                        start_pos = (r.x, r.y, r.heading, self.tick_counter)
                        self.coop_manager.reserve_path(name, states, 0, start_pos)
                    else:
                        blocking_time = self._estimate_blocking_time(r.x, r.y, r.heading, name)
                        best_alt_path = None
                        best_alt_pickup = None
                        best_alt_states = None
                        best_alt_len = float('inf')
                        has_exit = self._has_free_neighbor(r.x, r.y, self.tick_counter + 1)
                        if has_exit:
                            for alt_pickup in PICKUP_NODES:
                                if alt_pickup == r.target_node: continue
                                alt_path, alt_states = self.coop_manager.find_path_space_time(
                                    (r.x, r.y, r.heading), alt_pickup, self.tick_counter, name)
                                if alt_path is not None and len(alt_path) < best_alt_len:
                                    best_alt_len = len(alt_path)
                                    best_alt_path = alt_path
                                    best_alt_pickup = alt_pickup
                                    best_alt_states = alt_states
                        # Ưu tiên chờ (robot thật đi chậm, reroute tốn thời gian)
                        if blocking_time <= 20:
                            wait_ticks = min(blocking_time + 2, 15)
                            r.path_queue = ['WAIT'] * wait_ticks
                            for t_offset in range(wait_ticks + 2):
                                self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_offset)] = name
                        elif best_alt_path is not None and best_alt_len < blocking_time:
                            r.path_queue = best_alt_path
                            r.target_node = best_alt_pickup
                            r.current_job_pickup = best_alt_pickup
                            start_pos = (r.x, r.y, r.heading, self.tick_counter)
                            self.coop_manager.reserve_path(name, best_alt_states, 0, start_pos)
                        else:
                            r.path_queue = ['WAIT'] * 8
                            for t_off in range(10):
                                self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_off)] = name

            elif r.state == "PICKING":
                l_id = r.future_delivery_logical_id
                if l_id is None: l_id = random.choice(list(CONFIG_DELIVERY_LOCATIONS.keys()))
                real_delivery_node = self.resolve_best_delivery_node(l_id, r)
                self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                has_exit = self._has_free_neighbor(r.x, r.y, self.tick_counter + 1)
                path = None; states = None
                if has_exit:
                    path, states = self.coop_manager.find_path_space_time(
                        (r.x, r.y, r.heading), real_delivery_node, self.tick_counter, name)
                if path is not None:
                    r.path_queue = path
                    start_pos = (r.x, r.y, r.heading, self.tick_counter)
                    self.coop_manager.reserve_path(name, states, 0, start_pos)
                    r.target_node = real_delivery_node
                    r.state = "TO_DELIVERY"
                else:
                    r.path_queue = ['WAIT']
                    r.target_node = real_delivery_node
                    r.state = "TO_DELIVERY"
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name
                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 2)] = name

            elif r.state == "TO_DELIVERY":
                if (r.x, r.y) == r.target_node:
                    r.state = "DROPPING"
                    r.wait_timer = WAIT_TIMER_DROP
                else:
                    adj_wait = self._is_adjacent_blocked_by_working(r, name)
                    if adj_wait > 0:
                        wait_ticks = min(adj_wait, 25)
                        r.path_queue = ['WAIT'] * wait_ticks
                        for t_off in range(wait_ticks + 2):
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_off)] = name
                        continue
                    front_wait = self._is_front_blocked(r, name)
                    if front_wait > 0:
                        wait_ticks = min(front_wait, 15)
                        r.path_queue = ['WAIT'] * wait_ticks
                        for t_off in range(wait_ticks + 2):
                            self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_off)] = name
                        continue
                    self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                    has_exit = self._has_free_neighbor(r.x, r.y, self.tick_counter + 1)
                    path = None; states = None
                    if has_exit:
                        path, states = self.coop_manager.find_path_space_time(
                            (r.x, r.y, r.heading), r.target_node, self.tick_counter, name)
                    if path is not None:
                        r.path_queue = path
                        start_pos = (r.x, r.y, r.heading, self.tick_counter)
                        self.coop_manager.reserve_path(name, states, 0, start_pos)
                    else:
                        found_alternative = False
                        if has_exit:
                            l_id = r.future_delivery_logical_id
                            if l_id and l_id in CONFIG_DELIVERY_LOCATIONS:
                                for alt_delivery in CONFIG_DELIVERY_LOCATIONS[l_id]:
                                    if alt_delivery == r.target_node: continue
                                    alt_path, alt_states = self.coop_manager.find_path_space_time(
                                        (r.x, r.y, r.heading), alt_delivery, self.tick_counter, name)
                                    if alt_path is not None:
                                        r.path_queue = alt_path
                                        r.target_node = alt_delivery
                                        start_pos = (r.x, r.y, r.heading, self.tick_counter)
                                        self.coop_manager.reserve_path(name, alt_states, 0, start_pos)
                                        found_alternative = True
                                        break
                            if not found_alternative:
                                alt_count = 0
                                for other_l_id in CONFIG_DELIVERY_LOCATIONS.keys():
                                    if other_l_id == l_id: continue
                                    if alt_count >= 3: break
                                    alt_count += 1
                                    for alt_delivery in CONFIG_DELIVERY_LOCATIONS[other_l_id]:
                                        alt_path, alt_states = self.coop_manager.find_path_space_time(
                                            (r.x, r.y, r.heading), alt_delivery, self.tick_counter, name)
                                        if alt_path is not None:
                                            r.path_queue = alt_path
                                            r.target_node = alt_delivery
                                            r.future_delivery_logical_id = other_l_id
                                            start_pos = (r.x, r.y, r.heading, self.tick_counter)
                                            self.coop_manager.reserve_path(name, alt_states, 0, start_pos)
                                            found_alternative = True
                                            break
                                    if found_alternative: break
                        if not found_alternative:
                            escaped = False
                            # Escape sớm hơn để tránh deadlock
                            if r.stuck_counter > 20:
                                escaped = self._attempt_escape(r, name)
                            if not escaped:
                                r.path_queue = ['WAIT'] * 8
                                for t_off in range(10):
                                    self.coop_manager.reservations[(r.x, r.y, self.tick_counter + t_off)] = name

            elif r.state == "DROPPING":
                r.delivered_count += 1
                # Re-assign đơn hàng mới cho pickup vừa phục vụ
                if r.current_job_pickup and r.current_job_pickup in PICKUP_NODES:
                    # Re-assign phân tán: chọn logical_id ÍT pickup dùng nhất (trong top 6 gần)
                    px, py = r.current_job_pickup
                    all_lids = list(CONFIG_DELIVERY_LOCATIONS.keys())
                    all_lids.sort(key=lambda lid: min(
                        abs(px - d[0]) + abs(py - d[1])
                        for d in CONFIG_DELIVERY_LOCATIONS[lid]))
                    top_k = all_lids[:6]
                    # Đếm số pickup đang dùng mỗi logical_id
                    counts = {}
                    for lid in self.pickup_orders.values():
                        counts[lid] = counts.get(lid, 0) + 1
                    # Chọn logical_id ít bị dùng nhất trong top 6
                    top_k.sort(key=lambda lid: counts.get(lid, 0))
                    self.pickup_orders[r.current_job_pickup] = top_k[0]
                r.state = "IDLE"
                r.target_node = None
                r.future_delivery_logical_id = None
                r.current_job_pickup = None
                self.coop_manager.clear_reservation(name, self.tick_counter, (r.x, r.y))
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter)] = name
                self.coop_manager.reservations[(r.x, r.y, self.tick_counter + 1)] = name

        return None

    # ============================================================
    # Backup hàm A* cũ cho chế độ Manual
    # ============================================================
    def find_path_old(self, start_pose, target_xy):
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
            if (cx, cy) == (tx, ty): return path
            state = (cx, cy, ch)
            if state in visited_costs and visited_costs[state] <= cost: continue
            visited_costs[state] = cost
            dx, dy = DIR[ch]
            nx, ny = cx + dx, cy + dy
            allowed, penalty = self.traffic_mgr.is_valid_move((cx, cy), (nx, ny))
            if allowed:
                new_cost = cost + penalty
                heuristic = abs(nx - tx) + abs(ny - ty)
                heapq.heappush(queue, (new_cost + heuristic, new_cost, nx, ny, ch, path + ['F']))
            nh = RIGHT[ch]; new_cost = cost + 2
            heapq.heappush(queue, (new_cost + abs(cx-tx)+abs(cy-ty), new_cost, cx, cy, nh, path + ['R']))
            nh = LEFT[ch]; new_cost = cost + 2
            heapq.heappush(queue, (new_cost + abs(cx-tx)+abs(cy-ty), new_cost, cx, cy, nh, path + ['L']))
        return None
