# ============================================================
# BFS Planner — Pathfinding thuần BFS (không reservation)
# Dùng cho tổ hợp: BFS + Random Assignment
# ============================================================
from collections import deque


class BFSPlanner:
    """BFS pathfinder — cùng interface với CooperativePlanner nhưng
    KHÔNG có reservation system (space-time). Chỉ tìm đường ngắn nhất
    trên grid tĩnh dựa theo luật giao thông."""

    def __init__(self, grid_n, traffic_mgr):
        self.grid_n = grid_n
        self.traffic_mgr = traffic_mgr
        # Giữ reservations dict rỗng — FleetManager vẫn gọi nhưng không ảnh hưởng
        self.reservations = {}
        self.bfs_cache = {}
        self.all_nodes = [(x, y) for x in range(grid_n) for y in range(grid_n)]
        self.precompute_all_pairs()

    def precompute_all_pairs(self):
        for start_node in self.all_nodes:
            self._bfs_from_source(start_node)

    def _bfs_from_source(self, start_node):
        q = deque([(start_node, 0)])
        visited = {start_node: 0}
        while q:
            (cx, cy), dist = q.popleft()
            self.bfs_cache[(start_node, (cx, cy))] = dist
            neighbors = self.traffic_mgr.get_valid_neighbors(cx, cy)
            for (nx, ny), _ in neighbors:
                if (nx, ny) not in visited:
                    visited[(nx, ny)] = dist + 1
                    q.append(((nx, ny), dist + 1))
        for target in self.all_nodes:
            if (start_node, target) not in self.bfs_cache:
                self.bfs_cache[(start_node, target)] = 9999

    def get_heuristic(self, start_pose, target_xy):
        sx, sy = start_pose[0], start_pose[1]
        return self.bfs_cache.get(((sx, sy), target_xy),
                                   abs(sx - target_xy[0]) + abs(sy - target_xy[1]))

    # --- Reservation stubs (tương thích FleetManager) ---
    def clear_reservation(self, robot_name, keep_current_time=None, keep_pos=None):
        keys_to_remove = [k for k, v in self.reservations.items() if v == robot_name]
        for k in keys_to_remove:
            del self.reservations[k]

    def clear_old_reservations(self, current_time):
        keys_to_remove = [k for k in self.reservations if k[2] < current_time]
        for k in keys_to_remove:
            del self.reservations[k]

    def reserve_path(self, robot_name, path_states, start_time, start_pos=None):
        # BFS không dùng reservation — stub rỗng
        pass

    # --- BFS pathfinding ---
    def find_path_space_time(self, start_pose, target_xy, current_time, robot_name, max_depth=200):
        """Tìm đường bằng BFS thuần trên grid (bỏ qua time dimension).
        Trả (path_cmds, states) tương thích CooperativePlanner."""
        sx, sy, sh = start_pose
        if (sx, sy) == target_xy:
            return [], []
        if self.get_heuristic((sx, sy), target_xy) >= 9999:
            return None, None

        DIR = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
        LEFT = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
        RIGHT = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}

        # BFS trên state = (x, y, heading)
        start_state = (sx, sy, sh)
        queue = deque()
        queue.append((sx, sy, sh, [], []))
        visited = set()
        visited.add(start_state)

        while queue:
            cx, cy, ch, path, states = queue.popleft()
            if len(path) > max_depth:
                continue
            t = current_time + len(path)

            # MOVE forward
            dx, dy = DIR[ch]
            nx, ny = cx + dx, cy + dy
            allowed, _ = self.traffic_mgr.is_valid_move((cx, cy), (nx, ny))
            if allowed:
                if (nx, ny) == target_xy:
                    new_path = path + ['F']
                    new_states = states + [(nx, ny, ch, t + 1)]
                    return new_path, new_states
                new_state = (nx, ny, ch)
                if new_state not in visited:
                    visited.add(new_state)
                    queue.append((nx, ny, ch,
                                  path + ['F'],
                                  states + [(nx, ny, ch, t + 1)]))

            # TURN left / right
            for turn_cmd, next_h in [('L', LEFT[ch]), ('R', RIGHT[ch])]:
                new_state = (cx, cy, next_h)
                if new_state not in visited:
                    visited.add(new_state)
                    queue.append((cx, cy, next_h,
                                  path + [turn_cmd],
                                  states + [(cx, cy, next_h, t + 1)]))

        return None, None
