# ============================================================
# Standard A* Planner — A* không cooperative (không reservation)
# Dùng cho tổ hợp: A* + PSO
# ============================================================
import heapq
from collections import deque


class AStarPlanner:
    """A* pathfinder thuần — cùng interface với CooperativePlanner nhưng
    KHÔNG có reservation system (space-time). Dùng heuristic BFS-precomputed
    giống CooperativePlanner."""

    def __init__(self, grid_n, traffic_mgr):
        self.grid_n = grid_n
        self.traffic_mgr = traffic_mgr
        # Giữ reservations dict — FleetManager vẫn gọi nhưng không ảnh hưởng pathfinding
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
        # A* thuần không dùng reservation — stub rỗng
        pass

    # --- A* pathfinding ---
    def find_path_space_time(self, start_pose, target_xy, current_time, robot_name, max_depth=200):
        """Tìm đường bằng A* thuần (không reservation, không space-time).
        Trả (path_cmds, states) tương thích CooperativePlanner."""
        sx, sy, sh = start_pose
        if (sx, sy) == target_xy:
            return [], []
        if self.get_heuristic((sx, sy), target_xy) >= 9999:
            return None, None

        DIR = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
        LEFT = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
        RIGHT = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}

        # A* trên state = (x, y, heading)
        # Priority queue: (f, g, x, y, heading, path_cmds, states)
        h0 = self.get_heuristic((sx, sy), target_xy)
        queue = [(h0, 0, sx, sy, sh, [], [])]
        visited_costs = {}

        while queue:
            f, g, cx, cy, ch, path, states = heapq.heappop(queue)
            if g > max_depth:
                continue
            if (cx, cy) == target_xy:
                return path, states

            state_key = (cx, cy, ch)
            if state_key in visited_costs and visited_costs[state_key] <= g:
                continue
            visited_costs[state_key] = g
            t = current_time + g

            # MOVE forward
            dx, dy = DIR[ch]
            nx, ny = cx + dx, cy + dy
            allowed, penalty = self.traffic_mgr.is_valid_move((cx, cy), (nx, ny))
            if allowed:
                new_g = g + penalty
                h = self.get_heuristic((nx, ny), target_xy)
                heapq.heappush(queue, (new_g + h, new_g, nx, ny, ch,
                                       path + ['F'],
                                       states + [(nx, ny, ch, t + 1)]))

            # TURN left / right
            for turn_cmd, next_h in [('L', LEFT[ch]), ('R', RIGHT[ch])]:
                new_g = g + 1
                h = self.get_heuristic((cx, cy), target_xy)
                heapq.heappush(queue, (new_g + h, new_g, cx, cy, next_h,
                                       path + [turn_cmd],
                                       states + [(cx, cy, next_h, t + 1)]))

        return None, None
