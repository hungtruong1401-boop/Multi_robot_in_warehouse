# ============================================================
# SimulationProxy | Thực thi lệnh mô phỏng
# ============================================================


class SimulationProxy:
    """Thực thi lệnh di chuyển trên trạng thái nội bộ (mô phỏng)."""

    def execute_command(self, r, cmd, robot_name, robots, tick_counter, coop_manager):
        if cmd == 'F':
            dx, dy = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}[r.heading]
            new_x, new_y = r.x + dx, r.y + dy
            # Kiểm tra va chạm
            for other_name, other_r in robots.items():
                if other_name == robot_name:
                    continue
                if (other_r.x, other_r.y) == (new_x, new_y):
                    r.path_queue.clear()
                    coop_manager.reservations[(r.x, r.y, tick_counter)] = robot_name
                    coop_manager.reservations[(r.x, r.y, tick_counter + 1)] = robot_name
                    return False
            r.x = new_x
            r.y = new_y
            r.visual_heading = r.heading  # Chỉ cập nhật hướng HIỂN THỊ khi THẬT SỰ di chuyển
            r.step_backlog = r.SUB
            coop_manager.reservations[(r.x, r.y, tick_counter)] = robot_name
            coop_manager.reservations[(r.x, r.y, tick_counter + 1)] = robot_name
            return True
        elif cmd in ['L', 'R']:
            # L/R chỉ thay đổi heading NỘI BỘ (cho CA* tính toán)
            # KHÔNG thay đổi visual_heading → robot KHÔNG xoay trên màn hình
            # KHÔNG set step_backlog → không có animation xoay
            if cmd == 'L':
                r.heading = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[r.heading]
            else:
                r.heading = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[r.heading]
            # step_backlog = 0: turn là tức thời, không có animation
            return True
        return True
