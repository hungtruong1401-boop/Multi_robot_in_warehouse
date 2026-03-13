# ============================================================
# RobotState | Trạng thái Robot
# ============================================================


class RobotState:
    def __init__(self, name, color, sub=6):
        self.name = name
        self.color = color
        self.x = 0; self.y = 0; self.heading = 'N'
        self.visual_heading = 'N'  # Hướng HIỂN THỊ — chỉ đổi khi robot THẬT SỰ DI CHUYỂN
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
        self.has_planned = False
        self.stuck_counter = 0
        self.last_position = None
        self.MAX_STUCK = 150

        # Metric tracking (for benchmark)
        self.total_distance = 0       # Số lần thực hiện lệnh F (= số mét)
        self.total_wait_ticks = 0     # Số tick ở trạng thái WAIT
        self.total_stuck_events = 0   # Số lần chạm ngưỡng stuck

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
        self.total_distance = 0
        self.total_wait_ticks = 0
        self.total_stuck_events = 0
