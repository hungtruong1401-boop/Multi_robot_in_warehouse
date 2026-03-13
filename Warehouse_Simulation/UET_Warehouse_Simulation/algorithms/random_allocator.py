# ============================================================
# Random Task Allocator — Phân nhiệm ngẫu nhiên
# Dùng cho tổ hợp: BFS + Random Assignment
# ============================================================
import random

from config.settings import PICKUP_NODES, CONFIG_DELIVERY_LOCATIONS


class RandomTaskAllocator:
    """Phân nhiệm ngẫu nhiên — cùng interface với PSO_TaskAllocator.
    Random chọn pickup point + logical delivery ID."""

    def __init__(self, current_pos, current_heading, app_instance, caller_name=None):
        self.app = app_instance
        self.start_pos = current_pos
        self.caller_name = caller_name
        self.pickup_options = list(PICKUP_NODES)

    def run(self):
        """Chọn ngẫu nhiên pickup + logical_id, tránh pickup đang bị chiếm."""
        # Lọc bỏ pickup đang bận (robot khác đang chiếm hoặc đang hướng tới)
        available = []
        for p in self.pickup_options:
            occupied = False
            for rname, r in self.app.robots.items():
                if rname == self.caller_name:
                    continue
                if (r.x, r.y) == p:
                    occupied = True
                    break
                if r.target_node == p and r.state in ["TO_PICKUP", "PICKING"]:
                    occupied = True
                    break
            if not occupied:
                available.append(p)

        # Nếu tất cả đều bận → chọn ngẫu nhiên từ toàn bộ
        if not available:
            available = list(self.pickup_options)

        pickup = random.choice(available)
        logical_id = self.app.pickup_orders.get(pickup, 1)
        return {'pickup': pickup, 'logical_id': logical_id}
