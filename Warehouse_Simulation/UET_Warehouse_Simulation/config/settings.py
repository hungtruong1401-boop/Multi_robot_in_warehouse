# ============================================================
# UET | NCKH 2026 | Cấu hình hệ thống Robot kho hàng (MÔ PHỎNG)
# ============================================================

# CẤU HÌNH LƯỚI
GRID_N = 11
SUB = 10

# CẤU HÌNH TỌA ĐỘ
PICKUP_NODES = [(0,0), (3,0), (6,0), (9,0), (1,10), (4,10), (7,10), (10,10)]

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
    12: [(8,3), (2,6), (8,9)],
}

DELIVERY_NODES = []
for locs in CONFIG_DELIVERY_LOCATIONS.values():
    DELIVERY_NODES.extend(locs)

REMOVE_NODES = set()
BLUE_NODES = set(PICKUP_NODES)

SKIP_COLS = {2, 5, 8}
SKIP_ROWS = {2, 5, 8}

# TIMING
LOGIC_INTERVAL_MS = 13
ANIM_MS = 1
UI_REFRESH_MS = 200

# ROBOT
MAX_ROBOTS = 72
DEFAULT_ROBOT_COUNT = 5
WAIT_TIMER_PICK = 5
WAIT_TIMER_DROP = 5
MAX_STUCK = 150

# BENCHMARK CONFIG (Tương đương 1 Giờ hoạt động thực tế với 1 tick = 0.77s)
BENCHMARK_TICKS_LIMIT = 4675
TEST_DURATION = 60
