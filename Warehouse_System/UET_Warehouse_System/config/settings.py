# ============================================================
# UET | NCKH 2026 | Cấu hình hệ thống Robot kho hàng
# ============================================================

# ============================================================
# CẤU HÌNH LƯỚI (GRID)
# ============================================================
GRID_N = 11
SUB = 10

# ============================================================
# CẤU HÌNH TỌA ĐỘ VÀ BẢN ĐỒ
# ============================================================
PICKUP_NODES = [(0,0), (3,0), (6,0), (9,0), (1,10), (4,10), (7,10), (10,10)]

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
    12: [(8,3), (2,6), (8,9)],
}

DELIVERY_NODES = []
for locs in CONFIG_DELIVERY_LOCATIONS.values():
    DELIVERY_NODES.extend(locs)

REMOVE_NODES = set()
BLUE_NODES = set(PICKUP_NODES)

# Cấu hình các đường không link
SKIP_COLS = {2, 5, 8}
SKIP_ROWS = {2, 5, 8}

# ============================================================
# CẤU HÌNH THỜI GIAN (TIMING)
# ============================================================
LOGIC_INTERVAL_MS = 50   # Tốc độ xử lý logic (ms)
ANIM_MS = 10             # Tốc độ animation (ms)
UI_REFRESH_MS = 200      # Tốc độ cập nhật UI (ms)

# ============================================================
# CẤU HÌNH ROBOT
# ============================================================
MAX_ROBOTS = 72
DEFAULT_ROBOT_COUNT = 5
WAIT_TIMER_PICK = 20     # Thời gian bốc hàng (ticks)
WAIT_TIMER_DROP = 20     # Thời gian trả hàng (ticks)
MAX_STUCK = 150          # Số tick tối đa trước khi replan
TEST_DURATION = 60       # Thời gian benchmark (giây)

# ============================================================
# CẤU HÌNH RFID — Ánh xạ UID thẻ RFID → Ô giao hàng
# 36 thẻ RFID ứng với 36 điểm giao (12 địa chỉ × 3 điểm/địa chỉ)
# Robot quét được UID → xác nhận đã giao hàng tại ô đó
# ============================================================
RFID_DELIVERY_MAP = {
    # Địa chỉ 1: (2,1), (8,4), (2,7)
    "67674E24": {"cell": 1,  "coord": (2, 1), "address": 1,  "name": "Ô giao 1A"},
    "F0131B5F": {"cell": 1,  "coord": (8, 4), "address": 1,  "name": "Ô giao 1B"},
    "67D04E24": {"cell": 1,  "coord": (2, 7), "address": 1,  "name": "Ô giao 1C"},
    # Địa chỉ 2: (1,2), (9,5), (1,8)
    "67C53425": {"cell": 2,  "coord": (1, 2), "address": 2,  "name": "Ô giao 2A"},
    "A7672249": {"cell": 2,  "coord": (9, 5), "address": 2,  "name": "Ô giao 2B"},
    "B32A1C96": {"cell": 2,  "coord": (1, 8), "address": 2,  "name": "Ô giao 2C"},
    # Địa chỉ 3: (3,2), (7,5), (3,8)
    "6378F895": {"cell": 3,  "coord": (3, 2), "address": 3,  "name": "Ô giao 3A"},
    "03D96F96": {"cell": 3,  "coord": (7, 5), "address": 3,  "name": "Ô giao 3B"},
    "E32B0E96": {"cell": 3,  "coord": (3, 8), "address": 3,  "name": "Ô giao 3C"},
    # Địa chỉ 4: (2,3), (8,6), (2,9)
    "D3CE6996": {"cell": 4, "coord": (2, 3), "address": 4,  "name": "Ô giao 4A"},
    "933FB695": {"cell": 4, "coord": (8, 6), "address": 4,  "name": "Ô giao 4B"},
    "D39745A9": {"cell": 4, "coord": (2, 9), "address": 4,  "name": "Ô giao 4C"},
    # Địa chỉ 5: (5,1), (5,4), (5,7)
    "A3F73A96": {"cell": 5, "coord": (5, 1), "address": 5,  "name": "Ô giao 5A"},
    "33F16996": {"cell": 5, "coord": (5, 4), "address": 5,  "name": "Ô giao 5B"},
    "43949AA9": {"cell": 5, "coord": (5, 7), "address": 5,  "name": "Ô giao 5C"},
    # Địa chỉ 6: (4,2), (6,5), (4,8)
    "63BD5B96": {"cell": 6, "coord": (4, 2), "address": 6,  "name": "Ô giao 6A"},
    "B3DAE995": {"cell": 6, "coord": (6, 5), "address": 6,  "name": "Ô giao 6B"},
    "03AC1596": {"cell": 6, "coord": (4, 8), "address": 6,  "name": "Ô giao 6C"},
    # Địa chỉ 7: (6,2), (4,5), (6,8)
    "73926396": {"cell": 7, "coord": (6, 2), "address": 7,  "name": "Ô giao 7A"},
    "73A0A395": {"cell": 7, "coord": (4, 5), "address": 7,  "name": "Ô giao 7B"},
    "F3F94096": {"cell": 7, "coord": (6, 8), "address": 7,  "name": "Ô giao 7C"},
    # Địa chỉ 8: (5,3), (5,6), (5,9)
    "435C2D96": {"cell": 8, "coord": (5, 3), "address": 8,  "name": "Ô giao 8A"},
    "A39BA695": {"cell": 8, "coord": (5, 6), "address": 8,  "name": "Ô giao 8B"},
    "6370A295": {"cell": 8, "coord": (5, 9), "address": 8,  "name": "Ô giao 8C"},
    # Địa chỉ 9: (8,1), (2,4), (8,7)
    "73EC9BA9": {"cell": 9, "coord": (8, 1), "address": 9,  "name": "Ô giao 9A"},
    "53966AA9": {"cell": 9, "coord": (2, 4), "address": 9,  "name": "Ô giao 9B"},
    "E34988A9": {"cell": 9, "coord": (8, 7), "address": 9,  "name": "Ô giao 9C"},
    # Địa chỉ 10: (7,2), (3,5), (7,8)
    "C3B65F96": {"cell": 10, "coord": (7, 2), "address": 10, "name": "Ô giao 10A"},
    "83410E96": {"cell": 10, "coord": (3, 5), "address": 10, "name": "Ô giao 10B"},
    "F338CA95": {"cell": 10, "coord": (7, 8), "address": 10, "name": "Ô giao 10C"},
    # Địa chỉ 11: (9,2), (1,5), (9,8)
    "43C9BF95": {"cell": 11, "coord": (9, 2), "address": 11, "name": "Ô giao 11A"},
    "D3420396": {"cell": 11, "coord": (1, 5), "address": 11, "name": "Ô giao 11B"},
    "A3E197A9": {"cell": 11, "coord": (9, 8), "address": 11, "name": "Ô giao 11C"},
    # Địa chỉ 12: (8,3), (2,6), (8,9)
    "D3708DA9": {"cell": 12, "coord": (8, 3), "address": 12, "name": "Ô giao 12A"},
    "E324BD95": {"cell": 12, "coord": (2, 6), "address": 12, "name": "Ô giao 12B"},
    "4174CB16": {"cell": 12, "coord": (8, 9), "address": 12, "name": "Ô giao 12C"},
}
