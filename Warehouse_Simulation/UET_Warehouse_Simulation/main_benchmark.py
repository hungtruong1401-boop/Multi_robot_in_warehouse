# ============================================================
# UET | NCKH 2026 | Entry Point — BENCHMARK (Đánh giá thuật toán)
# Chạy: python main_benchmark.py
# ============================================================
"""
Benchmark hệ thống Robot kho hàng — Đánh giá hiệu suất thuật toán.

Tính năng:
    1. Chọn 1 trong 3 tổ hợp thuật toán TRONG GIAO DIỆN:
       - Option 1: CA* + PSO extended (tối ưu nhất)
       - Option 2: A* + PSO
       - Option 3: BFS + Random Assignment
    2. Vị trí spawn robot cố định (deterministic)
    3. Xuất kết quả benchmark ra CSV
"""
import sys
import os
import csv
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tkinter as tk
from tkinter import ttk, messagebox

from config.settings import GRID_N, SUB, REMOVE_NODES
from algorithms.traffic import TrafficManager
from core.robot_proxy import SimulationProxy
from core.fleet_manager import FleetManager


# ============================================================
# Cấu hình thuật toán
# ============================================================
ALGORITHM_OPTIONS = {
    "CA* + PSO Extended": {
        "mode": "ca_pso",
        "planner_factory": lambda traffic: __import__('algorithms.coop_astar', fromlist=['CooperativePlanner']).CooperativePlanner(GRID_N, traffic),
        "allocator_class": lambda: __import__('algorithms.pso_allocator', fromlist=['PSO_TaskAllocator']).PSO_TaskAllocator,
    },
    "A* + PSO": {
        "mode": "a_pso",
        "planner_factory": lambda traffic: __import__('algorithms.astar_planner', fromlist=['AStarPlanner']).AStarPlanner(GRID_N, traffic),
        "allocator_class": lambda: __import__('algorithms.pso_basic_allocator', fromlist=['PSO_BasicAllocator']).PSO_BasicAllocator,
    },
    "A* + Hungarian": {
        "mode": "a_hungarian",
        "planner_factory": lambda traffic: __import__('algorithms.astar_planner', fromlist=['AStarPlanner']).AStarPlanner(GRID_N, traffic),
        "allocator_class": lambda: __import__('algorithms.hungarian_allocator', fromlist=['HungarianAllocator']).HungarianAllocator,
    },
    "CA* + Hungarian": {
        "mode": "ca_hungarian",
        "planner_factory": lambda traffic: __import__('algorithms.coop_astar', fromlist=['CooperativePlanner']).CooperativePlanner(GRID_N, traffic),
        "allocator_class": lambda: __import__('algorithms.hungarian_allocator', fromlist=['HungarianAllocator']).HungarianAllocator,
    },
    "BFS + Random": {
        "mode": "bfs_random",
        "planner_factory": lambda traffic: __import__('algorithms.bfs_planner', fromlist=['BFSPlanner']).BFSPlanner(GRID_N, traffic),
        "allocator_class": lambda: __import__('algorithms.random_allocator', fromlist=['RandomTaskAllocator']).RandomTaskAllocator,
    },
}


# ============================================================
# Ghi kết quả CSV
# ============================================================
def save_benchmark_csv(algorithm_label, fleet, cpu_time):
    """Append kết quả benchmark đầy đủ metrics vào file CSV.

    Metrics:
        - UPH (Throughput = tổng delivered trong 4675 ticks = 1 giờ mô phỏng)
        - CPU Time (thời gian thực tế máy tính xử lý)
        - Distance stats (Total, Max, Min, StdDev)
        - Wait Ratio (% tổng thời gian chờ)
        - Stuck Events (số lần deadlock)
        - Per-robot delivered breakdown
    """
    import math
    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "benchmark_data.csv")
    file_exists = os.path.isfile(csv_path)

    robots = list(fleet.robots.values())
    num_robots = len(robots)
    total_ticks = fleet.tick_counter

    # Throughput
    total_delivered = sum(r.delivered_count for r in robots)

    # Distance stats
    distances = [r.total_distance for r in robots]
    total_distance = sum(distances)
    max_distance = max(distances) if distances else 0
    min_distance = min(distances) if distances else 0
    mean_dist = total_distance / num_robots if num_robots > 0 else 0
    variance = sum((d - mean_dist) ** 2 for d in distances) / num_robots if num_robots > 0 else 0
    stddev_distance = math.sqrt(variance)

    # Wait ratio
    total_wait = sum(r.total_wait_ticks for r in robots)
    total_possible_ticks = total_ticks * num_robots
    wait_ratio_pct = (total_wait / total_possible_ticks * 100) if total_possible_ticks > 0 else 0

    # Stuck events
    total_stuck = sum(r.total_stuck_events for r in robots)

    # Per-robot delivered (compact string)
    per_robot = "|".join(f"{r.name}:{r.delivered_count}" for r in robots)

    with open(csv_path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow([
                "Timestamp", "Algorithm", "Num_Robots", "Total_Ticks",
                "UPH", "CPU_Time_s",
                "Total_Distance_m", "Max_Distance_m", "Min_Distance_m", "StdDev_Distance",
                "Total_Wait_Ticks", "Wait_Ratio_Pct",
                "Total_Stuck_Events",
                "Per_Robot_Delivered"
            ])
        writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            algorithm_label,
            num_robots,
            total_ticks,
            total_delivered,
            f"{cpu_time:.2f}",
            total_distance,
            max_distance,
            min_distance,
            f"{stddev_distance:.2f}",
            total_wait,
            f"{wait_ratio_pct:.2f}",
            total_stuck,
            per_robot
        ])
    return csv_path


# ============================================================
# Dashboard mở rộng cho Benchmark
# ============================================================
class BenchmarkDashboard:
    """Dashboard mở rộng từ WarehouseDashboard, thêm:
    - Dropdown chọn thuật toán trong giao diện
    - Deterministic spawning
    - CSV output khi benchmark kết thúc"""

    def __init__(self, root):
        from ui.dashboard import WarehouseDashboard

        self.root = root
        self.traffic_mgr = TrafficManager(GRID_N, REMOVE_NODES)
        self.sim_proxy = SimulationProxy()

        # Khởi tạo với thuật toán mặc định (CA* + PSO)
        self.current_algo_label = "CA* + PSO Extended"
        self._build_fleet(self.current_algo_label)

        # Tạo dashboard gốc
        self.dashboard = WarehouseDashboard(root, self.fleet)

        # Cập nhật tiêu đề
        root.title("BENCHMARK — Danh gia hieu suat thuat toan | UET NCKH2026")

        # === Thêm khung chọn thuật toán vào GUI ===
        self.frame_algo = tk.LabelFrame(
            self.dashboard.frame_config,
            text="Chon to hop thuat toan",
            padx=5, pady=5
        )
        self.frame_algo.pack(fill="x", pady=(5, 0), before=self.dashboard.frame_config.winfo_children()[0])

        algo_names = list(ALGORITHM_OPTIONS.keys())
        self.combo_algo = ttk.Combobox(
            self.frame_algo, values=algo_names,
            state="readonly", width=30,
            font=("Arial", 10)
        )
        self.combo_algo.set(self.current_algo_label)
        self.combo_algo.pack(side="left", padx=5, pady=3)

        self.lbl_algo_status = tk.Label(
            self.frame_algo,
            text="[ Dang dung ]",
            font=("Arial", 9, "bold"),
            fg="#2E7D32"
        )
        self.lbl_algo_status.pack(side="left", padx=5)

        # Bind sự kiện chọn thuật toán
        self.combo_algo.bind("<<ComboboxSelected>>", self._on_algo_changed)

        # === Reconfigure nút "Tạo Robot" ===
        self.dashboard.btn_create.config(command=self._setup_robots_deterministic_ui)

        # === Ghi đè _logic_tick ===
        self.dashboard._logic_tick = self._benchmark_logic_tick

        # Khởi tạo robot deterministic (ghi đè random spawn ban đầu)
        self._setup_robots_deterministic_ui()

    def _build_fleet(self, algo_label):
        """Tạo FleetManager với thuật toán được chọn."""
        cfg = ALGORITHM_OPTIONS[algo_label]
        self.traffic_mgr = TrafficManager(GRID_N, REMOVE_NODES)
        planner = cfg["planner_factory"](self.traffic_mgr)
        allocator_cls = cfg["allocator_class"]()
        mode = cfg["mode"]

        self.fleet = FleetManager(
            GRID_N, self.traffic_mgr, planner, self.sim_proxy, sub=SUB,
            algorithm_mode=mode, allocator_class=allocator_cls
        )

    def _on_algo_changed(self, event=None):
        """Xử lý khi người dùng chọn thuật toán khác."""
        new_algo = self.combo_algo.get()
        if new_algo == self.current_algo_label:
            return

        # Dừng hệ thống nếu đang chạy
        self.fleet.is_running = False
        self.fleet.is_benchmark_mode = False

        # Lưu số robot hiện tại
        try:
            n = int(self.dashboard.spin_count.get())
            if n < 1: n = 1
            if n > 72: n = 72
        except:
            from config.settings import DEFAULT_ROBOT_COUNT
            n = DEFAULT_ROBOT_COUNT

        # Tạo fleet mới với thuật toán mới
        self.current_algo_label = new_algo
        self._build_fleet(new_algo)

        # Cập nhật reference trong dashboard
        self.dashboard.fleet = self.fleet

        # Setup robot deterministic
        self.fleet.setup_robots_deterministic(n)

        # Cập nhật giao diện
        self.dashboard.btn_start.config(text="KHOI DONG HE THONG", bg="#4CAF50")
        self.lbl_algo_status.config(text="[ Dang dung ]", fg="#2E7D32")
        self.dashboard._update_combo()
        self.dashboard.draw_map()
        self.dashboard.draw_robot()

        self.root.title(f"BENCHMARK — {new_algo} | UET NCKH2026")

    def _setup_robots_deterministic_ui(self):
        """Setup robots dùng vị trí cố định thay vì random."""
        try:
            n = int(self.dashboard.spin_count.get())
            if n < 1: n = 1
            if n > 72: n = 72
        except:
            from config.settings import DEFAULT_ROBOT_COUNT
            n = DEFAULT_ROBOT_COUNT
        self.fleet.is_running = False
        self.fleet.is_benchmark_mode = False
        self.dashboard.btn_start.config(text="KHOI DONG HE THONG", bg="#4CAF50")
        self.fleet.setup_robots_deterministic(n)
        self.dashboard._update_combo()
        self.dashboard.draw_map()
        self.dashboard.draw_robot()

    def _benchmark_logic_tick(self):
        """Logic tick mở rộng — tự động lưu CSV khi benchmark kết thúc."""
        if not self.fleet.is_running:
            self.root.after(self.dashboard.LOGIC_MS, self._benchmark_logic_tick)
            return

        # Timer
        if self.fleet.is_benchmark_mode:
            from config.settings import BENCHMARK_TICKS_LIMIT
            progress = (self.fleet.tick_counter / BENCHMARK_TICKS_LIMIT) * 100
            self.dashboard.lbl_timer.config(
                text=f"Tien do: {self.fleet.tick_counter}/{BENCHMARK_TICKS_LIMIT} Ticks ({progress:.1f}%)")
            self.lbl_algo_status.config(text="[ DANG TEST... ]", fg="#E65100")
        else:
            elapsed = time.time() - self.fleet.start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.dashboard.lbl_timer.config(
                text=f"Thoi gian: {hours:02d}:{minutes:02d}:{seconds:02d}")
            self.lbl_algo_status.config(text="[ Dang chay ]", fg="#1565C0")

        result = self.fleet.update_tick()

        if result and result.get('event') == 'benchmark_done':
            self.dashboard.btn_start.config(text="KHOI DONG HE THONG", bg="#4CAF50")
            self.dashboard.lbl_timer.config(text="KET THUC TEST!")
            self.lbl_algo_status.config(text="[ Hoan thanh ]", fg="#2E7D32")

            # Tính CPU time
            cpu_time = result['elapsed']
            num_robots = len(self.fleet.robots)
            throughput = result['total']

            # Tính metrics nhanh cho popup
            import math
            robots = list(self.fleet.robots.values())
            distances = [r.total_distance for r in robots]
            total_wait = sum(r.total_wait_ticks for r in robots)
            total_possible = self.fleet.tick_counter * num_robots
            wait_pct = (total_wait / total_possible * 100) if total_possible > 0 else 0
            total_stuck = sum(r.total_stuck_events for r in robots)

            # Lưu CSV
            csv_path = save_benchmark_csv(self.current_algo_label, self.fleet, cpu_time)

            messagebox.showinfo(
                "KET QUA BENCHMARK",
                f"Thuat toan: {self.current_algo_label}\n"
                f"So robot: {num_robots}\n"
                f"Thong luong (UPH): {throughput} don/gio\n"
                f"CPU Time: {cpu_time:.1f}s\n"
                f"Tong quang duong: {sum(distances)}m\n"
                f"Max/Min distance: {max(distances)}/{min(distances)}m\n"
                f"Ti le cho: {wait_pct:.1f}%\n"
                f"Deadlock events: {total_stuck}\n\n"
                f"Da luu CSV: {csv_path}"
            )

        self.root.after(self.dashboard.LOGIC_MS, self._benchmark_logic_tick)


# ============================================================
# MAIN
# ============================================================
def main():
    root = tk.Tk()
    benchmark_dash = BenchmarkDashboard(root)
    root.mainloop()


if __name__ == "__main__":
    main()
