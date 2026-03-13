# ============================================================
# UET | NCKH 2026 | Entry Point — MÔ PHỎNG (Modular)
# Chạy: python main_sim.py
# ============================================================
"""
Mô phỏng hệ thống Robot kho hàng vận hành tự động.

Cấu trúc module:
    config/settings.py          - Hằng số cấu hình
    algorithms/pso_allocator.py - PSO phân bổ nhiệm vụ
    algorithms/traffic.py       - Quản lý giao thông
    algorithms/coop_astar.py    - Cooperative A*
    core/robot_state.py         - Trạng thái robot
    core/robot_proxy.py         - Thực thi lệnh mô phỏng
    core/fleet_manager.py       - Bộ não điều phối
    ui/dashboard.py             - Giao diện
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tkinter as tk
from config.settings import GRID_N, SUB, REMOVE_NODES
from algorithms.traffic import TrafficManager
from algorithms.coop_astar import CooperativePlanner
from core.robot_proxy import SimulationProxy
from core.fleet_manager import FleetManager
from ui.dashboard import WarehouseDashboard


def main():
    # 1. Algorithms
    traffic_mgr = TrafficManager(GRID_N, REMOVE_NODES)
    coop_planner = CooperativePlanner(GRID_N, traffic_mgr)

    # 2. Core
    sim_proxy = SimulationProxy()
    fleet = FleetManager(GRID_N, traffic_mgr, coop_planner, sim_proxy, sub=SUB)

    # 3. UI
    root = tk.Tk()
    dashboard = WarehouseDashboard(root, fleet)

    # 4. Run
    root.mainloop()


if __name__ == "__main__":
    main()
