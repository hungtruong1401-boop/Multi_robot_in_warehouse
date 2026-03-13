# ============================================================
# Warehouse Dashboard | Giao diện mô phỏng (MÔ PHỎNG)
# ============================================================
import tkinter as tk
from tkinter import ttk, messagebox
import time
import heapq

from config.settings import (
    GRID_N, SUB, PICKUP_NODES, CONFIG_DELIVERY_LOCATIONS,
    DELIVERY_NODES, REMOVE_NODES, BLUE_NODES,
    SKIP_COLS, SKIP_ROWS, ANIM_MS, LOGIC_INTERVAL_MS,
    UI_REFRESH_MS, DEFAULT_ROBOT_COUNT,
)


class WarehouseDashboard:
    def __init__(self, root, fleet):
        self.root = root
        self.fleet = fleet
        self.root.title("Mô phỏng hệ thống Robot kho hàng vận hành tự động | UET NCKH2026")
        self.root.state("zoomed")
        self.grid_n = GRID_N
        self.SUB = SUB

        # ================= GUI LAYOUT =================
        self.main_pane = tk.PanedWindow(root, orient=tk.HORIZONTAL, sashwidth=6)
        self.main_pane.pack(fill="both", expand=True)

        # MAP
        self.frame_map = tk.Frame(self.main_pane, bg="white")
        self.main_pane.add(self.frame_map, minsize=600)
        self.cell = 55
        self.MARGIN = 40
        size = (self.grid_n - 1) * self.cell + 2 * self.MARGIN
        self.canvas = tk.Canvas(self.frame_map, width=size, height=size, bg="white")
        self.canvas.pack(padx=10, pady=10)

        # Pickup Monitor
        self.frame_pickup_info = tk.LabelFrame(self.frame_map, text="Giám sát Trạm Nhận Hàng",
                                                bg="white", padx=5, pady=5)
        self.frame_pickup_info.pack(fill="x", padx=10, pady=(0, 10))
        pk_cols = ("id", "coord", "status", "delivery_coord", "assigned_robot")
        self.tree_pickup = ttk.Treeview(self.frame_pickup_info, columns=pk_cols,
                                         show="headings", height=8)
        self.tree_pickup.heading("id", text="ID Trạm")
        self.tree_pickup.heading("coord", text="Tọa độ")
        self.tree_pickup.heading("status", text="Đơn hàng (Tới Điểm trả)")
        self.tree_pickup.heading("delivery_coord", text="Tọa độ trả")
        self.tree_pickup.heading("assigned_robot", text="Robot thực hiện")
        self.tree_pickup.column("id", width=60, anchor="center")
        self.tree_pickup.column("coord", width=80, anchor="center")
        self.tree_pickup.column("status", width=120, anchor="center")
        self.tree_pickup.column("delivery_coord", width=80, anchor="center")
        self.tree_pickup.column("assigned_robot", width=100, anchor="center")
        self.tree_pickup.pack(fill="both", expand=True)

        # ĐIỀU KHIỂN
        self.frame_right = tk.Frame(self.main_pane)
        self.main_pane.add(self.frame_right, minsize=400)
        tk.Label(self.frame_right, text="Trung tâm điều phối robot | NCKH2026",
                 font=("Arial", 14, "bold"), fg="#333").pack(pady=10)
        self.lbl_timer = tk.Label(self.frame_right, text="Thời gian: 00:00:00",
                                   font=("Arial", 12, "bold"), fg="red")
        self.lbl_timer.pack(pady=2)

        # Config frame
        self.frame_config = tk.LabelFrame(self.frame_right, text="Cấu hình hệ thống", padx=5, pady=5)
        self.frame_config.pack(fill="x", padx=10, pady=5)
        f_cfg_row1 = tk.Frame(self.frame_config)
        f_cfg_row1.pack(fill="x", pady=2)
        tk.Label(f_cfg_row1, text="Số lượng Robot (1-72): ").pack(side="left")
        self.spin_count = tk.Spinbox(f_cfg_row1, from_=1, to=72, width=5)
        self.spin_count.delete(0, "end")
        self.spin_count.insert(0, DEFAULT_ROBOT_COUNT)
        self.spin_count.pack(side="left", padx=5)
        self.btn_create = tk.Button(f_cfg_row1, text="Tạo Robot", bg="#2196F3", fg="white",
                  command=self.setup_robots_ui)
        self.btn_create.pack(side="left", padx=5)

        f_btn_row = tk.Frame(self.frame_config)
        f_btn_row.pack(fill="x", pady=5)
        self.btn_start = tk.Button(f_btn_row, text="KHỞI ĐỘNG HỆ THỐNG", bg="#4CAF50",
                                    fg="white", font=("Arial", 10, "bold"), height=2,
                                    command=self.toggle_system_start)
        self.btn_start.pack(side="left", fill="x", expand=True, padx=(0, 5))
        self.btn_test_60s = tk.Button(f_btn_row, text="CHẠY TEST (60s)", bg="#FF9800",
                                       fg="white", font=("Arial", 10, "bold"), height=2,
                                       command=self.start_benchmark)
        self.btn_test_60s.pack(side="left", padx=0)

        # Status frame
        self.frame_pos = tk.LabelFrame(self.frame_right, text="Giám sát trạng thái", padx=5, pady=5)
        self.frame_pos.pack(fill="both", expand=True, padx=10, pady=5)
        self.lbl_total = tk.Label(self.frame_pos, text="Tổng hàng đã giao: 0",
                                   font=("Arial", 11, "bold"), fg="blue")
        self.lbl_total.pack(anchor="w", padx=5, pady=(0, 5))
        columns = ("name", "status", "pos", "target", "next_zone", "count")
        self.tree_monitor = ttk.Treeview(self.frame_pos, columns=columns,
                                          show="headings", height=12)
        self.tree_monitor.heading("name", text="Robot")
        self.tree_monitor.heading("status", text="Trạng thái")
        self.tree_monitor.heading("pos", text="Vị trí")
        self.tree_monitor.heading("target", text="Đích đến")
        self.tree_monitor.heading("next_zone", text="Điểm Trả")
        self.tree_monitor.heading("count", text="Đã giao")
        self.tree_monitor.column("name", width=50, anchor="center")
        self.tree_monitor.column("status", width=90, anchor="w")
        self.tree_monitor.column("pos", width=60, anchor="center")
        self.tree_monitor.column("target", width=60, anchor="center")
        self.tree_monitor.column("next_zone", width=60, anchor="center")
        self.tree_monitor.column("count", width=50, anchor="center")
        scrollbar = ttk.Scrollbar(self.frame_pos, orient=tk.VERTICAL,
                                   command=self.tree_monitor.yview)
        self.tree_monitor.configure(yscroll=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.tree_monitor.pack(fill="both", expand=True)

        # Manual control
        self.frame_manual = tk.LabelFrame(self.frame_right, text="Điều khiển thủ công",
                                            padx=10, pady=10)
        self.frame_manual.pack(fill="x", padx=10, pady=10)
        frame_sel = tk.Frame(self.frame_manual)
        frame_sel.pack(fill="x", pady=5)
        tk.Label(frame_sel, text="Chọn Robot:").pack(side="left")
        self.combo_robots = ttk.Combobox(frame_sel, values=[], state="readonly", width=10)
        self.combo_robots.pack(side="left", padx=5)
        self.btn_pause = tk.Button(frame_sel, text="Dừng/Tiếp", bg="yellow",
                                    command=self.toggle_robot_pause)
        self.btn_pause.pack(side="left", padx=5)
        frame_tgt = tk.Frame(self.frame_manual)
        frame_tgt.pack(fill="x", pady=5)
        tk.Label(frame_tgt, text="Điểm nhận (x,y):").pack(side="left")
        self.ent_x = tk.Entry(frame_tgt, width=5)
        self.ent_x.pack(side="left", padx=2)
        self.ent_y = tk.Entry(frame_tgt, width=5)
        self.ent_y.pack(side="left", padx=2)
        tk.Button(frame_tgt, text="Di chuyển", bg="#4CAF50", fg="white",
                  command=self.manual_set_target).pack(side="left", padx=5)

        # ================= INIT & TIMERS =================
        self.fleet.setup_robots(DEFAULT_ROBOT_COUNT)
        self._update_combo()
        self.draw_map()
        self.draw_robot()

        self.ANIM_MS = ANIM_MS
        self.LOGIC_MS = LOGIC_INTERVAL_MS
        self.root.after(self.ANIM_MS, self._anim_tick)
        self.root.after(self.LOGIC_MS, self._logic_tick)
        self.root.after(UI_REFRESH_MS, self._refresh_status)

    # ============================================================
    # SYSTEM CONTROL
    # ============================================================
    def setup_robots_ui(self):
        try:
            n = int(self.spin_count.get())
            if n < 1: n = 1
            if n > 72: n = 72
        except:
            n = DEFAULT_ROBOT_COUNT
        self.fleet.is_running = False
        self.fleet.is_benchmark_mode = False
        self.btn_start.config(text="KHỞI ĐỘNG HỆ THỐNG", bg="#4CAF50")
        self.fleet.setup_robots(n)
        self._update_combo()
        self.draw_map()
        self.draw_robot()

    def toggle_system_start(self):
        self.fleet.is_benchmark_mode = False
        self.fleet.is_running = not self.fleet.is_running
        if self.fleet.is_running:
            self.fleet.start_time = time.time()
            self.btn_start.config(text="ĐANG CHẠY (NHẤN ĐỂ DỪNG)", bg="#F44336")
        else:
            self.lbl_timer.config(text="Thời gian: 00:00:00")
            self.btn_start.config(text="KHỞI ĐỘNG HỆ THỐNG", bg="#4CAF50")

    def start_benchmark(self):
        for r in self.fleet.robots.values():
            r.delivered_count = 0
        self.fleet.is_benchmark_mode = True
        self.fleet.start_time = time.time()
        self.fleet.is_running = True
        self.btn_start.config(text="ĐANG TEST 60s...", bg="#FF9800")

    def toggle_robot_pause(self):
        name = self.combo_robots.get()
        if name in self.fleet.robots:
            self.fleet.robots[name].paused = not self.fleet.robots[name].paused
            self.draw_robot()

    def manual_set_target(self):
        name = self.combo_robots.get()
        try:
            tx = int(self.ent_x.get())
            ty = int(self.ent_y.get())
        except:
            messagebox.showerror("Error", "Lỗi nhập liệu")
            return
        if not (0 <= tx < self.grid_n and 0 <= ty < self.grid_n): return
        if name in self.fleet.robots:
            r = self.fleet.robots[name]
            r.manual_override = True
            r.state = "Đi thủ công"
            r.path_queue.clear()
            r.target_node = (tx, ty)
            path = self.fleet.find_path_old((r.x, r.y, r.heading), (tx, ty))
            if path:
                r.path_queue = path
                r.paused = False
            else:
                messagebox.showwarning("Warning", "Không tìm thấy đường")

    def _update_combo(self):
        self.combo_robots['values'] = list(self.fleet.robots.keys())
        if self.fleet.robots:
            self.combo_robots.current(0)

    # ============================================================
    # VẼ MAP
    # ============================================================
    def draw_map(self):
        self.canvas.delete("all")
        off = self.MARGIN
        for x in range(self.grid_n):
            if x not in SKIP_COLS:
                cx = off + x * self.cell
                self.canvas.create_line(cx, off, cx, off + (self.grid_n - 1) * self.cell,
                                         width=2, fill="#e0e0e0")
                if x in self.fleet.traffic_mgr.COLS_DOWN:
                    self.canvas.create_text(cx, off - 15, text="↓", fill="gray",
                                             font=("Arial", 10, "bold"))
                elif x in self.fleet.traffic_mgr.COLS_UP:
                    self.canvas.create_text(cx, off + (self.grid_n - 1) * self.cell + 15,
                                             text="↑", fill="gray", font=("Arial", 10, "bold"))
        for y in range(self.grid_n):
            if y not in SKIP_ROWS:
                cy = off + (self.grid_n - 1 - y) * self.cell
                self.canvas.create_line(off, cy, off + (self.grid_n - 1) * self.cell, cy,
                                         width=2, fill="#e0e0e0")
                if y in self.fleet.traffic_mgr.ROWS_LEFT:
                    self.canvas.create_text(off + (self.grid_n - 1) * self.cell + 15, cy,
                                             text="←", fill="gray", font=("Arial", 10, "bold"))
                elif y in self.fleet.traffic_mgr.ROWS_RIGHT:
                    self.canvas.create_text(off - 15, cy, text="→", fill="gray",
                                             font=("Arial", 10, "bold"))
        # Delivery nodes
        for l_id, coords in CONFIG_DELIVERY_LOCATIONS.items():
            for (dx, dy) in coords:
                cx = off + dx * self.cell
                cy = off + (self.grid_n - 1 - dy) * self.cell
                self.canvas.create_rectangle(cx-7, cy-7, cx+7, cy+7,
                                              fill="#AAF4B7", outline="green", width=1)
                self.canvas.create_text(cx, cy, text=str(l_id), font=("Arial", 7), fill="black")
        # Pickup nodes
        rr = 7
        for x in range(self.grid_n):
            for y in range(self.grid_n):
                cx = off + x * self.cell
                cy = off + (self.grid_n - 1 - y) * self.cell
                if (x, y) in BLUE_NODES:
                    self.canvas.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                                             fill="#4287f5", outline="blue")
                    try:
                        idx = PICKUP_NODES.index((x, y)) + 1
                        self.canvas.create_text(cx, cy, text=str(idx),
                                                 font=("Arial", 7, "bold"), fill="white")
                    except:
                        pass
                elif (x, y) not in DELIVERY_NODES and (x not in SKIP_COLS or y not in SKIP_ROWS):
                    self.canvas.create_oval(cx - 2, cy - 2, cx + 2, cy + 2, fill="black")

    def draw_robot(self):
        for name in self.fleet.robots:
            self.canvas.delete(f"robot_{name}")
        for name, r in self.fleet.robots.items():
            x = r.sx / r.SUB
            y = r.sy / r.SUB
            cx = self.MARGIN + x * self.cell
            cy = self.MARGIN + (self.grid_n - 1 - y) * self.cell
            size = 14
            outline_w = 0; outline_color = "black"; stipple = ""
            if r.state in ["PICKING", "DROPPING"]:
                outline_color = "#41CEC4"
                outline_w = 4
            if r.paused:
                outline_w = 4; stipple = "gray50"; outline_color = "black"
            self.canvas.create_rectangle(cx - size, cy - size, cx + size, cy + size,
                                          fill=r.color, outline=outline_color,
                                          width=outline_w, stipple=stipple,
                                          tags=f"robot_{name}")
            if r.state in ["TO_DELIVERY", "DROPPING"]:
                cargo_sz = 9
                self.canvas.create_rectangle(cx - cargo_sz, cy - cargo_sz,
                                              cx + cargo_sz, cy + cargo_sz,
                                              fill="#E1720A", outline="black",
                                              tags=f"robot_{name}")
            short_name = name.replace("ESP", "")
            self.canvas.create_text(cx, cy, text=short_name, font=("Arial", 9, "bold"),
                                     fill="white", tags=f"robot_{name}")
            tri = {
                'N': [cx, cy-size-8, cx-6, cy-size, cx+6, cy-size],
                'E': [cx+size+8, cy, cx+size, cy-6, cx+size, cy+6],
                'S': [cx, cy+size+8, cx-6, cy+size, cx+6, cy+size],
                'W': [cx-size-8, cy, cx-size, cy-6, cx-size, cy+6],
            }
            self.canvas.create_polygon(tri[r.visual_heading], fill="blue", outline="blue",
                                        tags=f"robot_{name}")

    # ============================================================
    # ANIMATION
    # ============================================================
    def _anim_tick(self):
        any_change = False
        for name, r in self.fleet.robots.items():
            target_sx = r.x * r.SUB
            target_sy = r.y * r.SUB
            if r.sx != target_sx or r.sy != target_sy:
                diff_x = target_sx - r.sx
                diff_y = target_sy - r.sy
                
                step_x = max(1, abs(diff_x) // 3) if diff_x != 0 else 0
                step_y = max(1, abs(diff_y) // 3) if diff_y != 0 else 0
                
                if diff_x > 0: r.sx = min(target_sx, r.sx + step_x)
                elif diff_x < 0: r.sx = max(target_sx, r.sx - step_x)
                
                if diff_y > 0: r.sy = min(target_sy, r.sy + step_y)
                elif diff_y < 0: r.sy = max(target_sy, r.sy - step_y)
                
                any_change = True
                
        if any_change:
            self.draw_robot()
        self.root.after(self.ANIM_MS, self._anim_tick)

    # ============================================================
    # LOGIC TICK
    # ============================================================
    def _logic_tick(self):
        if not self.fleet.is_running:
            self.root.after(self.LOGIC_MS, self._logic_tick)
            return

        # Timer
        if self.fleet.is_benchmark_mode:
            elapsed = time.time() - self.fleet.start_time
            remaining = max(0, self.fleet.TEST_DURATION - elapsed)
            self.lbl_timer.config(text=f"Thời gian: {int(remaining)}s / {self.fleet.TEST_DURATION}s")
        else:
            elapsed = time.time() - self.fleet.start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.lbl_timer.config(text=f"Thời gian: {hours:02d}:{minutes:02d}:{seconds:02d}")

        result = self.fleet.update_tick()

        if result and result.get('event') == 'benchmark_done':
            self.btn_start.config(text="KHỞI ĐỘNG HỆ THỐNG", bg="#4CAF50")
            self.lbl_timer.config(text="KẾT THÚC TEST 60s!")
            messagebox.showinfo("KẾT QUẢ TEST 60s",
                                f"TỔNG HÀNG ĐÃ GIAO: {result['total']}")

        self.root.after(self.LOGIC_MS, self._logic_tick)

    # ============================================================
    # STATUS REFRESH
    # ============================================================
    def _refresh_status(self):
        for item in self.tree_monitor.get_children():
            self.tree_monitor.delete(item)
        STATE_MAP = {
            "IDLE": "Chờ lệnh", "TO_PICKUP": "Đi lấy hàng",
            "PICKING": "Đang bốc...", "TO_DELIVERY": "Đi giao hàng",
            "DROPPING": "Đang trả...", "MANUAL_MOVE": "Thủ công"
        }
        total = 0
        active_pickup_map = {}
        for name in sorted(self.fleet.robots.keys()):
            r = self.fleet.robots[name]
            st = STATE_MAP.get(r.state, r.state)
            if r.paused: st = "TẠM DỪNG"
            elif r.wait_timer > 0: st += f" ({r.wait_timer})"
            total += r.delivered_count
            pos = f"({r.x},{r.y})"
            tgt = f"({r.target_node[0]},{r.target_node[1]})" if r.target_node else "--"
            zone = f"Điểm trả {r.future_delivery_logical_id}" if r.future_delivery_logical_id else "--"
            self.tree_monitor.insert("", "end",
                                      values=(name, st, pos, tgt, zone, r.delivered_count))
            if r.current_job_pickup:
                active_pickup_map[r.current_job_pickup] = {
                    'robot': name,
                    'zone': f"Điểm trả {r.future_delivery_logical_id}" if r.future_delivery_logical_id else "?",
                    'delivery_coord': str(r.target_node) if r.state in ["TO_DELIVERY", "DROPPING"] and r.target_node else "--"
                }
        self.lbl_total.config(text=f"Tổng hàng đã giao: {total}")

        # Pickup table
        for item in self.tree_pickup.get_children():
            self.tree_pickup.delete(item)
        for i, node in enumerate(PICKUP_NODES):
            data = active_pickup_map.get(node)
            self.tree_pickup.insert("", "end", values=(
                f"Trạm {i+1}", str(node),
                data['zone'] if data else "Đợi hàng",
                data['delivery_coord'] if data else "--",
                data['robot'] if data else "Đang trống"
            ))

        self.root.after(UI_REFRESH_MS, self._refresh_status)
