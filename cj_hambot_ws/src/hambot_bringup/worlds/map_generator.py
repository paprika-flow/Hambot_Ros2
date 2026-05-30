import math
import tkinter as tk
import xml.etree.ElementTree as ET
from pathlib import Path
from tkinter import filedialog, messagebox, ttk


class ScrollableFrame(ttk.Frame):
    """A scrollable frame container utilizing a Canvas and Scrollbar"""

    def __init__(self, container, *args, **kwargs):
        super().__init__(container, *args, **kwargs)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0, width=320)
        self.scrollbar = ttk.Scrollbar(
            self, orient="vertical", command=self.canvas.yview
        )
        self.scrollable_frame = ttk.Frame(self.canvas, padding="5")

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")),
        )

        self.canvas_window = self.canvas.create_window(
            (0, 0), window=self.scrollable_frame, anchor="nw"
        )
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.bind(
            "<Configure>",
            lambda e: self.canvas.itemconfig(self.canvas_window, width=e.width),
        )

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")


class CollapsiblePane(ttk.Frame):
    """A collapsible frame container (Accordion style)"""

    def __init__(self, parent, text="", *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.columnconfigure(0, weight=1)
        self._show = True
        self.text = text

        self.header = ttk.Frame(self)
        self.header.grid(row=0, column=0, sticky="ew", pady=2)

        self.toggle_btn = ttk.Button(
            self.header, text="▼  " + self.text, command=self.toggle, style="Toolbutton"
        )
        self.toggle_btn.pack(side="left", fill="x", expand=True)

        self.content = ttk.LabelFrame(self, padding=(10, 8))
        self.content.grid(row=1, column=0, sticky="nsew", padx=2, pady=(0, 10))

    def toggle(self):
        if self._show:
            self.content.grid_remove()
            self.toggle_btn.config(text="▶  " + self.text)
            self._show = False
        else:
            self.content.grid()
            self.toggle_btn.config(text="▼  " + self.text)
            self._show = True


class SidewalkMapGenerator:
    def __init__(self, root):
        self.root = root
        self.root.title("Modular Gazebo Sidewalk Map Generator")
        self.root.geometry("1200x800")

        # Map data state
        # Each segment:
        # - Sidewalk:    {'type': 'sidewalk', 'x': x, 'y': y, 'length': l, 'yaw': yaw, 'width': w, 'height': h}
        # - Grass:       {'type': 'grass', 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2}
        # - Spawn Point: {'type': 'spawn_point', 'x': x, 'y': y, 'yaw': yaw, 'id': id}
        self.segments = []
        self.selected_idx = -1  # Index of active selection (-1 = none)

        # Interactive drawing and drag/resize state
        self.scale = 25.0  # Pixels per meter (Zoom level)
        self.center_x = 450.0
        self.center_y = 375.0

        self.drag_mode = None  # "draw", "move", "resize_length", "resize_width", "move_grass", "resize_grass_corner", "pan", "move_spawn", "rotate_spawn"
        self.has_dragged = False
        self.space_held = False
        self.press_cx = 0
        self.press_cy = 0
        self.pan_start_x = 0
        self.pan_start_y = 0

        self.drag_offset_x = 0.0
        self.drag_offset_y = 0.0
        self.drag_offset_x1 = 0.0
        self.drag_offset_y1 = 0.0
        self.drag_offset_x2 = 0.0
        self.drag_offset_y2 = 0.0
        self.active_corner_keys = None

        self.start_wx = 0.0
        self.start_wy = 0.0

        # Drag redraw throttle
        self._redraw_timer = None
        self._drag_redraw_count = 0
        self._last_grid_center = None
        self._last_grid_scale = None

        self.setup_ui()
        self.update_button_states()
        self.update_editor_labels()

    def setup_ui(self):
        # 1. Top Bar Menu Dashboard
        self.top_bar = ttk.Frame(
            self.root, padding="8", relief=tk.RAISED, borderwidth=1
        )
        self.top_bar.pack(side=tk.TOP, fill=tk.X)

        title_lbl = ttk.Label(
            self.top_bar, text="🗺️ Gazebo Map Builder", font=("Arial", 11, "bold")
        )
        title_lbl.pack(side=tk.LEFT, padx=(10, 20))

        ttk.Button(self.top_bar, text="↩ Undo Last", command=self.undo_last).pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(self.top_bar, text="🗑️ Clear Map", command=self.clear_map).pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(
            self.top_bar, text="📂 Import SDF World", command=self.import_sdf
        ).pack(side=tk.LEFT, padx=5)
        ttk.Button(
            self.top_bar, text="💾 Export SDF World", command=self.export_sdf
        ).pack(side=tk.LEFT, padx=5)

        # 2. Left Sidebar (Scrollable Container)
        self.sidebar_container = ttk.Frame(self.root, width=340)
        self.sidebar_container.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.sidebar_container.pack_propagate(False)

        self.sidebar = ScrollableFrame(self.sidebar_container)
        self.sidebar.pack(fill=tk.BOTH, expand=True)

        # Scroll bindings
        self.sidebar.canvas.bind("<Enter>", self._bind_mousewheel)
        self.sidebar.canvas.bind("<Leave>", self._unbind_mousewheel)

        # 3. Right Canvas
        self.canvas_frame = ttk.Frame(self.root)
        self.canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(self.canvas_frame, bg="#2e2e2e", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Bind key pan triggers
        self.root.bind("<KeyPress-space>", self.on_space_press)
        self.root.bind("<KeyRelease-space>", self.on_space_release)

        # Bind canvas actions
        self.canvas.bind("<Configure>", self.on_resize)
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_press)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_release)

        # Bind canvas zoom actions
        self.canvas.bind("<MouseWheel>", self.on_canvas_wheel)
        self.canvas.bind("<Button-4>", self.on_canvas_wheel)
        self.canvas.bind("<Button-5>", self.on_canvas_wheel)

        # --------------------------------------------------
        # PANEL 1: Global Settings
        # --------------------------------------------------
        self.pane_global = CollapsiblePane(
            self.sidebar.scrollable_frame, text="Global Settings"
        )
        self.pane_global.pack(fill=tk.X, expand=True)
        global_content = self.pane_global.content

        # Active Tool Selector (Stacked vertically to prevent horizontal sidebar clipping)
        ttk.Label(
            global_content, text="Active Drawing Tool:", font=("Arial", 10, "bold")
        ).pack(anchor=tk.W, pady=(0, 5))
        self.tool_var = tk.StringVar(value="sidewalk")
        self.tool_var.trace_add("write", lambda *args: self.on_tool_change())

        tool_frame = ttk.Frame(global_content)
        tool_frame.pack(fill=tk.X, pady=(2, 10))
        ttk.Radiobutton(
            tool_frame, text="Sidewalk Path", variable=self.tool_var, value="sidewalk"
        ).pack(anchor=tk.W, pady=2)
        ttk.Radiobutton(
            tool_frame, text="Grass Area", variable=self.tool_var, value="grass"
        ).pack(anchor=tk.W, pady=2)
        ttk.Radiobutton(
            tool_frame, text="Spawn Point", variable=self.tool_var, value="spawn_point"
        ).pack(anchor=tk.W, pady=2)

        self.snap_var = tk.BooleanVar(value=True)
        self.snap_check = ttk.Checkbutton(
            global_content, text="Snap to Grid", variable=self.snap_var
        )
        self.snap_check.pack(anchor=tk.W, pady=2)

        # Toggle UI Info Labels
        self.show_labels_var = tk.BooleanVar(value=True)
        self.lbl_toggle_check = ttk.Checkbutton(
            global_content,
            text="Show Segment Labels",
            variable=self.show_labels_var,
            command=self.redraw,
        )
        self.lbl_toggle_check.pack(anchor=tk.W, pady=5)

        ttk.Label(global_content, text="Default Draw Width (m):").pack(anchor=tk.W)
        self.default_width_entry = ttk.Entry(global_content)
        self.default_width_entry.insert(0, "1.2")
        self.default_width_entry.pack(fill=tk.X, pady=(0, 5))

        ttk.Label(global_content, text="Default Draw Height (m):").pack(anchor=tk.W)
        self.default_height_entry = ttk.Entry(global_content)
        self.default_height_entry.insert(0, "0.05")
        self.default_height_entry.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(global_content, text="Manual Zoom Controls:").pack(anchor=tk.W)
        zoom_frame = ttk.Frame(global_content)
        zoom_frame.pack(fill=tk.X, pady=5)
        ttk.Button(zoom_frame, text="Zoom In (+)", command=self.zoom_in).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        ttk.Button(zoom_frame, text="Zoom Out (-)", command=self.zoom_out).pack(
            side=tk.RIGHT, expand=True, fill=tk.X, padx=2
        )

        # --------------------------------------------------
        # PANEL 2: Precise Placement & Editing
        # --------------------------------------------------
        self.pane_edit = CollapsiblePane(
            self.sidebar.scrollable_frame, text="Precise Editor"
        )
        self.pane_edit.pack(fill=tk.X, expand=True)
        self.edit_content = self.pane_edit.content

        self.sel_status_label = ttk.Label(
            self.edit_content, text="No segment selected", font=("Arial", 10, "italic")
        )
        self.sel_status_label.pack(anchor=tk.W, pady=(0, 10))

        self.lbl_x = ttk.Label(self.edit_content, text="Start X (m):")
        self.precise_x = ttk.Entry(self.edit_content)

        self.lbl_y = ttk.Label(self.edit_content, text="Start Y (m):")
        self.precise_y = ttk.Entry(self.edit_content)

        self.lbl_len = ttk.Label(self.edit_content, text="Length (m):")
        self.precise_len = ttk.Entry(self.edit_content)

        self.lbl_ang = ttk.Label(self.edit_content, text="Angle (degrees):")
        self.precise_ang = ttk.Entry(self.edit_content)

        self.lbl_width = ttk.Label(self.edit_content, text="Segment Width (m):")
        self.precise_width = ttk.Entry(self.edit_content)

        self.lbl_height = ttk.Label(self.edit_content, text="Segment Height/Lift (m):")
        self.precise_height = ttk.Entry(self.edit_content)

        self.btn_add_new = ttk.Button(
            self.edit_content,
            text="Add as New Segment",
            command=self.add_precise_segment,
        )
        self.btn_update = ttk.Button(
            self.edit_content,
            text="Apply Changes to Selected",
            command=self.update_selected_segment,
        )
        self.btn_delete = ttk.Button(
            self.edit_content,
            text="Delete Selected",
            command=self.delete_selected_segment,
        )
        self.btn_deselect = ttk.Button(
            self.edit_content, text="Deselect", command=self.deselect_segment
        )

        # --------------------------------------------------
        # PANEL 3: Help Guide
        # --------------------------------------------------
        self.pane_actions = CollapsiblePane(
            self.sidebar.scrollable_frame, text="Help Guide"
        )
        self.pane_actions.pack(fill=tk.X, expand=True)
        actions_content = self.pane_actions.content

        guide_text = (
            "Interactive Controls:\n"
            "- Space + Drag Mouse: Pan view.\n"
            "- Scroll Wheel: Zoom to cursor.\n"
            "- Drag Segment Body: Translate.\n"
            "- Drag Blue Square: Length & yaw.\n"
            "- Drag Pink Square: Width.\n"
            "- Drag Cyan Square: Grass corners.\n"
            "- Drag Spawn Point: Move & rotate.\n"
            "- Red Axis: X direction.\n"
            "- Green Axis: Y direction."
        )
        ttk.Label(
            actions_content, text=guide_text, justify=tk.LEFT, foreground="gray"
        ).pack(anchor=tk.W, pady=5)

    # --------------------------------------------------
    # Keyboard Navigation Listeners
    # --------------------------------------------------
    def on_space_press(self, event):
        if not self.space_held:
            self.space_held = True
            self.canvas.config(cursor="fleur")

    def on_space_release(self, event):
        self.space_held = False
        self.canvas.config(cursor="")

    # --------------------------------------------------
    # Sidebar Scrolling Utilities
    # --------------------------------------------------
    def _bind_mousewheel(self, event):
        self.sidebar.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.sidebar.canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.sidebar.canvas.bind_all("<Button-5>", self._on_mousewheel)

    def _unbind_mousewheel(self, event):
        self.sidebar.canvas.unbind_all("<MouseWheel>")
        self.sidebar.canvas.unbind_all("<Button-4>")
        self.sidebar.canvas.unbind_all("<Button-5>")

    def _on_mousewheel(self, event):
        if event.num == 4:
            self.sidebar.canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self.sidebar.canvas.yview_scroll(1, "units")
        else:
            self.sidebar.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    # --------------------------------------------------
    # Canvas Zoom-to-Pointer Wheel Handler
    # --------------------------------------------------
    def on_canvas_wheel(self, event):
        mx, my = self.to_world(event.x, event.y)

        if event.num == 4 or event.delta > 0:
            zoom_factor = 1.15
        else:
            zoom_factor = 0.85

        new_scale = self.scale * zoom_factor
        new_scale = max(5.0, min(200.0, new_scale))

        self.scale = new_scale
        self.center_x = event.x - mx * self.scale
        self.center_y = event.y + my * self.scale

        self.redraw()

    # --------------------------------------------------
    # UI Adaptive Display Formatting
    # --------------------------------------------------
    def on_tool_change(self):
        if self.selected_idx != -1:
            seg = self.segments[self.selected_idx]
            if seg.get("type", "sidewalk") != self.tool_var.get():
                self.selected_idx = -1
        self.update_button_states()
        self.update_editor_labels()
        self.redraw()

    def update_editor_labels(self):
        self.lbl_x.pack_forget()
        self.precise_x.pack_forget()
        self.lbl_y.pack_forget()
        self.precise_y.pack_forget()
        self.lbl_len.pack_forget()
        self.precise_len.pack_forget()
        self.lbl_ang.pack_forget()
        self.precise_ang.pack_forget()
        self.lbl_width.pack_forget()
        self.precise_width.pack_forget()
        self.lbl_height.pack_forget()
        self.precise_height.pack_forget()

        self.btn_add_new.pack_forget()
        self.btn_update.pack_forget()
        self.btn_delete.pack_forget()
        self.btn_deselect.pack_forget()

        if self.selected_idx == -1:
            seg_type = self.tool_var.get()
        else:
            seg_type = self.segments[self.selected_idx].get("type", "sidewalk")

        if seg_type == "sidewalk":
            self.lbl_x.config(text="Start X (m):")
            self.lbl_y.config(text="Start Y (m):")
            self.lbl_len.config(text="Length (m):")
            self.lbl_ang.config(text="Angle (degrees):")

            self.lbl_x.pack(anchor=tk.W)
            self.precise_x.pack(fill=tk.X, pady=(0, 5))
            self.lbl_y.pack(anchor=tk.W)
            self.precise_y.pack(fill=tk.X, pady=(0, 5))
            self.lbl_len.pack(anchor=tk.W)
            self.precise_len.pack(fill=tk.X, pady=(0, 5))
            self.lbl_ang.pack(anchor=tk.W)
            self.precise_ang.pack(fill=tk.X, pady=(0, 5))
            self.lbl_width.pack(anchor=tk.W)
            self.precise_width.pack(fill=tk.X, pady=(0, 5))
            self.lbl_height.pack(anchor=tk.W)
            self.precise_height.pack(fill=tk.X, pady=(0, 10))
        elif seg_type == "grass":
            self.lbl_x.config(text="Corner 1 X (m):")
            self.lbl_y.config(text="Corner 1 Y (m):")
            self.lbl_len.config(text="Corner 2 X (m):")
            self.lbl_ang.config(text="Corner 2 Y (m):")

            self.lbl_x.pack(anchor=tk.W)
            self.precise_x.pack(fill=tk.X, pady=(0, 5))
            self.lbl_y.pack(anchor=tk.W)
            self.precise_y.pack(fill=tk.X, pady=(0, 5))
            self.lbl_len.pack(anchor=tk.W)
            self.precise_len.pack(fill=tk.X, pady=(0, 5))
            self.lbl_ang.pack(anchor=tk.W)
            self.precise_ang.pack(fill=tk.X, pady=(0, 10))
        elif seg_type == "spawn_point":
            self.lbl_x.config(text="Spawn X (m):")
            self.lbl_y.config(text="Spawn Y (m):")
            self.lbl_ang.config(text="Yaw Angle (deg):")

            self.lbl_x.pack(anchor=tk.W)
            self.precise_x.pack(fill=tk.X, pady=(0, 5))
            self.lbl_y.pack(anchor=tk.W)
            self.precise_y.pack(fill=tk.X, pady=(0, 5))
            self.lbl_ang.pack(anchor=tk.W)
            self.precise_ang.pack(fill=tk.X, pady=(0, 10))

        self.btn_add_new.pack(fill=tk.X, pady=(5, 3))
        self.btn_update.pack(fill=tk.X, pady=3)
        self.btn_delete.pack(fill=tk.X, pady=3)
        self.btn_deselect.pack(fill=tk.X, pady=3)

    def set_entry_val(self, entry, val):
        entry.delete(0, tk.END)
        entry.insert(0, f"{val:.2f}")

    def load_segment_to_entries(self, seg):
        seg_type = seg.get("type", "sidewalk")
        if seg_type == "sidewalk":
            self.set_entry_val(self.precise_x, seg["x"])
            self.set_entry_val(self.precise_y, seg["y"])
            self.set_entry_val(self.precise_len, seg["length"])
            self.set_entry_val(self.precise_ang, math.degrees(seg["yaw"]))
            self.set_entry_val(self.precise_width, seg["width"])
            self.set_entry_val(self.precise_height, seg.get("height", 0.05))
        elif seg_type == "grass":
            self.set_entry_val(self.precise_x, seg["x1"])
            self.set_entry_val(self.precise_y, seg["y1"])
            self.set_entry_val(self.precise_len, seg["x2"])
            self.set_entry_val(self.precise_ang, seg["y2"])
        elif seg_type == "spawn_point":
            self.set_entry_val(self.precise_x, seg["x"])
            self.set_entry_val(self.precise_y, seg["y"])
            self.set_entry_val(self.precise_ang, math.degrees(seg["yaw"]))

    def update_button_states(self):
        if self.selected_idx != -1:
            self.btn_update.config(state=tk.NORMAL)
            self.btn_delete.config(state=tk.NORMAL)
            self.btn_deselect.config(state=tk.NORMAL)
            self.sel_status_label.config(
                text=f"Editing Segment {self.selected_idx}", foreground="#ffa500"
            )
        else:
            self.btn_update.config(state=tk.DISABLED)
            self.btn_delete.config(state=tk.DISABLED)
            self.btn_deselect.config(state=tk.DISABLED)
            self.sel_status_label.config(text="No segment selected", foreground="gray")

    def reindex_spawn_points(self):
        """Maintains consecutive start position labeling after deletions"""
        count = 1
        for seg in self.segments:
            if seg.get("type") == "spawn_point":
                seg["id"] = count
                count += 1

    # --------------------------------------------------
    # Frame & Scale Conversions
    # --------------------------------------------------
    def on_resize(self, event):
        self.center_x = event.width / 2
        self.center_y = event.height / 2
        self.redraw()

    def to_canvas(self, wx, wy):
        cx = self.center_x + wx * self.scale
        cy = self.center_y - wy * self.scale
        return cx, cy

    def to_world(self, cx, cy):
        wx = (cx - self.center_x) / self.scale
        wy = (self.center_y - cy) / self.scale
        return wx, wy

    def zoom_in(self):
        self.scale = min(200.0, self.scale + 5)
        self.redraw()

    def zoom_out(self):
        self.scale = max(5.0, self.scale - 5)
        self.redraw()

    # --------------------------------------------------
    # Selection and Dragging Math
    # --------------------------------------------------
    def snap_to_sidewalk(self, wx, wy):
        """
        Pulls coordinate to the exact centerline of the nearest sidewalk
        and snaps along its path in 0.5m intervals.
        """
        sidewalks = [s for s in self.segments if s.get("type") == "sidewalk"]
        if not sidewalks:
            return round(wx), round(wy), 0.0

        closest_idx = -1
        min_dist = float("inf")
        best_t_snapped = 0.0

        for idx, seg in enumerate(sidewalks):
            x_s, y_s = seg["x"], seg["y"]
            L, yaw = seg["length"], seg["yaw"]

            dx = L * math.cos(yaw)
            dy = L * math.sin(yaw)

            wx_rel = wx - x_s
            wy_rel = wy - y_s
            line_lensq = L * L

            if line_lensq == 0:
                t = 0.0
            else:
                t_ratio = (wx_rel * dx + wy_rel * dy) / line_lensq
                t_ratio = max(0.0, min(1.0, t_ratio))
                t = t_ratio * L

            c_wx = x_s + t * math.cos(yaw)
            c_wy = y_s + t * math.sin(yaw)
            dist = math.sqrt((wx - c_wx) ** 2 + (wy - c_wy) ** 2)

            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
                t_snapped = round(t * 2.0) / 2.0
                best_t_snapped = max(0.0, min(L, t_snapped))

        if closest_idx != -1:
            best_seg = sidewalks[closest_idx]
            x_s, y_s = best_seg["x"], best_seg["y"]
            yaw = best_seg["yaw"]
            snapped_x = x_s + best_t_snapped * math.cos(yaw)
            snapped_y = y_s + best_t_snapped * math.sin(yaw)
            return snapped_x, snapped_y, yaw

        return round(wx), round(wy), 0.0

    def find_clicked_segment(self, wx, wy):
        closest_idx = -1
        min_dist = float("inf")
        active_tool = self.tool_var.get()

        for idx, seg in enumerate(self.segments):
            seg_type = seg.get("type", "sidewalk")

            if seg_type != active_tool:
                continue

            if seg_type == "sidewalk":
                x_s, y_s = seg["x"], seg["y"]
                L, yaw, w = seg["length"], seg["yaw"], seg["width"]

                dx = L * math.cos(yaw)
                dy = L * math.sin(yaw)

                wx_rel = wx - x_s
                wy_rel = wy - y_s

                line_lensq = L * L
                if line_lensq == 0:
                    t = 0.0
                else:
                    t = (wx_rel * dx + wy_rel * dy) / line_lensq
                    t = max(0.0, min(1.0, t))

                c_wx = x_s + t * dx
                c_wy = y_s + t * dy

                dist = math.sqrt((wx - c_wx) ** 2 + (wy - c_wy) ** 2)
                selection_tolerance = max(w / 2.0, 0.4)
                if dist <= selection_tolerance:
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = idx

            elif seg_type == "grass":
                x1, y1 = seg["x1"], seg["y1"]
                x2, y2 = seg["x2"], seg["y2"]
                min_x, max_x = min(x1, x2), max(x1, x2)
                min_y, max_y = min(y1, y2), max(y1, y2)

                if min_x <= wx <= max_x and min_y <= wy <= max_y:
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    dist = math.sqrt((wx - cx) ** 2 + (wy - cy) ** 2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = idx

            elif seg_type == "spawn_point":
                # Direct circular proximity check (40cm range)
                dist = math.sqrt((wx - seg["x"]) ** 2 + (wy - seg["y"]) ** 2)
                if dist <= 0.4:
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = idx

        return closest_idx

    # --------------------------------------------------
    # Mouse Interaction Engine
    # --------------------------------------------------
    def on_mouse_press(self, event):
        self.press_cx = event.x
        self.press_cy = event.y
        self.has_dragged = False
        self.drag_mode = None

        # Space-Pan Override State
        if self.space_held:
            self.drag_mode = "pan"
            self.pan_start_x = event.x
            self.pan_start_y = event.y
            return

        wx, wy = self.to_world(event.x, event.y)
        active_tool = self.tool_var.get()

        # 1. Check handles of the currently selected segment
        if self.selected_idx != -1:
            seg = self.segments[self.selected_idx]
            seg_type = seg.get("type", "sidewalk")

            if seg_type == active_tool:
                if seg_type == "sidewalk":
                    x_s, y_s = seg["x"], seg["y"]
                    L, yaw, w = seg["length"], seg["yaw"], seg["width"]

                    x_e = x_s + L * math.cos(yaw)
                    y_e = y_s + L * math.sin(yaw)
                    cx_e, cy_e = self.to_canvas(x_e, y_e)
                    dist_e = math.sqrt((event.x - cx_e) ** 2 + (event.y - cy_e) ** 2)

                    x_m = x_s + (L / 2) * math.cos(yaw)
                    y_m = y_s + (L / 2) * math.sin(yaw)
                    p_x = -math.sin(yaw)
                    p_y = math.cos(yaw)
                    x_w = x_m + (w / 2) * p_x
                    y_w = y_m + (w / 2) * p_y
                    cx_w, cy_w = self.to_canvas(x_w, y_w)
                    dist_w = math.sqrt((event.x - cx_w) ** 2 + (event.y - cy_w) ** 2)

                    if dist_e <= 10:
                        self.drag_mode = "resize_length"
                        self.start_wx = x_s
                        self.start_wy = y_s
                        return
                    elif dist_w <= 10:
                        self.drag_mode = "resize_width"
                        return

                    if self.find_clicked_segment(wx, wy) == self.selected_idx:
                        self.drag_mode = "move"
                        self.drag_offset_x = wx - x_s
                        self.drag_offset_y = wy - y_s
                        return

                elif seg_type == "grass":
                    x1, y1 = seg["x1"], seg["y1"]
                    x2, y2 = seg["x2"], seg["y2"]

                    corners = [
                        (x1, y1, "x1", "y1"),
                        (x2, y1, "x2", "y1"),
                        (x1, y2, "x1", "y2"),
                        (x2, y2, "x2", "y2"),
                    ]
                    for cx_w, cy_w, x_key, y_key in corners:
                        ccx, ccy = self.to_canvas(cx_w, cy_w)
                        dist = math.sqrt((event.x - ccx) ** 2 + (event.y - ccy) ** 2)
                        if dist <= 10:
                            self.drag_mode = "resize_grass_corner"
                            self.active_corner_keys = (x_key, y_key)
                            return

                    if self.find_clicked_segment(wx, wy) == self.selected_idx:
                        self.drag_mode = "move_grass"
                        self.drag_offset_x1 = wx - x1
                        self.drag_offset_y1 = wy - y1
                        self.drag_offset_x2 = wx - x2
                        self.drag_offset_y2 = wy - y2
                        return

                elif seg_type == "spawn_point":
                    # Rotate handle
                    x_sp, y_sp = seg["x"], seg["y"]
                    yaw = seg["yaw"]
                    cx_sp, cy_sp = self.to_canvas(x_sp, y_sp)
                    ax = cx_sp + 18 * math.cos(yaw)
                    ay = cy_sp - 18 * math.sin(yaw)
                    dist_rot = math.sqrt((event.x - ax) ** 2 + (event.y - ay) ** 2)

                    if dist_rot <= 10:
                        self.drag_mode = "rotate_spawn"
                        return

                    if self.find_clicked_segment(wx, wy) == self.selected_idx:
                        self.drag_mode = "move_spawn"
                        self.drag_offset_x = wx - x_sp
                        self.drag_offset_y = wy - y_sp
                        return

        # 2. Check other segments matching active tool type
        clicked_idx = self.find_clicked_segment(wx, wy)
        if clicked_idx != -1:
            self.selected_idx = clicked_idx
            seg = self.segments[clicked_idx]
            seg_type = seg.get("type", "sidewalk")

            if seg_type == "sidewalk":
                self.drag_mode = "move"
                self.drag_offset_x = wx - seg["x"]
                self.drag_offset_y = wy - seg["y"]
            elif seg_type == "grass":
                self.drag_mode = "move_grass"
                self.drag_offset_x1 = wx - seg["x1"]
                self.drag_offset_y1 = wy - seg["y1"]
                self.drag_offset_x2 = wx - seg["x2"]
                self.drag_offset_y2 = seg["y2"]
            elif seg_type == "spawn_point":
                self.drag_mode = "move_spawn"
                self.drag_offset_x = wx - seg["x"]
                self.drag_offset_y = wy - seg["y"]

            self.load_segment_to_entries(seg)
            self.update_button_states()
            self.update_editor_labels()
            self.redraw()
            return

        # 3. Handle Spawning Point creation on single click
        if active_tool == "spawn_point":
            if self.snap_var.get():
                wx, wy, yaw = self.snap_to_sidewalk(wx, wy)
            else:
                yaw = 0.0

            count = len([s for s in self.segments if s.get("type") == "spawn_point"])
            self.segments.append(
                {"type": "spawn_point", "x": wx, "y": wy, "yaw": yaw, "id": count + 1}
            )
            self.selected_idx = len(self.segments) - 1
            self.load_segment_to_entries(self.segments[-1])
            self.update_button_states()
            self.update_editor_labels()
            self.redraw()
            return

        # 4. Clicking empty space (Sidewalk/Grass)
        self.selected_idx = -1
        self.update_button_states()
        self.update_editor_labels()
        self.redraw()

        if self.snap_var.get():
            wx, wy = round(wx), round(wy)
        self.start_wx = wx
        self.start_wy = wy
        self.drag_mode = "draw"

    def on_mouse_drag(self, event):
        if not self.drag_mode:
            return

        move_dist = math.sqrt(
            (event.x - self.press_cx) ** 2 + (event.y - self.press_cy) ** 2
        )
        if move_dist > 5:
            self.has_dragged = True

        curr_wx, curr_wy = self.to_world(event.x, event.y)

        # --- MODE: SPACE PANNING ---
        if self.drag_mode == "pan":
            dx = event.x - self.pan_start_x
            dy = event.y - self.pan_start_y
            self.center_x += dx
            self.center_y += dy
            self.pan_start_x = event.x
            self.pan_start_y = event.y
            self.schedule_redraw()
            return

        # --- MODE: DRAWING NEW ELEMENTS ---
        elif self.drag_mode == "draw":
            if not self.has_dragged:
                return
            if self.snap_var.get():
                curr_wx, curr_wy = round(curr_wx), round(curr_wy)

            self.canvas.delete("preview")
            cx1, cy1 = self.to_canvas(self.start_wx, self.start_wy)
            cx2, cy2 = self.to_canvas(curr_wx, curr_wy)

            tool = self.tool_var.get()
            if tool == "sidewalk":
                try:
                    width = float(self.default_width_entry.get())
                except ValueError:
                    width = 1.2

                self.canvas.create_line(
                    cx1,
                    cy1,
                    cx2,
                    cy2,
                    width=width * self.scale,
                    fill="#555555",
                    capstyle="butt",
                    tags="preview",
                )
                self.canvas.create_line(
                    cx1, cy1, cx2, cy2, fill="#ff4500", dash=(4, 4), tags="preview"
                )
            elif tool == "grass":
                self.canvas.create_rectangle(
                    cx1, cy1, cx2, cy2, outline="#00ff00", dash=(4, 4), tags="preview"
                )

        # --- MODE: TRANSLATE SIDEWALK ---
        elif self.drag_mode == "move":
            seg = self.segments[self.selected_idx]
            new_x = curr_wx - self.drag_offset_x
            new_y = curr_wy - self.drag_offset_y

            if self.snap_var.get():
                new_x, new_y = round(new_x), round(new_y)

            seg["x"] = new_x
            seg["y"] = new_y
            self.load_segment_to_entries(seg)
            self.redraw()

        # --- MODE: TRANSLATE GRASS ---
        elif self.drag_mode == "move_grass":
            seg = self.segments[self.selected_idx]
            new_x1 = curr_wx - self.drag_offset_x1
            new_y1 = curr_wy - self.drag_offset_y1
            new_x2 = curr_wx - self.drag_offset_x2
            new_y2 = curr_wy - self.drag_offset_y2

            if self.snap_var.get():
                new_x1, new_y1 = round(new_x1), round(new_y1)
                new_x2, new_y2 = round(new_x2), round(new_y2)

            seg["x1"] = new_x1
            seg["y1"] = new_y1
            seg["x2"] = new_x2
            seg["y2"] = new_y2
            self.load_segment_to_entries(seg)
            self.redraw()

        # --- MODE: TRANSLATE SPAWN POINT ---
        elif self.drag_mode == "move_spawn":
            seg = self.segments[self.selected_idx]
            new_x = curr_wx - self.drag_offset_x
            new_y = curr_wy - self.drag_offset_y

            if self.snap_var.get():
                new_x, new_y, yaw = self.snap_to_sidewalk(new_x, new_y)
                seg["yaw"] = yaw

            seg["x"] = new_x
            seg["y"] = new_y
            self.load_segment_to_entries(seg)
            self.redraw()

        # --- MODE: RESIZE SIDEWALK LENGTH/ANGLE ---
        elif self.drag_mode == "resize_length":
            seg = self.segments[self.selected_idx]
            x_s, y_s = self.start_wx, self.start_wy

            if self.snap_var.get():
                curr_wx, curr_wy = round(curr_wx), round(curr_wy)

            dx = curr_wx - x_s
            dy = curr_wy - y_s
            length = math.sqrt(dx**2 + dy**2)

            if length > 0.1:
                yaw = math.atan2(dy, dx)
                seg["length"] = length
                seg["yaw"] = yaw
                self.load_segment_to_entries(seg)
                self.redraw()

        # --- MODE: RESIZE SIDEWALK WIDTH ---
        elif self.drag_mode == "resize_width":
            seg = self.segments[self.selected_idx]
            x_s, y_s = seg["x"], seg["y"]
            L, yaw = seg["length"], seg["yaw"]

            x_m = x_s + (L / 2) * math.cos(yaw)
            y_m = y_s + (L / 2) * math.sin(yaw)
            p_x = -math.sin(yaw)
            p_y = math.cos(yaw)

            mx = curr_wx - x_m
            my = curr_wy - y_m
            perpendicular_dist = mx * p_x + my * p_y
            w = abs(perpendicular_dist) * 2.0

            if self.snap_var.get():
                w = round(w * 10.0) / 10.0

            seg["width"] = max(0.1, w)
            self.load_segment_to_entries(seg)
            self.redraw()

        # --- MODE: ROTATE SPAWN POINT ---
        elif self.drag_mode == "rotate_spawn":
            seg = self.segments[self.selected_idx]
            dx = curr_wx - seg["x"]
            dy = curr_wy - seg["y"]
            yaw = math.atan2(dy, dx)
            seg["yaw"] = yaw
            self.load_segment_to_entries(seg)
            self.redraw()

        # --- MODE: RESIZE GRASS CORNER ---
        elif self.drag_mode == "resize_grass_corner":
            seg = self.segments[self.selected_idx]
            x_key, y_key = self.active_corner_keys

            if self.snap_var.get():
                curr_wx = round(curr_wx)
                curr_wy = round(curr_wy)

            seg[x_key] = curr_wx
            seg[y_key] = curr_wy
            self.load_segment_to_entries(seg)
            self.schedule_redraw()

    def schedule_redraw(self):
        """Throttle redraws during drag. Keeps ~30fps max."""
        if self._redraw_timer:
            self.root.after_cancel(self._redraw_timer)
        self._redraw_timer = self.root.after(16, self._do_redraw)

    def _do_redraw(self):
        self._redraw_timer = None
        self._drag_redraw_count += 1
        self.redraw()

    def on_mouse_release(self, event):
        if not self.drag_mode:
            return
        # Cancel pending throttled redraw, do final redraw immediately
        if self._redraw_timer:
            self.root.after_cancel(self._redraw_timer)
            self._redraw_timer = None
        self.canvas.delete("preview")

        if self.drag_mode == "draw" and self.has_dragged:
            end_wx, end_wy = self.to_world(event.x, event.y)
            if self.snap_var.get():
                end_wx, end_wy = round(end_wx), round(end_wy)

            dx = end_wx - self.start_wx
            dy = end_wy - self.start_wy

            tool = self.tool_var.get()
            if tool == "sidewalk":
                length = math.sqrt(dx**2 + dy**2)
                if length >= 0.2:
                    yaw = math.atan2(dy, dx)
                    try:
                        width = float(self.default_width_entry.get())
                        height = float(self.default_height_entry.get())
                    except ValueError:
                        width = 1.2
                        height = 0.05

                    self.segments.append(
                        {
                            "type": "sidewalk",
                            "x": self.start_wx,
                            "y": self.start_wy,
                            "length": length,
                            "yaw": yaw,
                            "width": width,
                            "height": height,
                        }
                    )
            elif tool == "grass":
                if abs(dx) > 0.2 and abs(dy) > 0.2:
                    self.segments.append(
                        {
                            "type": "grass",
                            "x1": self.start_wx,
                            "y1": self.start_wy,
                            "x2": end_wx,
                            "y2": end_wy,
                        }
                    )

        self.drag_mode = None
        if self._redraw_timer:
            self.root.after_cancel(self._redraw_timer)
            self._redraw_timer = None
        self.redraw()

    # --------------------------------------------------
    # Precise Panel Command Triggers
    # --------------------------------------------------
    def add_precise_segment(self):
        try:
            x1 = float(self.precise_x.get())
            y1 = float(self.precise_y.get())

            mode = self.tool_var.get()
            if mode == "sidewalk":
                length = float(self.precise_len.get())
                angle_deg = float(self.precise_ang.get())
                width = float(self.precise_width.get())
                height = float(self.precise_height.get())
                yaw = math.radians(angle_deg)

                self.segments.append(
                    {
                        "type": "sidewalk",
                        "x": x1,
                        "y": y1,
                        "length": length,
                        "yaw": yaw,
                        "width": width,
                        "height": height,
                    }
                )
            elif mode == "grass":
                x2 = float(self.precise_len.get())
                y2 = float(self.precise_ang.get())
                self.segments.append(
                    {"type": "grass", "x1": x1, "y1": y1, "x2": x2, "y2": y2}
                )
            elif mode == "spawn_point":
                yaw_deg = float(self.precise_ang.get())
                count = len(
                    [s for s in self.segments if s.get("type") == "spawn_point"]
                )
                self.segments.append(
                    {
                        "type": "spawn_point",
                        "x": x1,
                        "y": y1,
                        "yaw": math.radians(yaw_deg),
                        "id": count + 1,
                    }
                )

            self.selected_idx = -1
            self.update_button_states()
            self.update_editor_labels()
            self.redraw()
        except ValueError:
            messagebox.showerror(
                "Invalid Input", "Please enter valid numeric parameters."
            )

    def update_selected_segment(self):
        if self.selected_idx == -1:
            return
        try:
            x1 = float(self.precise_x.get())
            y1 = float(self.precise_y.get())

            seg = self.segments[self.selected_idx]
            seg_type = seg.get("type", "sidewalk")

            if seg_type == "sidewalk":
                length = float(self.precise_len.get())
                angle_deg = float(self.precise_ang.get())
                width = float(self.precise_width.get())
                height = float(self.precise_height.get())

                self.segments[self.selected_idx] = {
                    "type": "sidewalk",
                    "x": x1,
                    "y": y1,
                    "length": length,
                    "yaw": math.radians(angle_deg),
                    "width": width,
                    "height": height,
                }
            elif seg_type == "grass":
                x2 = float(self.precise_len.get())
                y2 = float(self.precise_ang.get())

                self.segments[self.selected_idx] = {
                    "type": "grass",
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                }
            elif seg_type == "spawn_point":
                yaw_deg = float(self.precise_ang.get())
                seg["x"] = x1
                seg["y"] = y1
                seg["yaw"] = math.radians(yaw_deg)

            self.redraw()
        except ValueError:
            messagebox.showerror(
                "Invalid Input", "Please enter valid numeric parameters."
            )

    def delete_selected_segment(self):
        if self.selected_idx == -1:
            return
        self.segments.pop(self.selected_idx)
        self.selected_idx = -1
        self.reindex_spawn_points()
        self.update_button_states()
        self.update_editor_labels()
        self.redraw()

    def deselect_segment(self):
        self.selected_idx = -1
        self.update_button_states()
        self.update_editor_labels()
        self.redraw()

    # --------------------------------------------------
    # Draw State Updating & Layer-Ordering Engine
    # --------------------------------------------------
    def redraw(self):
        # Only delete dynamic layers, keep cached grid
        self.canvas.delete("grass", "sidewalk", "spawn_point", "overlay", "preview")

        self.draw_grid()  # Layer 1 (cached)
        self.draw_grass_patches()  # Layer 2
        self.draw_sidewalks()  # Layer 3
        self.draw_spawn_points()  # Layer 3.5
        self.draw_active_handles()  # Layer 4

    def draw_grid(self):
        # Skip redraw if viewport unchanged (cache grid)
        if hasattr(self, '_grid_items') and self._last_grid_center == (self.center_x, self.center_y) and self._last_grid_scale == self.scale:
            return
        # Delete old grid items
        self.canvas.delete("grid")
        self._grid_items = []

        # 1. Draw standard grid vertical lines
        for x in range(-50, 51):
            if x == 0:
                continue
            cx, cy_start = self.to_canvas(x, -50)
            _, cy_end = self.to_canvas(x, 50)
            color = "#363636"
            dash = () if x % 5 == 0 else (1, 5)
            self._grid_items.append(
                self.canvas.create_line(
                    cx, cy_start, cx, cy_end, fill=color, dash=dash, tags="grid"
                )
            )

        # 2. Draw standard grid horizontal lines
        for y in range(-50, 51):
            if y == 0:
                continue
            cx_start, cy = self.to_canvas(-50, y)
            cx_end, _ = self.to_canvas(50, y)
            color = "#363636"
            dash = () if y % 5 == 0 else (1, 5)
            self._grid_items.append(
                self.canvas.create_line(
                    cx_start, cy, cx_end, cy, fill=color, dash=dash, tags="grid"
                )
            )

        # 3. Draw vertical Y-Axis (Green)
        cx_0, cy_start = self.to_canvas(0, -50)
        _, cy_end = self.to_canvas(0, 50)
        self._grid_items.append(
            self.canvas.create_line(
                cx_0, cy_start, cx_0, cy_end, fill="#2e7d32", width=3, tags="grid"
            )
        )

        # 4. Draw horizontal X-Axis (Red)
        cx_start, cy_0 = self.to_canvas(-50, 0)
        cx_end, _ = self.to_canvas(50, 0)
        self._grid_items.append(
            self.canvas.create_line(
                cx_start, cy_0, cx_end, cy_0, fill="#d32f2f", width=3, tags="grid"
            )
        )

        self._last_grid_center = (self.center_x, self.center_y)
        self._last_grid_scale = self.scale

    def draw_grass_patches(self):
        """Layer 1: Deep forest green grass patches"""
        for idx, seg in enumerate(self.segments):
            if seg.get("type", "sidewalk") != "grass":
                continue
            x1, y1 = seg["x1"], seg["y1"]
            x2, y2 = seg["x2"], seg["y2"]

            cx1, cy1 = self.to_canvas(x1, y1)
            cx2, cy2 = self.to_canvas(x2, y2)

            if idx == self.selected_idx:
                self.canvas.create_rectangle(
                    min(cx1, cx2) - 3,
                    min(cy1, cy2) - 3,
                    max(cx1, cx2) + 3,
                    max(cy1, cy2) + 3,
                    outline="#ffd700",
                    width=3,
                    tags="grass",
                )

            self.canvas.create_rectangle(
                cx1,
                cy1,
                cx2,
                cy2,
                fill="#264d26",
                outline="#3b6b3b",
                width=1,
                tags="grass",
            )

    def draw_sidewalks(self):
        """Layer 3: Sidewalk slabs drawn cleanly on top of grass and grid lines"""
        for idx, seg in enumerate(self.segments):
            if seg.get("type", "sidewalk") != "sidewalk":
                continue
            x_s, y_s = seg["x"], seg["y"]
            L, yaw, w = seg["length"], seg["yaw"], seg["width"]

            x_e = x_s + L * math.cos(yaw)
            y_e = y_s + L * math.sin(yaw)

            cx1, cy1 = self.to_canvas(x_s, y_s)
            cx2, cy2 = self.to_canvas(x_e, y_e)

            if idx == self.selected_idx:
                self.canvas.create_line(
                    cx1,
                    cy1,
                    cx2,
                    cy2,
                    width=(w * self.scale) + 6,
                    fill="#ffd700",
                    capstyle="butt",
                    tags="sidewalk",
                )

            self.canvas.create_line(
                cx1,
                cy1,
                cx2,
                cy2,
                width=w * self.scale,
                fill="#8a8a8a",
                capstyle="butt",
                tags="sidewalk",
            )

            # Anchor start node indicator
            anchor_color = "#ffd700" if idx == self.selected_idx else "#ff4500"
            self.canvas.create_oval(
                cx1 - 4,
                cy1 - 4,
                cx1 + 4,
                cy1 + 4,
                fill=anchor_color,
                outline="white",
                tags="sidewalk",
            )

    def draw_spawn_points(self):
        """Layer 3.5: Robot Spawn points drawn on top of the sidewalk path"""
        for idx, seg in enumerate(self.segments):
            if seg.get("type") != "spawn_point":
                continue
            x, y = seg["x"], seg["y"]
            yaw = seg["yaw"]
            sp_id = seg["id"]

            cx, cy = self.to_canvas(x, y)

            # Selection glow outer ring
            if idx == self.selected_idx:
                self.canvas.create_oval(
                    cx - 14,
                    cy - 14,
                    cx + 14,
                    cy + 14,
                    outline="#ffd700",
                    width=3,
                    tags="spawn_point",
                )

            # Draw primary Spawn icon (Cyan circle)
            self.canvas.create_oval(
                cx - 10,
                cy - 10,
                cx + 10,
                cy + 10,
                fill="#00ffff",
                outline="white",
                width=2,
                tags="spawn_point",
            )

            # Heading reference arrow
            arrow_len = 18
            ax = cx + arrow_len * math.cos(yaw)
            ay = cy - arrow_len * math.sin(yaw)
            self.canvas.create_line(
                cx,
                cy,
                ax,
                ay,
                fill="#ff1493",
                width=2,
                arrow=tk.LAST,
                tags="spawn_point",
            )

            # Inner ID label text
            self.canvas.create_text(
                cx,
                cy,
                text=str(sp_id),
                fill="black",
                font=("Arial", 9, "bold"),
                tags="spawn_point",
            )

    def draw_active_handles(self):
        """Layer 4: Selection handles and parameter text overlay"""
        for idx, seg in enumerate(self.segments):
            seg_type = seg.get("type", "sidewalk")

            if seg_type == "sidewalk":
                x_s, y_s = seg["x"], seg["y"]
                L, yaw, w = seg["length"], seg["yaw"], seg["width"]
                h = seg.get("height", 0.05)

                x_e = x_s + L * math.cos(yaw)
                y_e = y_s + L * math.sin(yaw)
                cx2, cy2 = self.to_canvas(x_e, y_e)

                if idx == self.selected_idx:
                    # Blue length/yaw handle
                    self.canvas.create_rectangle(
                        cx2 - 6,
                        cy2 - 6,
                        cx2 + 6,
                        cy2 + 6,
                        fill="#00bfff",
                        outline="white",
                        tags="overlay",
                    )
                    # Pink width handle
                    x_m = x_s + (L / 2) * math.cos(yaw)
                    y_m = y_s + (L / 2) * math.sin(yaw)
                    p_x = -math.sin(yaw)
                    p_y = math.cos(yaw)
                    x_w = x_m + (w / 2) * p_x
                    y_w = y_m + (w / 2) * p_y
                    cx_w, cy_w = self.to_canvas(x_w, y_w)
                    self.canvas.create_rectangle(
                        cx_w - 6,
                        cy_w - 6,
                        cx_w + 6,
                        cy_w + 6,
                        fill="#ff1493",
                        outline="white",
                        tags="overlay",
                    )

                # Render parameters overlay (Toggled)
                if self.show_labels_var.get():
                    mid_x = x_s + (L / 2) * math.cos(yaw)
                    mid_y = y_s + (L / 2) * math.sin(yaw)
                    mcx, mcy = self.to_canvas(mid_x, mid_y)
                    text_color = "#ffd700" if idx == self.selected_idx else "black"
                    self.canvas.create_text(
                        mcx,
                        mcy,
                        text=f"S{idx}\n{L:.1f}m\nW:{w:.1f}m\nH:{h:.2f}m",
                        fill=text_color,
                        font=("Arial", 8, "bold"),
                        justify=tk.CENTER,
                        tags="overlay",
                    )

            elif seg_type == "grass":
                x1, y1 = seg["x1"], seg["y1"]
                x2, y2 = seg["x2"], seg["y2"]

                cx1, cy1 = self.to_canvas(x1, y1)
                cx2, cy2 = self.to_canvas(x2, y2)

                if idx == self.selected_idx:
                    # Cyan corner handles
                    for px, py in [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]:
                        ccx, ccy = self.to_canvas(px, py)
                        self.canvas.create_rectangle(
                            ccx - 5,
                            ccy - 5,
                            ccx + 5,
                            ccy + 5,
                            fill="#00ffff",
                            outline="white",
                            tags="overlay",
                        )

                # Render parameters overlay (Toggled)
                if self.show_labels_var.get():
                    mx = (x1 + x2) / 2.0
                    my = (y1 + y2) / 2.0
                    mcx, mcy = self.to_canvas(mx, my)
                    text_color = "#ffd700" if idx == self.selected_idx else "white"
                    self.canvas.create_text(
                        mcx,
                        mcy,
                        text=f"Grass G{idx}\n{abs(x2 - x1):.1f}x{abs(y2 - y1):.1f}m",
                        fill=text_color,
                        font=("Arial", 8, "bold"),
                        justify=tk.CENTER,
                        tags="overlay",
                    )

            elif seg_type == "spawn_point":
                if idx == self.selected_idx:
                    x, y, yaw = seg["x"], seg["y"], seg["yaw"]
                    cx, cy = self.to_canvas(x, y)
                    arrow_len = 18
                    ax = cx + arrow_len * math.cos(yaw)
                    ay = cy - arrow_len * math.sin(yaw)

                    # Blue rotation handle on direction arrow tip
                    self.canvas.create_rectangle(
                        ax - 5,
                        ay - 5,
                        ax + 5,
                        ay + 5,
                        fill="#00bfff",
                        outline="white",
                        tags="overlay",
                    )

    def undo_last(self):
        if self.segments:
            self.segments.pop()
            self.selected_idx = -1
            self.reindex_spawn_points()
            self.update_button_states()
            self.update_editor_labels()
            self.redraw()

    def clear_map(self):
        if messagebox.askyesno(
            "Confirm Clear", "Are you sure you want to delete all segments?"
        ):
            self.segments = []
            self.selected_idx = -1
            self.update_button_states()
            self.update_editor_labels()
            self.redraw()

    # --------------------------------------------------
    # SDF World Importer Logic
    # --------------------------------------------------
    def import_sdf(self):
        script_dir = Path(__file__).parent.resolve()

        file_path = filedialog.askopenfilename(
            filetypes=[("Simulation Description Format", "*.sdf")],
            initialdir=script_dir,
            title="Import Gazebo World SDF",
        )
        if not file_path:
            return

        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
        except Exception as e:
            messagebox.showerror("Import Error", f"Failed to parse SDF: {e}")
            return

        # Locate model elements inside the world
        models = root.findall(".//model")
        sidewalk_model = None
        grass_network_model = None

        for m in models:
            name = m.get("name")
            if name == "sidewalk_network":
                sidewalk_model = m
            elif name == "grass_network":
                grass_network_model = m

        imported_segments = []

        # Parse sidewalks
        if sidewalk_model is not None:
            links = sidewalk_model.findall(".//link")
            for link in links:
                pose_elem = link.find("pose")
                if pose_elem is None or not pose_elem.text:
                    continue
                pose_parts = pose_elem.text.strip().split()
                if len(pose_parts) < 6:
                    continue
                x = float(pose_parts[0])
                y = float(pose_parts[1])
                yaw = float(pose_parts[5])

                box = link.find(".//geometry/box/size")
                if box is None or not box.text:
                    continue
                size_parts = box.text.strip().split()
                if len(size_parts) < 3:
                    continue
                L = float(size_parts[0])
                W = float(size_parts[1])
                H = float(size_parts[2])

                imported_segments.append(
                    {
                        "type": "sidewalk",
                        "x": x,
                        "y": y,
                        "length": L,
                        "yaw": yaw,
                        "width": W,
                        "height": H,
                    }
                )

        # Parse custom grass network
        if grass_network_model is not None:
            links = grass_network_model.findall(".//link")
            for link in links:
                pose_elem = link.find("pose")
                if pose_elem is None or not pose_elem.text:
                    continue
                pose_parts = pose_elem.text.strip().split()
                if len(pose_parts) < 6:
                    continue
                cx = float(pose_parts[0])
                cy = float(pose_parts[1])

                box = link.find(".//geometry/box/size")
                if box is None or not box.text:
                    continue
                size_parts = box.text.strip().split()
                if len(size_parts) < 3:
                    continue
                dx = float(size_parts[0])
                dy = float(size_parts[1])

                x1 = cx - dx / 2.0
                x2 = cx + dx / 2.0
                y1 = cy - dy / 2.0
                y2 = cy + dy / 2.0

                imported_segments.append(
                    {"type": "grass", "x1": x1, "y1": y1, "x2": x2, "y2": y2}
                )

        # Parse pre-defined Robot Spawn points from SDF <frame> tags [3]
        frames = root.findall(".//frame")
        for frame in frames:
            name = frame.get("name")
            if name and name.startswith("start_position_"):
                try:
                    sp_id = int(name.split("_")[-1])
                except ValueError:
                    sp_id = (
                        len(
                            [s for s in imported_segments if s["type"] == "spawn_point"]
                        )
                        + 1
                    )

                pose_elem = frame.find("pose")
                if pose_elem is None or not pose_elem.text:
                    continue
                pose_parts = pose_elem.text.strip().split()
                if len(pose_parts) < 6:
                    continue
                x = float(pose_parts[0])
                y = float(pose_parts[1])
                yaw = float(pose_parts[5])

                imported_segments.append(
                    {"type": "spawn_point", "x": x, "y": y, "yaw": yaw, "id": sp_id}
                )

        if not imported_segments:
            messagebox.showwarning(
                "Import Warning",
                "Parsed SDF successfully, but found no usable map data to restore.",
            )
            return

        # Replace active segments with imported collection
        self.segments = imported_segments
        self.selected_idx = -1
        self.reindex_spawn_points()
        self.update_button_states()
        self.update_editor_labels()
        self.redraw()

        num_sidewalks = sum(1 for s in self.segments if s["type"] == "sidewalk")
        num_grass = sum(1 for s in self.segments if s["type"] == "grass")
        num_spawns = sum(1 for s in self.segments if s["type"] == "spawn_point")
        messagebox.showinfo(
            "Import Complete",
            f"Successfully imported:\n- {num_sidewalks} Sidewalk(s)\n- {num_grass} Grass Patch(es)\n- {num_spawns} Spawn Point(s)",
        )

    # --------------------------------------------------
    # SDF World Exporter Logic
    # --------------------------------------------------
    def export_sdf(self):
        if not self.segments:
            messagebox.showwarning("Empty Map", "No segments created to export.")
            return

        script_dir = Path(__file__).parent.resolve()

        file_path = filedialog.asksaveasfilename(
            defaultextension=".sdf",
            filetypes=[("Simulation Description Format", "*.sdf")],
            initialdir=script_dir,
            title="Save Map as Gazebo World",
        )

        if not file_path:
            return

        grass_patches = [
            s for s in self.segments if s.get("type", "sidewalk") == "grass"
        ]
        sidewalks = [
            s for s in self.segments if s.get("type", "sidewalk") == "sidewalk"
        ]
        spawn_points = [s for s in self.segments if s.get("type") == "spawn_point"]

        # 1. GENERATE GRASS LAYOUTS
        if not grass_patches:
            if not sidewalks:
                min_x, max_x, min_y, max_y = -10, 10, -10, 10
            else:
                min_x = min(
                    min(seg["x"], seg["x"] + seg["length"] * math.cos(seg["yaw"]))
                    for seg in sidewalks
                )
                max_x = max(
                    max(seg["x"], seg["x"] + seg["length"] * math.cos(seg["yaw"]))
                    for seg in sidewalks
                )
                min_y = min(
                    min(seg["y"], seg["y"] + seg["length"] * math.sin(seg["yaw"]))
                    for seg in sidewalks
                )
                max_y = max(
                    max(seg["y"], seg["y"] + seg["length"] * math.sin(seg["yaw"]))
                    for seg in sidewalks
                )

            width_x = max_x - min_x
            width_y = max_y - min_y
            grass_center_x = min_x + (width_x / 2.0)
            grass_center_y = min_y + (width_y / 2.0)
            grass_size_x = max(50.0, width_x + 30.0)
            grass_size_y = max(50.0, width_y + 30.0)

            grass_xml = f"""
    <model name="grass_plane">
      <static>true</static>
      <link name="link">
        <pose>{grass_center_x:.4f} {grass_center_y:.4f} -0.05 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>{grass_size_x:.2f} {grass_size_y:.2f} 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{grass_size_x:.2f} {grass_size_y:.2f} 0.1</size></box></geometry>
          <material>
            <ambient>0.3 0.6 0.3 1</ambient>
            <diffuse>0.3 0.6 0.3 1</diffuse>
          </material>
          <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
            <label>2</label> <!-- 2 = Grass -->
          </plugin>
        </visual>
      </link>
    </model>"""
        else:
            grass_xml_lines = []
            for idx, gp in enumerate(grass_patches):
                x1, y1 = gp["x1"], gp["y1"]
                x2, y2 = gp["x2"], gp["y2"]

                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                dx = abs(x2 - x1)
                dy = abs(y2 - y1)
                H_grass = 0.1

                line = f"""
      <link name="grass_patch_{idx}">
        <pose>{cx:.4f} {cy:.4f} 0.0 0 0 0</pose>
        <collision name="col">
          <pose>0 0 {-H_grass / 2:.4f} 0 0 0</pose>
          <geometry>
            <box><size>{dx:.4f} {dy:.4f} {H_grass:.4f}</size></box>
          </geometry>
        </collision>
        <visual name="vis">
          <pose>0 0 {-H_grass / 2:.4f} 0 0 0</pose>
          <geometry>
            <box><size>{dx:.4f} {dy:.4f} {H_grass:.4f}</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.6 0.3 1</ambient>
            <diffuse>0.3 0.6 0.3 1</diffuse>
          </material>
          <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
            <label>2</label> <!-- 2 = Grass -->
          </plugin>
        </visual>
      </link>"""
                grass_xml_lines.append(line)

            grass_network_xml = "\n".join(grass_xml_lines)
            grass_xml = f"""
    <model name="grass_network">
      <static>true</static>
{grass_network_xml}
    </model>"""

        # 2. GENERATE SIDEWALK INTERSECTIONS
        segments_xml_lines = []
        for idx, seg in enumerate(sidewalks):
            x, y = seg["x"], seg["y"]
            yaw = seg["yaw"]
            L = seg["length"]
            W = seg["width"]
            H = seg.get("height", 0.05)

            line = f"""
      <!-- Segment {idx}: Length {L:.2f}m, Width {W:.2f}m, Height {H:.2f}m -->
      <link name="segment_{idx}">
        <pose>{x:.4f} {y:.4f} 0.0 0 0 {yaw:.4f}</pose>
        <collision name="col">
          <pose>{L / 2:.4f} 0 {H / 2:.4f} 0 0 0</pose>
          <geometry>
            <box><size>{L:.4f} {W:.4f} {H:.4f}</size></box>
          </geometry>
        </collision>
        <visual name="vis">
          <pose>{L / 2:.4f} 0 {H / 2:.4f} 0 0 0</pose>
          <geometry>
            <box><size>{L:.4f} {W:.4f} {H:.4f}</size></box>
          </geometry>
          <material>
            <ambient>0.65 0.65 0.65 1</ambient>
            <diffuse>0.65 0.65 0.65 1</diffuse>
          </material>
          <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
            <label>1</label> <!-- 1 = Sidewalk -->
          </plugin>
        </visual>
      </link>"""
            segments_xml_lines.append(line)

        segments_xml = "\n".join(segments_xml_lines)

        # 3. GENERATE PREDEFINED SPAWN POINTS
        spawn_xml_lines = []
        for sp in spawn_points:
            x, y = sp["x"], sp["y"]
            yaw = sp["yaw"]
            sp_id = sp["id"]
            line = f"""
    <!-- Predefined Spawn Point {sp_id} -->
    <frame name="start_position_{sp_id}">
      <pose>{x:.4f} {y:.4f} 0.1000 0.0000 0.0000 {yaw:.4f}</pose>
    </frame>"""
            spawn_xml_lines.append(line)

        spawn_points_xml = "\n".join(spawn_xml_lines)

        sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="campus_sidewalk">

    <!-- ========================================== -->
    <!-- 1. SIMULATION PHYSICS & SYSTEM PLUGINS     -->
    <!-- ========================================== -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- ========================================== -->
    <!-- 2. LIGHTING & SUN                          -->
    <!-- ========================================== -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- ========================================== -->
    <!-- 3. GRASS MODEL                             -->
    <!-- ========================================== -->{grass_xml}

    <!-- ========================================== -->
    <!-- 4. SIDEWALK NETWORK                        -->
    <!-- ========================================== -->
    <model name="sidewalk_network">
      <static>true</static>
{segments_xml}
    </model>

    <!-- ========================================== -->
    <!-- 5. PREDEFINED SPAWN POINTS                 -->
    <!-- ========================================== -->{spawn_points_xml}

  </world>
</sdf>
"""
        try:
            with open(file_path, "w") as f:
                f.write(sdf_content)
            messagebox.showinfo("Export Successful", f"SDF Map saved to:\n{file_path}")
        except Exception as e:
            messagebox.showerror("Export Failed", f"Could not save file: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = SidewalkMapGenerator(root)
    root.mainloop()
