"""Native interactive desktop viewer for completed reBot telemetry CSV logs."""

from __future__ import annotations

import argparse
import math
import tkinter as tk
from collections import Counter
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable

import numpy as np

from .csv_analysis import (
    DEFAULT_SIGNAL_KEYS,
    JOINT_LABELS,
    SIGNAL_BY_KEY,
    SIGNAL_SPECS,
    TelemetryDataset,
    decimate_minmax,
)
from .csv_logger import JOINT_NAMES


BACKGROUND = "#e9edf1"
PANEL = "#f7f9fb"
PLOT_BACKGROUND = "#ffffff"
TEXT = "#17212b"
MUTED = "#687681"
GRID = "#dbe2e8"
ACCENT = "#167d8d"
CURVE_COLORS = (
    "#1565c0",
    "#d32f2f",
    "#2e7d32",
    "#7b1fa2",
    "#ef6c00",
    "#00838f",
    "#5d4037",
    "#c2185b",
    "#455a64",
    "#6a8e24",
    "#3949ab",
    "#ad1457",
)
SIGNAL_COLORS = {
    spec.key: CURVE_COLORS[index % len(CURVE_COLORS)]
    for index, spec in enumerate(SIGNAL_SPECS)
}
STATE_COLORS = {
    "waiting": "#9aa5ad",
    "idle": "#4d86bd",
    "active": "#3c9b68",
    "stale": "#d49b36",
    "hold": "#c65353",
}


def _format_number(value: float, *, digits: int = 4) -> str:
    if not math.isfinite(value):
        return "-"
    magnitude = abs(value)
    if magnitude >= 100_000 or (0.0 < magnitude < 0.001):
        return f"{value:.3e}"
    return f"{value:.{digits}f}"


def _nice_ticks(low: float, high: float, count: int = 6) -> list[float]:
    span = max(high - low, 1e-12)
    raw_step = span / max(count - 1, 1)
    power = 10.0 ** math.floor(math.log10(raw_step))
    fraction = raw_step / power
    if fraction <= 1.0:
        nice_fraction = 1.0
    elif fraction <= 2.0:
        nice_fraction = 2.0
    elif fraction <= 5.0:
        nice_fraction = 5.0
    else:
        nice_fraction = 10.0
    step = nice_fraction * power
    start = math.ceil(low / step) * step
    ticks: list[float] = []
    value = start
    while value <= high + step * 1e-6 and len(ticks) < 20:
        ticks.append(value)
        value += step
    return ticks


class TelemetryPlot(tk.Canvas):
    """Canvas plot with interval zoom, cursor inspection, and pan support."""

    LEFT = 76
    RIGHT = 24
    TOP = 35
    BOTTOM = 54

    def __init__(
        self,
        master: tk.Misc,
        dataset: TelemetryDataset,
        *,
        on_range_changed: Callable[[float, float], None],
        on_cursor_changed: Callable[[int], None],
    ) -> None:
        super().__init__(
            master,
            background=PLOT_BACKGROUND,
            highlightthickness=1,
            highlightbackground="#c9d2da",
            cursor="crosshair",
        )
        self.dataset = dataset
        self.selected_keys = list(DEFAULT_SIGNAL_KEYS)
        self.normalized = False
        self.view_start_s = float(dataset.time_s[0])
        self.view_end_s = float(dataset.time_s[-1])
        self.on_range_changed = on_range_changed
        self.on_cursor_changed = on_cursor_changed
        self._drag_start_x: float | None = None
        self._pan_start_x: float | None = None
        self._pan_initial_range: tuple[float, float] | None = None
        self._cursor_index: int | None = None
        self._redraw_after: str | None = None

        self.bind("<Configure>", self._schedule_redraw)
        self.bind("<ButtonPress-1>", self._start_zoom)
        self.bind("<B1-Motion>", self._drag_zoom)
        self.bind("<ButtonRelease-1>", self._finish_zoom)
        self.bind("<Double-Button-1>", lambda _event: self.reset_view())
        self.bind("<ButtonPress-2>", self._start_pan)
        self.bind("<B2-Motion>", self._drag_pan)
        self.bind("<ButtonRelease-2>", self._finish_pan)
        self.bind("<Button-3>", lambda _event: self.reset_view())
        self.bind("<MouseWheel>", self._wheel_zoom)
        self.bind("<Button-4>", self._wheel_zoom)
        self.bind("<Button-5>", self._wheel_zoom)
        self.bind("<Motion>", self._move_cursor)
        self.bind("<Leave>", lambda _event: self.delete("cursor_overlay"))

    def set_dataset(self, dataset: TelemetryDataset) -> None:
        self.dataset = dataset
        self.reset_view()

    def set_selected(self, keys: list[str]) -> None:
        self.selected_keys = keys
        self.redraw()

    def set_normalized(self, normalized: bool) -> None:
        self.normalized = bool(normalized)
        self.redraw()

    def reset_view(self) -> None:
        self.set_view(float(self.dataset.time_s[0]), float(self.dataset.time_s[-1]))

    def set_view(self, start_s: float, end_s: float) -> None:
        full_start = float(self.dataset.time_s[0])
        full_end = float(self.dataset.time_s[-1])
        full_span = max(full_end - full_start, 1e-6)
        minimum_span = max(full_span / max(self.dataset.row_count, 1) * 5.0, 1e-4)
        start, end = sorted((float(start_s), float(end_s)))
        if end - start < minimum_span:
            center = (start + end) * 0.5
            start = center - minimum_span * 0.5
            end = center + minimum_span * 0.5
        if start < full_start:
            end += full_start - start
            start = full_start
        if end > full_end:
            start -= end - full_end
            end = full_end
        start = max(start, full_start)
        end = min(end, full_end)
        if end <= start:
            start, end = full_start, max(full_end, full_start + 1e-6)
        self.view_start_s = start
        self.view_end_s = end
        self.redraw()
        self.on_range_changed(start, end)

    def _schedule_redraw(self, _event: tk.Event[tk.Misc]) -> None:
        if self._redraw_after is not None:
            self.after_cancel(self._redraw_after)
        self._redraw_after = self.after(30, self.redraw)

    def _plot_bounds(self) -> tuple[float, float, float, float]:
        return (
            float(self.LEFT),
            float(self.TOP),
            max(float(self.winfo_width() - self.RIGHT), float(self.LEFT + 10)),
            max(float(self.winfo_height() - self.BOTTOM), float(self.TOP + 10)),
        )

    def _inside_plot(self, x: float, y: float) -> bool:
        x0, y0, x1, y1 = self._plot_bounds()
        return x0 <= x <= x1 and y0 <= y <= y1

    def _time_from_x(self, x: float) -> float:
        x0, _y0, x1, _y1 = self._plot_bounds()
        ratio = min(max((x - x0) / max(x1 - x0, 1.0), 0.0), 1.0)
        return self.view_start_s + ratio * (self.view_end_s - self.view_start_s)

    def redraw(self) -> None:
        self._redraw_after = None
        self.delete("all")
        width = self.winfo_width()
        height = self.winfo_height()
        if width < 180 or height < 140:
            return
        x0, y0, x1, y1 = self._plot_bounds()
        self.create_rectangle(x0, y0, x1, y1, fill=PLOT_BACKGROUND, outline="")

        start, stop = self.dataset.range_indices(self.view_start_s, self.view_end_s)
        time_values = self.dataset.time_s[start:stop]
        plot_values: dict[str, np.ndarray] = {}
        finite_ranges: list[tuple[float, float]] = []
        for key in self.selected_keys:
            values = self.dataset.values[key][start:stop]
            finite = values[np.isfinite(values)]
            if not finite.size:
                plot_values[key] = values
                continue
            if self.normalized:
                low = float(np.min(finite))
                high = float(np.max(finite))
                center = (low + high) * 0.5
                half_span = max((high - low) * 0.5, 1e-12)
                transformed = (values - center) / half_span
                plot_values[key] = transformed
                finite_ranges.append((-1.0, 1.0))
            else:
                plot_values[key] = values
                finite_ranges.append((float(np.min(finite)), float(np.max(finite))))

        if finite_ranges:
            y_low = min(value[0] for value in finite_ranges)
            y_high = max(value[1] for value in finite_ranges)
            if math.isclose(y_low, y_high, rel_tol=0.0, abs_tol=1e-12):
                padding = max(abs(y_low) * 0.1, 1.0)
            else:
                padding = (y_high - y_low) * 0.08
            y_low -= padding
            y_high += padding
        else:
            y_low, y_high = -1.0, 1.0

        for tick in _nice_ticks(self.view_start_s, self.view_end_s, 7):
            px = x0 + (tick - self.view_start_s) / max(
                self.view_end_s - self.view_start_s, 1e-12
            ) * (x1 - x0)
            self.create_line(px, y0, px, y1, fill=GRID, width=1)
            self.create_text(
                px,
                y1 + 17,
                text=_format_number(tick, digits=3),
                fill=MUTED,
                font=("TkDefaultFont", 9),
            )
        for tick in _nice_ticks(y_low, y_high, 6):
            py = y1 - (tick - y_low) / max(y_high - y_low, 1e-12) * (y1 - y0)
            self.create_line(x0, py, x1, py, fill=GRID, width=1)
            self.create_text(
                x0 - 9,
                py,
                text=_format_number(tick, digits=3),
                anchor="e",
                fill=MUTED,
                font=("TkDefaultFont", 9),
            )

        self.create_line(x0, y1, x1, y1, fill="#83919c", width=1)
        self.create_line(x0, y0, x0, y1, fill="#83919c", width=1)
        self.create_text(
            (x0 + x1) * 0.5,
            height - 13,
            text="Elapsed time (s)",
            fill=TEXT,
            font=("TkDefaultFont", 9, "bold"),
        )
        units = {SIGNAL_BY_KEY[key].unit for key in self.selected_keys}
        if self.normalized:
            y_label = "Normalized per curve"
        elif len(units) == 1:
            y_label = next(iter(units))
        elif units:
            y_label = "Mixed units"
        else:
            y_label = "Value"
        self.create_text(
            17,
            (y0 + y1) * 0.5,
            text=y_label,
            angle=90,
            fill=TEXT,
            font=("TkDefaultFont", 9, "bold"),
        )

        max_points = max(int((x1 - x0) * 2), 100)
        for key in self.selected_keys:
            values = plot_values.get(key)
            if values is None:
                continue
            x_data, y_data = decimate_minmax(time_values, values, max_points)
            if x_data.size < 2:
                continue
            px = x0 + (x_data - self.view_start_s) / max(
                self.view_end_s - self.view_start_s, 1e-12
            ) * (x1 - x0)
            py = y1 - (y_data - y_low) / max(y_high - y_low, 1e-12) * (y1 - y0)
            coordinates = np.column_stack((px, py)).reshape(-1).tolist()
            self.create_line(
                *coordinates,
                fill=SIGNAL_COLORS[key],
                width=1.6,
                smooth=False,
            )

        self._draw_state_strip(start, stop, x0, x1, y1)
        if not self.selected_keys:
            self.create_text(
                (x0 + x1) * 0.5,
                (y0 + y1) * 0.5,
                text="Select one or more signals",
                fill=MUTED,
                font=("TkDefaultFont", 13),
            )
        elif len(units) > 1 and not self.normalized:
            self.create_text(
                x1 - 8,
                y0 + 13,
                text="Mixed units - enable normalization to compare shape",
                anchor="e",
                fill="#a05a16",
                font=("TkDefaultFont", 9, "bold"),
            )
        if self._cursor_index is not None:
            self._draw_cursor(self._cursor_index)

    def _draw_state_strip(
        self,
        start: int,
        stop: int,
        x0: float,
        x1: float,
        y1: float,
    ) -> None:
        if stop <= start:
            return
        states = self.dataset.state
        times = self.dataset.time_s
        segment_start = start
        state = states[start]
        for index in range(start + 1, stop + 1):
            changed = index == stop or states[index] != state
            if not changed:
                continue
            left_time = max(times[segment_start], self.view_start_s)
            right_time = self.view_end_s if index == stop else min(times[index], self.view_end_s)
            left = x0 + (left_time - self.view_start_s) / max(
                self.view_end_s - self.view_start_s, 1e-12
            ) * (x1 - x0)
            right = x0 + (right_time - self.view_start_s) / max(
                self.view_end_s - self.view_start_s, 1e-12
            ) * (x1 - x0)
            self.create_rectangle(
                left,
                y1 + 3,
                max(right, left + 1),
                y1 + 8,
                fill=STATE_COLORS.get(state.lower(), "#89949c"),
                outline="",
            )
            if index < stop:
                segment_start = index
                state = states[index]

    def _start_zoom(self, event: tk.Event[tk.Misc]) -> None:
        if self._inside_plot(event.x, event.y):
            self._drag_start_x = float(event.x)

    def _drag_zoom(self, event: tk.Event[tk.Misc]) -> None:
        if self._drag_start_x is None:
            return
        x0, y0, x1, y1 = self._plot_bounds()
        current = min(max(float(event.x), x0), x1)
        start = min(max(self._drag_start_x, x0), x1)
        self.delete("zoom_overlay")
        self.create_rectangle(
            start,
            y0,
            current,
            y1,
            fill="#a7d5db",
            stipple="gray25",
            outline=ACCENT,
            tags="zoom_overlay",
        )

    def _finish_zoom(self, event: tk.Event[tk.Misc]) -> None:
        if self._drag_start_x is None:
            return
        start_x = self._drag_start_x
        self._drag_start_x = None
        self.delete("zoom_overlay")
        if abs(float(event.x) - start_x) >= 8:
            self.set_view(self._time_from_x(start_x), self._time_from_x(float(event.x)))
        elif self._inside_plot(event.x, event.y):
            self._set_cursor_time(self._time_from_x(float(event.x)))

    def _start_pan(self, event: tk.Event[tk.Misc]) -> None:
        if self._inside_plot(event.x, event.y):
            self._pan_start_x = float(event.x)
            self._pan_initial_range = (self.view_start_s, self.view_end_s)
            self.configure(cursor="fleur")

    def _drag_pan(self, event: tk.Event[tk.Misc]) -> None:
        if self._pan_start_x is None or self._pan_initial_range is None:
            return
        x0, _y0, x1, _y1 = self._plot_bounds()
        start, end = self._pan_initial_range
        shift = -(float(event.x) - self._pan_start_x) / max(x1 - x0, 1.0) * (end - start)
        self.set_view(start + shift, end + shift)

    def _finish_pan(self, _event: tk.Event[tk.Misc]) -> None:
        self._pan_start_x = None
        self._pan_initial_range = None
        self.configure(cursor="crosshair")

    def _wheel_zoom(self, event: tk.Event[tk.Misc]) -> str:
        if not self._inside_plot(event.x, event.y):
            return "break"
        direction = getattr(event, "delta", 0)
        if getattr(event, "num", 0) == 4:
            direction = 1
        elif getattr(event, "num", 0) == 5:
            direction = -1
        factor = 0.8 if direction > 0 else 1.25
        center = self._time_from_x(float(event.x))
        start = center + (self.view_start_s - center) * factor
        end = center + (self.view_end_s - center) * factor
        self.set_view(start, end)
        return "break"

    def _move_cursor(self, event: tk.Event[tk.Misc]) -> None:
        if self._drag_start_x is not None or self._pan_start_x is not None:
            return
        if self._inside_plot(event.x, event.y):
            self._set_cursor_time(self._time_from_x(float(event.x)))

    def _set_cursor_time(self, time_s: float) -> None:
        index = self.dataset.index_at(time_s)
        if index == self._cursor_index:
            return
        self._cursor_index = index
        self._draw_cursor(index)
        self.on_cursor_changed(index)

    def _draw_cursor(self, index: int) -> None:
        self.delete("cursor_overlay")
        x0, y0, x1, y1 = self._plot_bounds()
        time_s = float(self.dataset.time_s[index])
        if not self.view_start_s <= time_s <= self.view_end_s:
            return
        x = x0 + (time_s - self.view_start_s) / max(
            self.view_end_s - self.view_start_s, 1e-12
        ) * (x1 - x0)
        self.create_line(
            x,
            y0,
            x,
            y1,
            fill="#26343e",
            dash=(3, 3),
            width=1,
            tags="cursor_overlay",
        )
        self.create_text(
            min(max(x + 7, x0 + 4), x1 - 4),
            y0 + 8,
            text=f"t={time_s:.4f}s",
            anchor="nw" if x < (x0 + x1) * 0.72 else "ne",
            fill=TEXT,
            font=("TkDefaultFont", 9, "bold"),
            tags="cursor_overlay",
        )


class CSVAnalysisApp:
    def __init__(self, root: tk.Tk, dataset: TelemetryDataset) -> None:
        self.root = root
        self.dataset = dataset
        self.selected_keys = list(DEFAULT_SIGNAL_KEYS)
        self.normalize_var = tk.BooleanVar(value=False)
        self.range_start_var = tk.StringVar()
        self.range_end_var = tk.StringVar()
        self.summary_var = tk.StringVar()
        self.selection_count_var = tk.StringVar()
        self.cursor_summary_var = tk.StringVar(value="Move over the plot to inspect a frame")
        self._leaf_to_key: dict[str, str] = {}
        self._key_to_leaf: dict[str, str] = {}
        self._group_items: dict[str, str] = {}
        self._group_to_keys: dict[str, list[str]] = {}

        self._configure_root()
        self._build_layout()
        self._set_dataset(dataset)

    def _configure_root(self) -> None:
        self.root.title("reBot VR CSV Analysis")
        self.root.minsize(1100, 680)
        self.root.configure(background=BACKGROUND)
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure(".", font=("TkDefaultFont", 10), background=BACKGROUND)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure(
            "Title.TLabel",
            font=("TkDefaultFont", 14, "bold"),
            foreground=TEXT,
            background=BACKGROUND,
        )
        style.configure("Summary.TLabel", foreground=MUTED, background=BACKGROUND)
        style.configure(
            "Treeview",
            rowheight=24,
            background=PLOT_BACKGROUND,
            fieldbackground=PLOT_BACKGROUND,
            foreground=TEXT,
            borderwidth=0,
        )
        style.configure(
            "Treeview.Heading",
            font=("TkDefaultFont", 9, "bold"),
            padding=(8, 7),
            foreground=TEXT,
        )
        style.configure("TButton", padding=(9, 6))
        style.configure("TEntry", padding=(5, 5))
        style.configure("TCombobox", padding=(5, 5))
        style.configure("TCheckbutton", padding=(3, 5))
        style.configure("TNotebook.Tab", padding=(12, 7))
        style.configure("Accent.TButton", foreground="#ffffff", background=ACCENT)
        style.map("Accent.TButton", background=[("active", "#116978")])

    def _build_layout(self) -> None:
        top = ttk.Frame(self.root, padding=(16, 13, 16, 10))
        top.pack(fill="x")
        ttk.Label(top, text="reBot VR telemetry analysis", style="Title.TLabel").pack(side="left")
        ttk.Button(top, text="Open CSV", command=self._open_file).pack(side="right")
        ttk.Label(top, textvariable=self.summary_var, style="Summary.TLabel").pack(side="right", padx=16)

        body = ttk.Panedwindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        controls = ttk.Frame(body, style="Panel.TFrame", padding=12, width=350)
        body.add(controls, weight=0)
        content = ttk.Frame(body, style="Panel.TFrame")
        body.add(content, weight=1)

        signal_header = ttk.Frame(controls, style="Panel.TFrame")
        signal_header.pack(fill="x", pady=(0, 8))
        ttk.Label(
            signal_header,
            text="Signals",
            font=("TkDefaultFont", 11, "bold"),
            background=PANEL,
        ).pack(side="left")
        ttk.Label(
            signal_header,
            textvariable=self.selection_count_var,
            foreground=MUTED,
            background=PANEL,
        ).pack(side="right")
        tree_frame = ttk.Frame(controls)
        tree_frame.pack(fill="both", expand=True)
        self.signal_tree = ttk.Treeview(
            tree_frame,
            show="tree",
            selectmode="none",
            height=20,
            padding=(4, 4),
        )
        tree_scroll = ttk.Scrollbar(tree_frame, orient="vertical", command=self.signal_tree.yview)
        self.signal_tree.configure(yscrollcommand=tree_scroll.set)
        self.signal_tree.pack(side="left", fill="both", expand=True)
        tree_scroll.pack(side="right", fill="y")
        self.signal_tree.bind("<ButtonRelease-1>", self._toggle_tree_item)
        self._populate_signal_tree()

        preset_frame = ttk.Frame(controls, style="Panel.TFrame")
        preset_frame.pack(fill="x", pady=(8, 4))
        joint_values = tuple(JOINT_LABELS[joint] for joint in JOINT_NAMES)
        self.joint_var = tk.StringVar(value=joint_values[0])
        joint_box = ttk.Combobox(
            preset_frame,
            textvariable=self.joint_var,
            values=joint_values,
            state="readonly",
            width=20,
        )
        joint_box.pack(side="left", fill="x", expand=True)
        ttk.Button(preset_frame, text="A/T/C", width=6, command=self._select_joint_trio).pack(side="left", padx=(5, 0))

        preset_buttons = ttk.Frame(controls, style="Panel.TFrame")
        preset_buttons.pack(fill="x", pady=3)
        ttk.Button(preset_buttons, text="All actual", command=self._select_all_actual).pack(side="left", fill="x", expand=True)
        ttk.Button(preset_buttons, text="Errors", command=self._select_errors).pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(preset_buttons, text="IK", command=self._select_diagnostics).pack(side="left", fill="x", expand=True)
        preset_buttons_2 = ttk.Frame(controls, style="Panel.TFrame")
        preset_buttons_2.pack(fill="x", pady=3)
        ttk.Button(preset_buttons_2, text="Select all", command=lambda: self._set_selected([spec.key for spec in SIGNAL_SPECS])).pack(side="left", fill="x", expand=True)
        ttk.Button(preset_buttons_2, text="Clear", command=lambda: self._set_selected([])).pack(side="left", fill="x", expand=True, padx=(4, 0))

        ttk.Checkbutton(
            controls,
            text="Normalize each curve",
            variable=self.normalize_var,
            command=lambda: self.plot.set_normalized(self.normalize_var.get()),
        ).pack(anchor="w", pady=(8, 5))

        range_frame = ttk.LabelFrame(controls, text="Visible time range (s)", padding=7)
        range_frame.pack(fill="x", pady=(5, 0))
        ttk.Label(range_frame, text="From").grid(row=0, column=0, sticky="w")
        ttk.Entry(range_frame, textvariable=self.range_start_var, width=10).grid(row=0, column=1, padx=(4, 8), sticky="ew")
        ttk.Label(range_frame, text="To").grid(row=0, column=2, sticky="w")
        ttk.Entry(range_frame, textvariable=self.range_end_var, width=10).grid(row=0, column=3, padx=(4, 0), sticky="ew")
        ttk.Button(range_frame, text="Apply", command=self._apply_range).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(range_frame, text="Reset", command=self.plot_reset).grid(row=1, column=2, columnspan=2, sticky="ew", padx=(4, 0), pady=(6, 0))
        range_frame.columnconfigure(1, weight=1)
        range_frame.columnconfigure(3, weight=1)

        self.plot = TelemetryPlot(
            content,
            self.dataset,
            on_range_changed=self._range_changed,
            on_cursor_changed=self._cursor_changed,
        )
        self.plot.pack(fill="both", expand=True, padx=8, pady=(8, 4))

        details = ttk.Notebook(content)
        details.pack(fill="x", padx=8, pady=(3, 8))
        cursor_tab = ttk.Frame(details, padding=6)
        stats_tab = ttk.Frame(details, padding=6)
        details.add(cursor_tab, text="Cursor values")
        details.add(stats_tab, text="Visible-range statistics")

        ttk.Label(cursor_tab, textvariable=self.cursor_summary_var).pack(fill="x", pady=(0, 5))
        self.cursor_table = ttk.Treeview(
            cursor_tab,
            columns=("signal", "value", "unit"),
            show="headings",
            height=5,
        )
        for column, title, width in (
            ("signal", "Signal", 360),
            ("value", "Value", 140),
            ("unit", "Unit", 90),
        ):
            self.cursor_table.heading(column, text=title)
            self.cursor_table.column(column, width=width, anchor="e" if column == "value" else "w")
        self.cursor_table.pack(fill="x")

        stats_frame = ttk.Frame(stats_tab)
        stats_frame.pack(fill="both", expand=True)
        columns = ("signal", "unit", "count", "min", "max", "mean", "rms", "peak", "peak_time")
        self.stats_table = ttk.Treeview(stats_frame, columns=columns, show="headings", height=6)
        headings = {
            "signal": "Signal",
            "unit": "Unit",
            "count": "N",
            "min": "Min",
            "max": "Max",
            "mean": "Mean",
            "rms": "RMS",
            "peak": "Max |value|",
            "peak_time": "Peak time (s)",
        }
        for column in columns:
            width = 260 if column == "signal" else 100
            self.stats_table.heading(column, text=headings[column])
            self.stats_table.column(column, width=width, anchor="w" if column == "signal" else "e")
        stats_y = ttk.Scrollbar(stats_frame, orient="vertical", command=self.stats_table.yview)
        stats_x = ttk.Scrollbar(stats_frame, orient="horizontal", command=self.stats_table.xview)
        self.stats_table.configure(yscrollcommand=stats_y.set, xscrollcommand=stats_x.set)
        self.stats_table.grid(row=0, column=0, sticky="nsew")
        stats_y.grid(row=0, column=1, sticky="ns")
        stats_x.grid(row=1, column=0, sticky="ew")
        stats_frame.columnconfigure(0, weight=1)
        stats_frame.rowconfigure(0, weight=1)

    def _populate_signal_tree(self) -> None:
        groups: dict[str, str] = {}
        for spec in SIGNAL_SPECS:
            if spec.group not in groups:
                groups[spec.group] = self.signal_tree.insert(
                    "",
                    "end",
                    text=spec.group,
                    open=spec.group == "Joint position",
                    tags=("group",),
                )
                self._group_items[spec.group] = groups[spec.group]
                self._group_to_keys[spec.group] = []
            leaf = self.signal_tree.insert(groups[spec.group], "end", text="")
            self._leaf_to_key[leaf] = spec.key
            self._key_to_leaf[spec.key] = leaf
            self._group_to_keys[spec.group].append(spec.key)
        self._refresh_tree_labels()

    def _refresh_tree_labels(self) -> None:
        selected = set(self.selected_keys)
        for key, leaf in self._key_to_leaf.items():
            spec = SIGNAL_BY_KEY[key]
            if key in selected:
                marker = "[x]"
            else:
                marker = "[ ]"
            self.signal_tree.item(leaf, text=f"{marker} {spec.label}", tags=())
        for group, item in self._group_items.items():
            keys = self._group_to_keys[group]
            count = sum(key in selected for key in keys)
            self.signal_tree.item(item, text=f"{group}   {count}/{len(keys)}")
        self.selection_count_var.set(f"{len(selected)} selected")

    def _toggle_tree_item(self, event: tk.Event[tk.Misc]) -> None:
        item = self.signal_tree.identify_row(event.y)
        key = self._leaf_to_key.get(item)
        if key is None:
            element = self.signal_tree.identify("element", event.x, event.y)
            if "indicator" in element:
                return
            group = next(
                (name for name, group_item in self._group_items.items() if group_item == item),
                None,
            )
            if group is not None:
                group_keys = self._group_to_keys[group]
                selected = list(self.selected_keys)
                if all(key in selected for key in group_keys):
                    selected = [key for key in selected if key not in group_keys]
                else:
                    selected.extend(key for key in group_keys if key not in selected)
                self._set_selected(selected)
            return
        selected = list(self.selected_keys)
        if key in selected:
            selected.remove(key)
        else:
            selected.append(key)
        self._set_selected(selected)

    def _set_selected(self, keys: list[str]) -> None:
        unique = list(dict.fromkeys(key for key in keys if key in SIGNAL_BY_KEY))
        self.selected_keys = unique
        self._refresh_tree_labels()
        self.plot.set_selected(unique)
        self._update_statistics()
        if self.plot._cursor_index is not None:
            self._cursor_changed(self.plot._cursor_index)

    def _select_joint_trio(self) -> None:
        label = self.joint_var.get()
        joint = next((name for name, value in JOINT_LABELS.items() if value == label), JOINT_NAMES[0])
        self._set_selected([f"{kind}_{joint}_deg" for kind in ("actual", "target", "command")])

    def _select_all_actual(self) -> None:
        self._set_selected([f"actual_{joint}_deg" for joint in JOINT_NAMES])

    def _select_errors(self) -> None:
        self._set_selected([
            spec.key for spec in SIGNAL_SPECS if spec.group == "Tracking error"
        ])

    def _select_diagnostics(self) -> None:
        self._set_selected([
            spec.key for spec in SIGNAL_SPECS if spec.group == "IK and control diagnostics"
        ])

    def _set_dataset(self, dataset: TelemetryDataset) -> None:
        self.dataset = dataset
        self.plot.set_dataset(dataset)
        self.plot.set_selected(self.selected_keys)
        states = Counter(dataset.state)
        state_text = ", ".join(f"{name}:{count}" for name, count in states.most_common())
        frequency = dataset.median_sample_hz
        frequency_text = "-" if frequency is None else f"{frequency:.1f} Hz"
        self.summary_var.set(
            f"{dataset.path.name}  |  {dataset.row_count:,} rows  |  "
            f"{dataset.duration_s:.2f} s  |  median {frequency_text}  |  {state_text}"
        )
        self.root.title(f"reBot VR CSV Analysis - {dataset.path.name}")
        self._range_changed(float(dataset.time_s[0]), float(dataset.time_s[-1]))
        self._cursor_changed(0)

    def _open_file(self) -> None:
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Open reBot telemetry CSV",
            initialdir=str(self.dataset.path.parent),
            filetypes=(("CSV files", "*.csv"), ("All files", "*")),
        )
        if not selected:
            return
        try:
            dataset = TelemetryDataset.load(selected)
        except ValueError as exc:
            messagebox.showerror("Cannot open CSV", str(exc), parent=self.root)
            return
        self._set_dataset(dataset)

    def _apply_range(self) -> None:
        try:
            start = float(self.range_start_var.get())
            end = float(self.range_end_var.get())
            if not math.isfinite(start) or not math.isfinite(end) or end <= start:
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "Invalid time range",
                "From and To must be finite numbers with To greater than From.",
                parent=self.root,
            )
            return
        self.plot.set_view(start, end)

    def plot_reset(self) -> None:
        self.plot.reset_view()

    def _range_changed(self, start_s: float, end_s: float) -> None:
        self.range_start_var.set(f"{start_s:.6f}")
        self.range_end_var.set(f"{end_s:.6f}")
        self._update_statistics()

    def _update_statistics(self) -> None:
        if not hasattr(self, "stats_table"):
            return
        self.stats_table.delete(*self.stats_table.get_children())
        start = self.plot.view_start_s
        end = self.plot.view_end_s
        for key in self.selected_keys:
            spec = SIGNAL_BY_KEY[key]
            stats = self.dataset.statistics(key, start, end)
            if stats is None:
                values = (spec.label, spec.unit, "0", "-", "-", "-", "-", "-", "-")
            else:
                values = (
                    spec.label,
                    spec.unit,
                    f"{stats.count:,}",
                    _format_number(stats.minimum),
                    _format_number(stats.maximum),
                    _format_number(stats.mean),
                    _format_number(stats.rms),
                    _format_number(stats.peak_abs),
                    f"{stats.peak_time_s:.6f}",
                )
            self.stats_table.insert("", "end", values=values)

    def _cursor_changed(self, index: int) -> None:
        timestamp_ns = int(self.dataset.timestamp_ns[index])
        local_time = datetime.fromtimestamp(timestamp_ns * 1e-9).astimezone()
        success = self.dataset.values["ik_success"][index]
        success_text = "-" if not math.isfinite(success) else ("yes" if success >= 0.5 else "no")
        reason = self.dataset.ik_reason[index] or "-"
        self.cursor_summary_var.set(
            f"t={self.dataset.time_s[index]:.6f}s  |  {local_time.isoformat(timespec='milliseconds')}  |  "
            f"state={self.dataset.state[index]}  |  IK={success_text}  |  reason={reason}"
        )
        self.cursor_table.delete(*self.cursor_table.get_children())
        for key in self.selected_keys:
            spec = SIGNAL_BY_KEY[key]
            value = float(self.dataset.values[key][index])
            self.cursor_table.insert(
                "",
                "end",
                values=(spec.label, _format_number(value, digits=6), spec.unit),
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Interactively analyze a completed reBot teleoperation CSV log.",
    )
    parser.add_argument("csv_path", type=Path, help="CSV file created by --csv-log")
    parser.add_argument(
        "--geometry",
        default="1500x900",
        help="initial desktop window geometry (default: 1500x900)",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    try:
        dataset = TelemetryDataset.load(args.csv_path)
    except ValueError as exc:
        raise SystemExit(str(exc)) from None
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise SystemExit(f"cannot open desktop display: {exc}") from None
    root.geometry(args.geometry)
    CSVAnalysisApp(root, dataset)
    root.mainloop()


__all__ = ["CSVAnalysisApp", "TelemetryPlot", "build_parser", "main"]


if __name__ == "__main__":
    main()
