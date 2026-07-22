#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路网可视化绘制工具

一个独立运行的GUI工具，用于在地图底图上可视化绘制和编辑路网。
保存为标准GeoJSON格式，可直接被route_planner功能包加载。

功能：
- 加载地图底图（.pgm/.png）显示真实环境
- 添加节点（点击添加）和边（依次点击两个节点连线）
- 拖拽移动节点
- 删除节点/边
- 编辑节点名称
- 导入/导出GeoJSON文件
- 自动吸附网格、显示坐标

使用方法：
  python route_editor.py
  python route_editor.py --map /path/to/MYCAR.pgm --yaml /path/to/MYCAR.yaml
  python route_editor.py --geojson /path/to/routes.geojson

依赖：Pillow（pip install Pillow）
"""

import tkinter as tk
from tkinter import messagebox, filedialog, simpledialog
import json
import math
import os
import argparse
import re

from PIL import Image, ImageTk


# ============================================================
#  数据模型
# ============================================================

class RouteNode:
    _counter = 0

    def __init__(self, x, y, name='', node_id=''):
        RouteNode._counter += 1
        self.id = node_id or str(RouteNode._counter)
        self.name = name or self.id
        self.x = float(x)
        self.y = float(y)
        self.canvas_id = None
        self.text_id = None
        self.properties = {}

    def __repr__(self):
        return f"Node({self.id}, {self.name}, {self.x:.2f}, {self.y:.2f})"


class RouteEdge:
    _counter = 0

    def __init__(self, start_id, end_id, bidirectional=True, name='', edge_id=''):
        RouteEdge._counter += 1
        self.id = edge_id or f'e{RouteEdge._counter}'
        self.start_node = start_id
        self.end_node = end_id
        self.bidirectional = bidirectional
        self.name = name or f'{start_id}->{end_id}'
        self.weight = 0.0
        self.canvas_id = None
        self.properties = {}

    def __repr__(self):
        return f"Edge({self.id}, {self.start_node}->{self.end_node})"


class RouteGraphModel:
    def __init__(self):
        self.nodes = {}
        self.edges = {}
        self.properties = {'name': '未命名路网', 'description': '', 'version': '1.0'}

    def add_node(self, x, y, name='', node_id=''):
        if node_id and node_id in self.nodes:
            return None
        node = RouteNode(x, y, name, node_id)
        if node_id:
            node.id = node_id
        self.nodes[node.id] = node
        return node

    def remove_node(self, node_id):
        to_remove = [eid for eid, e in self.edges.items()
                     if e.start_node == node_id or e.end_node == node_id]
        for eid in to_remove:
            del self.edges[eid]
        if node_id in self.nodes:
            del self.nodes[node_id]

    def add_edge(self, start_id, end_id, bidirectional=True, name=''):
        if start_id not in self.nodes or end_id not in self.nodes:
            return None
        if start_id == end_id:
            return None
        edge = RouteEdge(start_id, end_id, bidirectional, name)
        s = self.nodes[start_id]
        e = self.nodes[end_id]
        edge.weight = math.sqrt((s.x - e.x)**2 + (s.y - e.y)**2)
        self.edges[edge.id] = edge
        return edge

    def remove_edge(self, edge_id):
        if edge_id in self.edges:
            del self.edges[edge_id]

    def clear(self):
        self.nodes.clear()
        self.edges.clear()

    def to_geojson(self) -> dict:
        features = []
        for node in self.nodes.values():
            feature = {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [round(node.x, 4), round(node.y, 4), 0.0]},
                "properties": {
                    "id": node.id,
                    "name": node.name,
                    "type": "node",
                    **node.properties
                }
            }
            features.append(feature)

        for edge in self.edges.values():
            s = self.nodes.get(edge.start_node)
            e = self.nodes.get(edge.end_node)
            if s and e:
                feature = {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [
                            [round(s.x, 4), round(s.y, 4), 0.0],
                            [round(e.x, 4), round(e.y, 4), 0.0]
                        ]
                    },
                    "properties": {
                        "id": edge.id,
                        "start_node": edge.start_node,
                        "end_node": edge.end_node,
                        "type": "edge",
                        "name": edge.name,
                        "bidirectional": edge.bidirectional,
                        "weight": round(edge.weight, 4),
                        **edge.properties
                    }
                }
                features.append(feature)

        return {
            "type": "FeatureCollection",
            "properties": self.properties,
            "features": features
        }

    def from_geojson(self, data: dict):
        self.clear()
        RouteNode._counter = 0
        RouteEdge._counter = 0
        self.properties = data.get('properties', {})

        for feature in data.get('features', []):
            if feature.get('type') != 'Feature':
                continue
            geometry = feature.get('geometry', {})
            props = feature.get('properties', {})
            geo_type = geometry.get('type', '')
            feat_type = props.get('type', '').lower()

            if geo_type == 'Point' or feat_type == 'node':
                coords = geometry.get('coordinates', [0, 0])
                node_id = str(props.get('id', ''))
                name = props.get('name', node_id)
                node = self.add_node(coords[0], coords[1] if len(coords) > 1 else 0,
                                     name, node_id)
                if node and 'zone' in props:
                    node.properties['zone'] = props['zone']

            elif geo_type == 'LineString' or feat_type == 'edge':
                start_node = str(props.get('start_node', ''))
                end_node = str(props.get('end_node', ''))
                bidir = props.get('bidirectional', True)
                name = props.get('name', '')
                edge_id = str(props.get('id', ''))
                edge = self.add_edge(start_node, end_node, bidir, name)
                if edge:
                    if edge_id:
                        edge.id = edge_id
                    if 'zone' in props:
                        edge.properties['zone'] = props['zone']


# ============================================================
#  主编辑器
# ============================================================

class RouteEditor:
    NODE_RADIUS = 8
    HIT_RADIUS = 12

    def __init__(self, root, map_path=None, yaml_path=None, geojson_path=None):
        self.root = root
        self.root.title("路网可视化绘制工具")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')

        self.model = RouteGraphModel()

        # 地图数据
        self.map_pil = None           # 原始PIL Image (RGBA)
        self.map_photo = None         # Tk PhotoImage
        self.map_width_px = 0         # 像素宽
        self.map_height_px = 0        # 像素高
        self.map_res = 0.05           # 分辨率 (m/px)
        self.map_origin_x = 0.0       # 原点x (m)
        self.map_origin_y = 0.0       # 原点y (m)
        self.map_loaded = False

        # 状态
        self.mode = 'select'
        self.edge_start = None
        self.dragging_node = None
        self.selected_node = None
        self.selected_edge = None
        self.hover_node_id = None

        # 视图
        self.zoom = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0

        self.COLORS = {
            'bg': '#2b2b2b',
            'canvas_bg': '#1a1a2e',
            'node': '#e74c3c',
            'node_selected': '#ff6600',
            'node_hover': '#ff9900',
            'node_text': '#ffffff',
            'edge': '#2ecc71',
            'edge_selected': '#ffcc00',
            'grid': '#333355',
            'toolbar_bg': '#333333',
            'status_bg': '#1e1e1e',
            'status_text': '#aaaaaa',
        }

        self._build_ui()
        self._bind_events()

        if map_path or yaml_path:
            self._load_map(map_path, yaml_path)
        if geojson_path:
            self._load_geojson_file(geojson_path)
        else:
            self._fit_view()

        self._redraw()

    # ---- UI ----

    def _build_ui(self):
        toolbar = tk.Frame(self.root, bg=self.COLORS['toolbar_bg'], height=45)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        toolbar.pack_propagate(False)

        btn_style = {'bg': '#444', 'fg': 'white', 'relief': 'flat',
                     'padx': 10, 'pady': 4, 'font': ('Arial', 10)}

        self.btn_select = tk.Button(toolbar, text="[S] 选择/拖拽",
                                     command=lambda: self._set_mode('select'), **btn_style)
        self.btn_select.pack(side=tk.LEFT, padx=2, pady=6)

        self.btn_add_node = tk.Button(toolbar, text="[N] 添加节点",
                                      command=lambda: self._set_mode('add_node'), **btn_style)
        self.btn_add_node.pack(side=tk.LEFT, padx=2, pady=6)

        self.btn_add_edge = tk.Button(toolbar, text="[E] 添加边",
                                      command=lambda: self._set_mode('add_edge'), **btn_style)
        self.btn_add_edge.pack(side=tk.LEFT, padx=2, pady=6)

        sep = tk.Frame(toolbar, bg='#555', width=2)
        sep.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=6)

        self.btn_add_oneway = tk.Button(toolbar, text="[O] 添加单行线",
                                           command=lambda: self._set_mode('add_oneway_edge'), **btn_style)
        self.btn_add_oneway.pack(side=tk.LEFT, padx=2, pady=6)
        self.btn_toggle_dir = tk.Button(toolbar, text="单行/双向",
                                             command=self._toggle_edge_dir, **btn_style)
        self.btn_toggle_dir.pack(side=tk.LEFT, padx=2, pady=6)
        self.btn_flip_dir = tk.Button(toolbar, text="翻转方向",
                                          command=self._flip_edge_dir, **btn_style)
        self.btn_flip_dir.pack(side=tk.LEFT, padx=2, pady=6)

        sep4 = tk.Frame(toolbar, bg='#555', width=2)
        sep4.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=6)

        tk.Button(toolbar, text="[Del] 删除", command=self._delete_selected,
                   **btn_style).pack(side=tk.LEFT, padx=2, pady=6)
        tk.Button(toolbar, text="重命名", command=self._rename_selected,
                   **btn_style).pack(side=tk.LEFT, padx=2, pady=6)

        sep2 = tk.Frame(toolbar, bg='#555', width=2)
        sep2.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=6)

        tk.Button(toolbar, text="加载地图", command=self._browse_map,
                   **btn_style).pack(side=tk.LEFT, padx=2, pady=6)
        tk.Button(toolbar, text="加载路网", command=self._browse_geojson,
                   **btn_style).pack(side=tk.LEFT, padx=2, pady=6)
        tk.Button(toolbar, text="保存GeoJSON", command=self._save_geojson,
                   bg='#2e7d32', fg='white', relief='flat',
                   padx=10, pady=4, font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=2, pady=6)

        sep3 = tk.Frame(toolbar, bg='#555', width=2)
        sep3.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=6)

        tk.Button(toolbar, text="适应视图", command=self._fit_view,
                   **btn_style).pack(side=tk.LEFT, padx=2, pady=6)
        tk.Button(toolbar, text="清空", command=self._clear_all,
                   bg='#c62828', fg='white', relief='flat',
                   padx=10, pady=4, font=('Arial', 10)).pack(side=tk.LEFT, padx=2, pady=6)

        tk.Button(toolbar, text="+", width=3, command=lambda: self._zoom_step(1.2),
                   **btn_style).pack(side=tk.RIGHT, padx=2, pady=6)
        self.zoom_label = tk.Label(toolbar, text="100%", bg=self.COLORS['toolbar_bg'],
                                    fg='white', font=('Arial', 10))
        self.zoom_label.pack(side=tk.RIGHT, padx=2)
        tk.Button(toolbar, text="-", width=3, command=lambda: self._zoom_step(1/1.2),
                   **btn_style).pack(side=tk.RIGHT, padx=2, pady=6)

        self.canvas = tk.Canvas(self.root, bg=self.COLORS['canvas_bg'], highlightthickness=0)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        status_bar = tk.Frame(self.root, bg=self.COLORS['status_bg'], height=28)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        status_bar.pack_propagate(False)

        self.status_label = tk.Label(status_bar, text="就绪 | 模式: 选择",
                                      bg=self.COLORS['status_bg'],
                                      fg=self.COLORS['status_text'],
                                      font=('Consolas', 10), anchor='w')
        self.status_label.pack(side=tk.LEFT, padx=10)

        self.coord_label = tk.Label(status_bar, text="坐标: --",
                                      bg=self.COLORS['status_bg'],
                                      fg=self.COLORS['status_text'],
                                      font=('Consolas', 10), anchor='e')
        self.coord_label.pack(side=tk.RIGHT, padx=10)

        self.info_label = tk.Label(status_bar, text="节点: 0  边: 0",
                                     bg=self.COLORS['status_bg'],
                                     fg=self.COLORS['status_text'],
                                     font=('Consolas', 10), anchor='e')
        self.info_label.pack(side=tk.RIGHT, padx=10)

    def _bind_events(self):
        self.canvas.bind('<Button-1>', self._on_click)
        self.canvas.bind('<B1-Motion>', self._on_drag)
        self.canvas.bind('<ButtonRelease-1>', self._on_release)
        self.canvas.bind('<Motion>', self._on_motion)
        self.canvas.bind('<Button-3>', lambda e: self._cancel_action())
        self.canvas.bind('<MouseWheel>', self._on_scroll)
        self.root.bind('<Delete>', lambda e: self._delete_selected())
        self.root.bind('<s>', lambda e: self._set_mode('select'))
        self.root.bind('<n>', lambda e: self._set_mode('add_node'))
        self.root.bind('<e>', lambda e: self._set_mode('add_edge'))
        self.root.bind('<o>', lambda e: self._set_mode('add_oneway_edge'))
        self.root.bind('<Escape>', lambda e: self._cancel_action())
        self.canvas.bind('<Configure>', lambda e: self._redraw())

    # ---- 坐标变换 ----

    def _map_to_canvas(self, mx, my):
        """地图坐标(米) -> Canvas像素坐标"""
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        cx = cw / 2 + (mx - self.offset_x) * self.zoom
        cy = ch / 2 - (my - self.offset_y) * self.zoom
        return cx, cy

    def _canvas_to_map(self, cx, cy):
        """Canvas像素坐标 -> 地图坐标(米)"""
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        mx = (cx - cw / 2) / self.zoom + self.offset_x
        my = -(cy - ch / 2) / self.zoom + self.offset_y
        return mx, my

    # ---- 事件 ----

    def _on_click(self, event):
        mx, my = self._canvas_to_map(event.x, event.y)

        if self.mode == 'add_node':
            node = self.model.add_node(round(mx, 2), round(my, 2))
            if node:
                self._redraw()
                self._update_status(f"添加节点 {node.id} ({mx:.2f}, {my:.2f})")
        elif self.mode == 'add_edge':
            node = self._hit_test_node(event.x, event.y)
            if node:
                if self.edge_start is None:
                    self.edge_start = node.id
                    self._redraw()
                    self._update_status(f"选择起点 {node.id}，请点击终点节点")
                else:
                    if node.id != self.edge_start:
                        edge = self.model.add_edge(self.edge_start, node.id)
                        if edge:
                            self._update_status(f"添加边 {edge.id}: {edge.start_node}->{edge.end_node}")
                    self.edge_start = None
                    self._redraw()
        elif self.mode == 'add_oneway_edge':
            node = self._hit_test_node(event.x, event.y)
            if node:
                if self.edge_start is None:
                    self.edge_start = node.id
                    self._redraw()
                    self._update_status(f"选择单行线起点 {node.id}，请点击终点节点")
                else:
                    if node.id != self.edge_start:
                        edge = self.model.add_edge(
                            self.edge_start, node.id, bidirectional=False)
                        if edge:
                            self._update_status(
                                f"添加单行线 {edge.id}: "
                                f"{edge.start_node}->{edge.end_node} (仅此方向通行)")
                    self.edge_start = None
                    self._redraw()
        elif self.mode == 'select':
            node = self._hit_test_node(event.x, event.y)
            edge = self._hit_test_edge(event.x, event.y) if not node else None
            self.selected_node = node.id if node else None
            self.selected_edge = edge.id if edge else None
            if node:
                self.dragging_node = node.id
            self._redraw()

    def _on_drag(self, event):
        if self.mode == 'select' and self.dragging_node:
            mx, my = self._canvas_to_map(event.x, event.y)
            node = self.model.nodes.get(self.dragging_node)
            if node:
                node.x = round(mx, 2)
                node.y = round(my, 2)
                for edge in self.model.edges.values():
                    if edge.start_node == node.id or edge.end_node == node.id:
                        s = self.model.nodes[edge.start_node]
                        e = self.model.nodes[edge.end_node]
                        edge.weight = math.sqrt((s.x - e.x)**2 + (s.y - e.y)**2)
                self._redraw()

    def _on_release(self, event):
        self.dragging_node = None

    def _on_motion(self, event):
        mx, my = self._canvas_to_map(event.x, event.y)
        self.coord_label.config(text=f"坐标: ({mx:.2f}, {my:.2f})")
        node = self._hit_test_node(event.x, event.y)
        if node != self.hover_node_id:
            self.hover_node_id = node.id if node else None
            self._redraw()

    def _on_scroll(self, event):
        factor = 1.1 if event.delta > 0 else 1/1.1
        self._zoom_step(factor)

    def _cancel_action(self):
        self.edge_start = None
        self.selected_node = None
        self.selected_edge = None
        self._redraw()
        self._update_status("操作已取消")

    # ---- 命令 ----

    def _set_mode(self, mode):
        self.mode = mode
        self.edge_start = None
        self._update_toolbar()
        mode_names = {'select': '选择/拖拽', 'add_node': '添加节点',
                     'add_edge': '添加边', 'add_oneway_edge': '添加单行线'}
        self._update_status(f"模式: {mode_names.get(mode, mode)}")

    def _update_toolbar(self):
        for btn, m in [(self.btn_select, 'select'), (self.btn_add_node, 'add_node'),
                        (self.btn_add_edge, 'add_edge'), (self.btn_add_oneway, 'add_oneway_edge')]:
            if self.mode == m:
                btn.config(bg='#6e40c9', font=('Arial', 10, 'bold'))
            else:
                btn.config(bg='#444', font=('Arial', 10))

    def _delete_selected(self):
        if self.selected_node:
            name = self.selected_node
            self.model.remove_node(self.selected_node)
            self.selected_node = None
            self._redraw()
            self._update_status(f"删除节点 {name}")
        elif self.selected_edge:
            name = self.selected_edge
            self.model.remove_edge(self.selected_edge)
            self.selected_edge = None
            self._redraw()
            self._update_status(f"删除边 {name}")

    def _rename_selected(self):
        if self.selected_node:
            node = self.model.nodes.get(self.selected_node)
            if node:
                new_name = simpledialog.askstring("重命名节点",
                                                    f"当前名称: {node.name}",
                                                    initialvalue=node.name)
                if new_name is not None:
                    node.name = new_name
                    self._redraw()
        elif self.selected_edge:
            edge = self.model.edges.get(self.selected_edge)
            if edge:
                new_name = simpledialog.askstring("重命名边",
                                                    f"当前名称: {edge.name}",
                                                    initialvalue=edge.name)
                if new_name is not None:
                    edge.name = new_name
                    self._redraw()

    def _toggle_edge_dir(self):
        """切换选中边的 单行(单向)/双向"""
        if not self.selected_edge:
            self._update_status("请先选中一条边")
            return
        edge = self.model.edges.get(self.selected_edge)
        if edge:
            edge.bidirectional = not edge.bidirectional
            state = "单行(单向)" if not edge.bidirectional else "双向"
            self._redraw()
            self._update_status(f"边 {edge.id} 已切换为 {state}")

    def _flip_edge_dir(self):
        """翻转选中边的方向（交换 start/end）"""
        if not self.selected_edge:
            self._update_status("请先选中一条边")
            return
        edge = self.model.edges.get(self.selected_edge)
        if edge:
            edge.start_node, edge.end_node = edge.end_node, edge.start_node
            s = self.model.nodes.get(edge.start_node)
            e = self.model.nodes.get(edge.end_node)
            if s and e:
                edge.weight = math.sqrt((s.x - e.x)**2 + (s.y - e.y)**2)
            self._redraw()
            self._update_status(
                f"边 {edge.id} 方向翻转: {edge.start_node}->{edge.end_node}")

    def _clear_all(self):
        if messagebox.askyesno("确认清空", "确定清空所有节点和边吗？"):
            self.model.clear()
            RouteNode._counter = 0
            RouteEdge._counter = 0
            self.selected_node = None
            self.selected_edge = None
            self.edge_start = None
            self._redraw()

    def _zoom_step(self, factor):
        old_zoom = self.zoom
        self.zoom = max(0.05, min(200.0, self.zoom * factor))
        self.zoom_label.config(text=f"{int(self.zoom * 100)}%")
        self._redraw()

    def _fit_view(self):
        if self.map_loaded and self.map_pil:
            # 以地图为中心
            mx = self.map_origin_x + self.map_width_px * self.map_res / 2
            my = self.map_origin_y + self.map_height_px * self.map_res / 2
            self.offset_x = mx
            self.offset_y = my
            cw = max(self.canvas.winfo_width(), 100)
            ch = max(self.canvas.winfo_height(), 100)
            mw = self.map_width_px * self.map_res
            mh = self.map_height_px * self.map_res
            self.zoom = min(cw / mw, ch / mh) * 0.85
            self.zoom = max(0.05, min(200.0, self.zoom))
        elif self.model.nodes:
            xs = [n.x for n in self.model.nodes.values()]
            ys = [n.y for n in self.model.nodes.values()]
            self.offset_x = (min(xs) + max(xs)) / 2
            self.offset_y = (min(ys) + max(ys)) / 2
            range_x = max(xs) - min(xs) + 2
            range_y = max(ys) - min(ys) + 2
            cw = max(self.canvas.winfo_width(), 100)
            ch = max(self.canvas.winfo_height(), 100)
            self.zoom = min(cw / range_x, ch / range_y) * 0.8
            self.zoom = max(0.05, min(200.0, self.zoom))
        else:
            self.offset_x = 0
            self.offset_y = 0
            self.zoom = 50.0
        self.zoom_label.config(text=f"{int(self.zoom * 100)}%")
        self._redraw()

    # ---- 碰撞检测 ----

    def _hit_test_node(self, cx, cy):
        for node in self.model.nodes.values():
            nx, ny = self._map_to_canvas(node.x, node.y)
            dist = math.sqrt((cx - nx)**2 + (cy - ny)**2)
            if dist <= self.HIT_RADIUS:
                return node
        return None

    def _hit_test_edge(self, cx, cy):
        best_dist = 8
        best_edge = None
        for edge in self.model.edges.values():
            s = self.model.nodes.get(edge.start_node)
            e = self.model.nodes.get(edge.end_node)
            if not s or not e:
                continue
            sx, sy = self._map_to_canvas(s.x, s.y)
            ex, ey = self._map_to_canvas(e.x, e.y)
            dist = self._point_to_segment_dist(cx, cy, sx, sy, ex, ey)
            if dist < best_dist:
                best_dist = dist
                best_edge = edge
        return best_edge

    @staticmethod
    def _point_to_segment_dist(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq == 0:
            return math.sqrt((px - ax)**2 + (py - ay)**2)
        t = max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    # ---- 绘制 ----

    def _redraw(self):
        self.canvas.delete('all')
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()

        # 地图底图
        if self.map_loaded and self.map_pil:
            self._draw_map(cw, ch)
        else:
            self._draw_grid(cw, ch)

        # 边
        for edge in self.model.edges.values():
            self._draw_edge(edge)

        # 边起点高亮
        if self.edge_start and self.edge_start in self.model.nodes:
            n = self.model.nodes[self.edge_start]
            nx, ny = self._map_to_canvas(n.x, n.y)
            r = self.NODE_RADIUS + 4
            self.canvas.create_oval(nx - r, ny - r, nx + r, ny + r,
                                     outline='#ffcc00', width=2, dash=(4, 4))

        # 节点
        for node in self.model.nodes.values():
            self._draw_node(node)

        self.info_label.config(
            text=f"节点: {len(self.model.nodes)}  边: {len(self.model.edges)}")

    def _draw_map(self, cw, ch):
        """绘制地图底图 — 根据当前zoom动态缩放并放置到正确位置"""
        if not self.map_pil:
            return

        # 计算显示尺寸（像素 = 米 * zoom）
        display_w = max(1, int(self.map_width_px * self.map_res * self.zoom))
        display_h = max(1, int(self.map_height_px * self.map_res * self.zoom))

        # 缩放图像
        resized = self.map_pil.resize((display_w, display_h), Image.NEAREST)
        self.map_photo = ImageTk.PhotoImage(resized)

        # 计算地图在canvas上的左上角位置
        # ROS地图：origin是左下角坐标
        # PGM图像：(0,0)是左上角
        # 所以PGM左上角对应的地图坐标是 (origin_x, origin_y + height * res)
        top_left_x = self.map_origin_x
        top_left_y = self.map_origin_y + self.map_height_px * self.map_res
        cx, cy = self._map_to_canvas(top_left_x, top_left_y)

        self.canvas.create_image(cx, cy, image=self.map_photo, anchor='nw')

    def _draw_grid(self, cw, ch):
        step = 1.0
        if self.zoom < 0.2:
            step = 10.0
        elif self.zoom < 0.5:
            step = 5.0
        elif self.zoom < 1.0:
            step = 2.0

        x_start = int(self.offset_x - cw / 2 / self.zoom / step) * step
        x_end = int(self.offset_x + cw / 2 / self.zoom / step + 1) * step
        y_start = int(self.offset_y - ch / 2 / self.zoom / step) * step
        y_end = int(self.offset_y + ch / 2 / self.zoom / step + 1) * step

        for x in range(int(x_start), int(x_end) + 1, int(step)):
            cx_pos, _ = self._map_to_canvas(x, 0)
            self.canvas.create_line(cx_pos, 0, cx_pos, ch, fill=self.COLORS['grid'],
                                     width=1, dash=(2, 4))
            self.canvas.create_text(cx_pos + 2, ch - 4, text=str(x),
                                       fill='#555', font=('Arial', 8), anchor='sw')

        for y in range(int(y_start), int(y_end) + 1, int(step)):
            _, cy_pos = self._map_to_canvas(0, y)
            self.canvas.create_line(0, cy_pos, cw, cy_pos, fill=self.COLORS['grid'],
                                     width=1, dash=(2, 4))
            self.canvas.create_text(4, cy_pos - 2, text=str(y),
                                       fill='#555', font=('Arial', 8), anchor='sw')

        ox, oy = self._map_to_canvas(0, 0)
        self.canvas.create_line(ox - 10, oy, ox + 10, oy, fill='#888', width=2)
        self.canvas.create_line(ox, oy - 10, ox, oy + 10, fill='#888', width=2)
        self.canvas.create_text(ox + 12, oy + 2, text="(0,0)", fill='#888',
                                   font=('Arial', 9), anchor='w')

    def _draw_node(self, node):
        nx, ny = self._map_to_canvas(node.x, node.y)
        r = self.NODE_RADIUS

        is_selected = (node.id == self.selected_node)
        is_edge_start = (node.id == self.edge_start)
        is_hover = (self.hover_node_id == node.id)

        if is_selected:
            color = self.COLORS['node_selected']
            outline_color = '#ffffff'
            outline_w = 2
        elif is_hover:
            color = self.COLORS['node_hover']
            outline_color = '#ffffff'
            outline_w = 2
        elif is_edge_start:
            color = '#ffcc00'
            outline_color = '#ffffff'
            outline_w = 2
        else:
            color = self.COLORS['node']
            outline_color = '#333'
            outline_w = 1

        self.canvas.create_oval(nx - r, ny - r, nx + r, ny + r,
                                 fill=color, outline=outline_color, width=outline_w)
        self.canvas.create_text(nx, ny, text=node.id, fill=self.COLORS['node_text'],
                                 font=('Arial', 8, 'bold'))
        self.canvas.create_text(nx, ny - r - 8, text=node.name,
                                 fill='#cccccc', font=('Arial', 9))

    def _draw_edge(self, edge):
        s = self.model.nodes.get(edge.start_node)
        e = self.model.nodes.get(edge.end_node)
        if not s or not e:
            return

        sx, sy = self._map_to_canvas(s.x, s.y)
        ex, ey = self._map_to_canvas(e.x, e.y)

        is_selected = (edge.id == self.selected_edge)
        is_oneway = not edge.bidirectional
        if is_selected:
            color = self.COLORS['edge_selected']
        elif is_oneway:
            color = '#ff8800'
        else:
            color = self.COLORS['edge']
        width = 3 if is_selected else 2

        self.canvas.create_line(sx, sy, ex, ey, fill=color, width=width)

        if is_oneway:
            dx, dy = ex - sx, ey - sy
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                ux, uy = dx / length, dy / length
                ax1 = ex - ux * 12 - uy * 6
                ay1 = ey - uy * 12 + ux * 6
                ax2 = ex - ux * 12 + uy * 6
                ay2 = ey - uy * 12 - ux * 6
                self.canvas.create_polygon(ex, ey, ax1, ay1, ax2, ay2,
                                            fill=color, outline='')

        mx, my = (sx + ex) / 2, (sy + ey) / 2
        label = f"{edge.weight:.1f}m"
        if is_oneway:
            label += " →"
        self.canvas.create_text(mx, my - 8,
                                 text=label,
                                 fill=('#ff8800' if is_oneway else '#999'),
                                 font=('Arial', 8))

    # ---- 文件操作 ----

    def _load_map(self, map_path=None, yaml_path=None):
        """加载地图底图"""
        if not map_path:
            return
        if not os.path.exists(map_path):
            self._update_status(f"地图文件不存在: {map_path}")
            return

        try:
            img = Image.open(map_path)
            if img.mode != 'RGBA':
                img = img.convert('RGBA')

            # 解析YAML获取地图参数
            if yaml_path and os.path.exists(yaml_path):
                self._parse_yaml(yaml_path)
            else:
                # 尝试自动找YAML
                auto_yaml = map_path.rsplit('.', 1)[0] + '.yaml'
                if os.path.exists(auto_yaml):
                    self._parse_yaml(auto_yaml)

            self.map_pil = img
            self.map_width_px = img.size[0]
            self.map_height_px = img.size[1]
            self.map_loaded = True

            self._update_status(
                f"地图: {os.path.basename(map_path)} "
                f"({self.map_width_px}x{self.map_height_px}px, "
                f"res={self.map_res}, origin=({self.map_origin_x},{self.map_origin_y}))"
            )

            # 自动适应视图
            self._fit_view()
        except Exception as e:
            self._update_status(f"加载地图失败: {e}")
            import traceback
            traceback.print_exc()

    def _parse_yaml(self, yaml_path):
        """解析ROS地图YAML文件"""
        try:
            with open(yaml_path, 'r') as f:
                content = f.read()

            # resolution: 0.05
            m = re.search(r'resolution:\s*([\d.]+)', content)
            if m:
                self.map_res = float(m.group(1))

            # origin: [-0.802, -0.432, 0]
            m = re.search(r'origin:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]', content)
            if m:
                self.map_origin_x = float(m.group(1))
                self.map_origin_y = float(m.group(2))
        except Exception as e:
            self._update_status(f"解析YAML警告: {e}")

    def _browse_map(self):
        path = filedialog.askopenfilename(
            title="选择地图文件",
            filetypes=[("地图文件", "*.pgm *.png *.jpg"),
                       ("PGM地图", "*.pgm"),
                       ("PNG图片", "*.png"),
                       ("所有文件", "*.*")]
        )
        if path:
            yaml_path = path.rsplit('.', 1)[0] + '.yaml'
            self._load_map(path, yaml_path)
            self._redraw()

    def _load_geojson_file(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.model.from_geojson(data)
            self._update_status(f"加载路网: {len(self.model.nodes)}节点, {len(self.model.edges)}边")
            self._fit_view()
        except Exception as e:
            self._update_status(f"加载路网失败: {e}")

    def _browse_geojson(self):
        path = filedialog.askopenfilename(
            title="选择路网GeoJSON文件",
            filetypes=[("GeoJSON", "*.geojson *.json"), ("所有文件", "*.*")]
        )
        if path:
            self._load_geojson_file(path)

    def _save_geojson(self):
        path = filedialog.asksaveasfilename(
            title="保存路网GeoJSON",
            defaultextension=".geojson",
            filetypes=[("GeoJSON", "*.geojson"), ("JSON", "*.json")],
            initialfile=self.model.properties.get('name', 'route') + '.geojson'
        )
        if path:
            try:
                data = self.model.to_geojson()
                with open(path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                self._update_status(f"保存成功: {path}")
            except Exception as e:
                messagebox.showerror("保存失败", str(e))

    def _update_status(self, text):
        self.status_label.config(text=text)


def main():
    parser = argparse.ArgumentParser(description='路网可视化绘制工具')
    parser.add_argument('--map', help='地图图片路径(.pgm/.png)')
    parser.add_argument('--yaml', help='地图YAML配置路径')
    parser.add_argument('--geojson', help='路网GeoJSON文件路径')
    args = parser.parse_args()

    root = tk.Tk()
    app = RouteEditor(root, map_path=args.map, yaml_path=args.yaml,
                      geojson_path=args.geojson)
    root.mainloop()


if __name__ == '__main__':
    main()
