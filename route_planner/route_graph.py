#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路网图数据结构与路径规划算法

提供：
- 图数据结构管理
- A*路径规划
- Dijkstra最短路径
- 最近节点/边查找
- 路径点密集化（生成平滑曲线）
"""

import math
import heapq
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field


@dataclass
class RouteNode:
    """路网节点"""
    id: str
    x: float
    y: float
    z: float = 0.0
    name: str = ''
    metadata: Dict = field(default_factory=dict)


@dataclass
class RouteEdge:
    """路网边"""
    id: str
    start_node: str
    end_node: str
    weight: float
    bidirectional: bool = True
    name: str = ''
    metadata: Dict = field(default_factory=dict)


class RouteGraph:
    """路网图数据结构"""

    def __init__(self):
        self.nodes: Dict[str, RouteNode] = {}
        self.edges: Dict[str, RouteEdge] = {}
        self.adjacency: Dict[str, List[Tuple[str, str, float]]] = {}

    def add_node(self, node_id: str, x: float, y: float, z: float = 0.0,
                 name: str = '', metadata: Dict = None) -> bool:
        if node_id in self.nodes:
            return False
        self.nodes[node_id] = RouteNode(
            id=node_id, x=x, y=y, z=z,
            name=name or node_id, metadata=metadata or {}
        )
        if node_id not in self.adjacency:
            self.adjacency[node_id] = []
        return True

    def add_edge(self, edge_id: str, start_node: str, end_node: str,
                 weight: float = None, bidirectional: bool = True,
                 name: str = '', metadata: Dict = None) -> bool:
        if start_node not in self.nodes or end_node not in self.nodes:
            return False
        if weight is None:
            weight = self._euclidean_distance(start_node, end_node)

        self.edges[edge_id] = RouteEdge(
            id=edge_id, start_node=start_node, end_node=end_node,
            weight=weight, bidirectional=bidirectional,
            name=name or edge_id, metadata=metadata or {}
        )

        if start_node not in self.adjacency:
            self.adjacency[start_node] = []
        self.adjacency[start_node].append((end_node, edge_id, weight))

        if bidirectional:
            if end_node not in self.adjacency:
                self.adjacency[end_node] = []
            self.adjacency[end_node].append((start_node, edge_id, weight))
        return True

    def remove_edge(self, edge_id: str) -> bool:
        if edge_id not in self.edges:
            return False
        edge = self.edges[edge_id]
        self.adjacency[edge.start_node] = [
            item for item in self.adjacency[edge.start_node]
            if item[1] != edge_id
        ]
        if edge.bidirectional:
            self.adjacency[edge.end_node] = [
                item for item in self.adjacency[edge.end_node]
                if item[1] != edge_id
            ]
        del self.edges[edge_id]
        return True

    def get_neighbors(self, node_id: str) -> List[Tuple[str, str, float]]:
        return self.adjacency.get(node_id, [])

    def _euclidean_distance(self, node_a: str, node_b: str) -> float:
        a = self.nodes[node_a]
        b = self.nodes[node_b]
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

    def heuristic(self, node_a: str, node_b: str) -> float:
        return self._euclidean_distance(node_a, node_b)

    def find_nearest_node(self, x: float, y: float,
                          max_distance: float = float('inf')) -> Optional[str]:
        if not self.nodes:
            return None
        min_dist = float('inf')
        nearest_id = None
        for node_id, node in self.nodes.items():
            dist = math.sqrt((node.x - x)**2 + (node.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_id = node_id
        if min_dist <= max_distance:
            return nearest_id
        return None

    def distance_to_nearest_node(self, x: float, y: float) -> float:
        """返回到最近节点的距离"""
        min_dist = float('inf')
        for node in self.nodes.values():
            dist = math.sqrt((node.x - x)**2 + (node.y - y)**2)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def is_on_graph(self, x: float, y: float, threshold: float = 0.3) -> bool:
        """判断坐标是否在路网边上（到最近边的距离小于阈值）"""
        min_dist = float('inf')
        for edge in self.edges.values():
            d = self._point_to_segment_dist(
                x, y,
                self.nodes[edge.start_node].x, self.nodes[edge.start_node].y,
                self.nodes[edge.end_node].x, self.nodes[edge.end_node].y
            )
            if d < min_dist:
                min_dist = d
        return min_dist <= threshold

    @staticmethod
    def _point_to_segment_dist(px, py, ax, ay, bx, by) -> float:
        """计算点到线段的距离"""
        dx, dy = bx - ax, by - ay
        length_sq = dx*dx + dy*dy
        if length_sq == 0:
            return math.sqrt((px - ax)**2 + (py - ay)**2)
        t = max(0, min(1, ((px - ax)*dx + (py - ay)*dy) / length_sq))
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    def dijkstra(self, start: str, goal: str) -> Tuple[Optional[List[str]], float]:
        if start not in self.nodes or goal not in self.nodes:
            return None, float('inf')
        distances = {node_id: float('inf') for node_id in self.nodes}
        distances[start] = 0.0
        previous = {node_id: None for node_id in self.nodes}
        visited = set()
        pq = [(0.0, start)]
        while pq:
            current_dist, current = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                break
            for neighbor, edge_id, weight in self.adjacency.get(current, []):
                if neighbor in visited:
                    continue
                new_dist = current_dist + weight
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))
        if distances[goal] == float('inf'):
            return None, float('inf')
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = previous[current]
        path.reverse()
        return path, distances[goal]

    def astar(self, start: str, goal: str) -> Tuple[Optional[List[str]], float]:
        if start not in self.nodes or goal not in self.nodes:
            return None, float('inf')
        g_score = {node_id: float('inf') for node_id in self.nodes}
        g_score[start] = 0.0
        f_score = {node_id: float('inf') for node_id in self.nodes}
        f_score[start] = self.heuristic(start, goal)
        previous = {node_id: None for node_id in self.nodes}
        open_set = [(f_score[start], start)]
        open_set_ids = {start}
        closed_set = set()
        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_ids.discard(current)
            if current == goal:
                path = []
                node = goal
                while node is not None:
                    path.append(node)
                    node = previous[node]
                path.reverse()
                return path, g_score[goal]
            closed_set.add(current)
            for neighbor, edge_id, weight in self.adjacency.get(current, []):
                if neighbor in closed_set:
                    continue
                tentative_g = g_score[current] + weight
                if tentative_g < g_score[neighbor]:
                    previous[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    if neighbor not in open_set_ids:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_ids.add(neighbor)
        return None, float('inf')

    def plan_path(self, start: str, goal: str,
                  algorithm: str = 'astar') -> Tuple[Optional[List[str]], float]:
        if algorithm.lower() == 'dijkstra':
            return self.dijkstra(start, goal)
        return self.astar(start, goal)

    def get_path_edges(self, node_path: List[str]) -> List[str]:
        if len(node_path) < 2:
            return []
        edges = []
        for i in range(len(node_path) - 1):
            current = node_path[i]
            next_node = node_path[i + 1]
            for neighbor, edge_id, weight in self.adjacency.get(current, []):
                if neighbor == next_node:
                    edges.append(edge_id)
                    break
        return edges

    def get_path_waypoints(self, node_path: List[str],
                           include_midpoints: bool = False) -> List[Tuple[float, float, float]]:
        """获取路径的航点列表 (x, y, yaw)"""
        if not node_path:
            return []
        waypoints = []
        for i, node_id in enumerate(node_path):
            node = self.nodes[node_id]
            if i < len(node_path) - 1:
                next_node = self.nodes[node_path[i + 1]]
                yaw = math.atan2(next_node.y - node.y, next_node.x - node.x)
            else:
                if i > 0:
                    prev_node = self.nodes[node_path[i - 1]]
                    yaw = math.atan2(node.y - prev_node.y, node.x - prev_node.x)
                else:
                    yaw = 0.0
            waypoints.append((node.x, node.y, yaw))
            if include_midpoints and i < len(node_path) - 1:
                next_node = self.nodes[node_path[i + 1]]
                mid_x = (node.x + next_node.x) / 2.0
                mid_y = (node.y + next_node.y) / 2.0
                waypoints.append((mid_x, mid_y, yaw))
        return waypoints

    def get_statistics(self) -> Dict:
        total_edge_length = sum(edge.weight for edge in self.edges.values())
        avg_edge_length = total_edge_length / len(self.edges) if self.edges else 0
        return {
            'node_count': len(self.nodes),
            'edge_count': len(self.edges),
            'total_edge_length': total_edge_length,
            'average_edge_length': avg_edge_length,
            'bidirectional_edges': sum(1 for e in self.edges.values() if e.bidirectional),
            'one_way_edges': sum(1 for e in self.edges.values() if not e.bidirectional)
        }

    def clear(self):
        self.nodes.clear()
        self.edges.clear()
        self.adjacency.clear()
