#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GeoJSON路网文件解析模块

支持标准GeoJSON格式解析：
- 节点（Node）: Feature类型，geometry为Point，properties包含id、name等
- 边（Edge）: Feature类型，geometry为LineString，properties包含id、start_node、end_node、weight等

GeoJSON结构示例:
{
  "type": "FeatureCollection",
  "properties": {
    "name": "仓库路网",
    "description": "工厂AGV运输路网"
  },
  "features": [
    {
      "type": "Feature",
      "geometry": {"type": "Point", "coordinates": [x, y]},
      "properties": {"id": "node_1", "name": "起点", "type": "node"}
    },
    {
      "type": "Feature",
      "geometry": {"type": "LineString", "coordinates": [[x1,y1], [x2,y2]]},
      "properties": {"id": "edge_1", "start_node": "node_1", "end_node": "node_2", "type": "edge", "weight": 1.0}
    }
  ]
}
"""

import json
import math
from typing import Dict, List, Optional, Tuple, Any


class GeoJSONRouteParser:
    """GeoJSON路网文件解析器"""

    def __init__(self):
        self.raw_data: Dict[str, Any] = {}
        self.nodes: Dict[str, Dict] = {}
        self.edges: Dict[str, Dict] = {}
        self.properties: Dict[str, Any] = {}

    def parse_file(self, filepath: str) -> bool:
        """从文件解析GeoJSON路网"""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                self.raw_data = json.load(f)
            return self._process_data()
        except Exception as e:
            print(f"解析GeoJSON文件失败: {e}")
            return False

    def parse_string(self, geojson_str: str) -> bool:
        """从字符串解析GeoJSON路网"""
        try:
            self.raw_data = json.loads(geojson_str)
            return self._process_data()
        except Exception as e:
            print(f"解析GeoJSON字符串失败: {e}")
            return False

    def _process_data(self) -> bool:
        """处理解析后的原始数据"""
        if self.raw_data.get('type') != 'FeatureCollection':
            print("错误: GeoJSON根类型必须是FeatureCollection")
            return False

        # 提取全局属性
        self.properties = self.raw_data.get('properties', {})

        features = self.raw_data.get('features', [])
        if not features:
            print("警告: GeoJSON中未找到任何features")
            return False

        self.nodes.clear()
        self.edges.clear()

        for feature in features:
            if feature.get('type') != 'Feature':
                continue

            geometry = feature.get('geometry', {})
            properties = feature.get('properties', {})
            feature_type = properties.get('type', '').lower()

            geo_type = geometry.get('type', '')

            if geo_type == 'Point' or feature_type == 'node':
                self._parse_node(feature)
            elif geo_type == 'LineString' or feature_type == 'edge':
                self._parse_edge(feature)

        if not self.nodes:
            print("警告: 未解析到任何节点")
            return False

        print(f"GeoJSON解析完成: {len(self.nodes)}个节点, {len(self.edges)}条边")
        return True

    def _parse_node(self, feature: Dict):
        """解析节点特征"""
        properties = feature.get('properties', {})
        geometry = feature.get('geometry', {})
        coordinates = geometry.get('coordinates', [0.0, 0.0])

        node_id = str(properties.get('id', ''))
        if not node_id:
            # 如果没有id，生成一个
            node_id = f"node_{len(self.nodes)}"

        self.nodes[node_id] = {
            'id': node_id,
            'name': properties.get('name', node_id),
            'x': float(coordinates[0]),
            'y': float(coordinates[1]) if len(coordinates) > 1 else 0.0,
            'z': float(coordinates[2]) if len(coordinates) > 2 else 0.0,
            'properties': properties
        }

    def _parse_edge(self, feature: Dict):
        """解析边特征"""
        properties = feature.get('properties', {})
        geometry = feature.get('geometry', {})
        coordinates = geometry.get('coordinates', [])

        edge_id = str(properties.get('id', ''))
        if not edge_id:
            edge_id = f"edge_{len(self.edges)}"

        start_node = str(properties.get('start_node', ''))
        end_node = str(properties.get('end_node', ''))

        # 如果没有显式指定起止节点，尝试从坐标推断
        if not start_node and coordinates:
            start_node = self._find_nearest_node(coordinates[0])
        if not end_node and coordinates:
            end_node = self._find_nearest_node(coordinates[-1])

        # 计算边的长度作为默认权重
        weight = properties.get('weight', None)
        if weight is None and len(coordinates) >= 2:
            weight = self._calculate_length(coordinates)
        elif weight is None:
            weight = 1.0

        self.edges[edge_id] = {
            'id': edge_id,
            'name': properties.get('name', edge_id),
            'start_node': start_node,
            'end_node': end_node,
            'weight': float(weight),
            'coordinates': coordinates,
            'bidirectional': properties.get('bidirectional', True),
            'properties': properties
        }

    def _find_nearest_node(self, coord: List[float]) -> str:
        """根据坐标找到最近的节点"""
        if not self.nodes:
            return ''

        min_dist = float('inf')
        nearest_id = ''
        x, y = float(coord[0]), float(coord[1]) if len(coord) > 1 else 0.0

        for node_id, node in self.nodes.items():
            dist = math.sqrt((node['x'] - x)**2 + (node['y'] - y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_id = node_id

        return nearest_id

    @staticmethod
    def _calculate_length(coordinates: List[List[float]]) -> float:
        """计算折线长度"""
        length = 0.0
        for i in range(1, len(coordinates)):
            x1, y1 = float(coordinates[i-1][0]), float(coordinates[i-1][1])
            x2, y2 = float(coordinates[i][0]), float(coordinates[i][1])
            length += math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return length

    def get_nodes(self) -> Dict[str, Dict]:
        """获取所有节点"""
        return self.nodes

    def get_edges(self) -> Dict[str, Dict]:
        """获取所有边"""
        return self.edges

    def get_node_by_id(self, node_id: str) -> Optional[Dict]:
        """根据ID获取节点"""
        return self.nodes.get(node_id)

    def get_edge_by_id(self, edge_id: str) -> Optional[Dict]:
        """根据ID获取边"""
        return self.edges.get(edge_id)

    def validate(self) -> Tuple[bool, List[str]]:
        """验证路网数据的完整性"""
        errors = []

        # 检查边引用的节点是否存在
        for edge_id, edge in self.edges.items():
            if edge['start_node'] not in self.nodes:
                errors.append(f"边 {edge_id} 引用的起点节点 {edge['start_node']} 不存在")
            if edge['end_node'] not in self.nodes:
                errors.append(f"边 {edge_id} 引用的终点节点 {edge['end_node']} 不存在")
            if edge['start_node'] == edge['end_node']:
                errors.append(f"边 {edge_id} 的起点和终点相同")

            # 单行线（单向通行）方向提示：规划只依赖 start/end 节点与
            # bidirectional，几何坐标不影响通行方向；此处仅当几何方向与
            # start->end 相反时给出提示，方便在编辑器里修正。
            if edge.get('bidirectional', True) is False:
                coords = edge.get('coordinates', [])
                s = self.nodes.get(edge['start_node'])
                e = self.nodes.get(edge['end_node'])
                if len(coords) >= 2 and s and e:
                    gx = float(coords[-1][0]) - float(coords[0][0])
                    gy = float(coords[-1][1]) - float(coords[0][1])
                    nx = e['x'] - s['x']
                    ny = e['y'] - s['y']
                    if (gx * nx + gy * ny) < 0:
                        errors.append(
                            f"提示: 单行线 {edge_id} 的几何方向与 "
                            f"{edge['start_node']}->{edge['end_node']} 相反")

        # 检查孤立节点
        connected_nodes = set()
        for edge in self.edges.values():
            connected_nodes.add(edge['start_node'])
            connected_nodes.add(edge['end_node'])

        isolated = set(self.nodes.keys()) - connected_nodes
        if isolated:
            errors.append(f"孤立节点(无连接): {', '.join(isolated)}")

        return len(errors) == 0, errors

    def to_simple_graph(self) -> Tuple[Dict[str, Tuple[float, float]], Dict[str, List[Tuple[str, float]]]]:
        """
        转换为简单图结构，便于路径规划算法使用
        返回: (nodes_dict, adjacency_list)
        nodes_dict: {node_id: (x, y)}
        adjacency_list: {node_id: [(neighbor_id, weight), ...]}
        """
        nodes_dict = {}
        adjacency = {node_id: [] for node_id in self.nodes}

        for node_id, node in self.nodes.items():
            nodes_dict[node_id] = (node['x'], node['y'])

        for edge in self.edges.values():
            start = edge['start_node']
            end = edge['end_node']
            weight = edge['weight']

            if start in adjacency and end in adjacency:
                adjacency[start].append((end, weight))
                if edge.get('bidirectional', True):
                    adjacency[end].append((start, weight))

        return nodes_dict, adjacency
