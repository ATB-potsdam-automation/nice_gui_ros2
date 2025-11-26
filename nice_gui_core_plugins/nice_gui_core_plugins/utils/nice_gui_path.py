from typing import List

from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from nicegui.elements.scene import Scene
from nicegui.elements.scene.scene_objects import Line


class NiceguiPath:
    def __init__(self, scene: Scene, color: str = "#ff0000"):
        self._path: List[Line] = []
        self._scene = scene
        self._color = color

    def draw(self, path: Path):
        for i in range(len(path.poses) - 1):
            pose: Point = path.poses[i].pose.position
            next_pose: Point = path.poses[i + 1].pose .position
            line = self._scene.line([pose.x, pose.y, 0], [next_pose.x, next_pose.y, 0])
            line.material(self._color)
            self._path.append(line)

    def delete(self):
        for line in self._path:
            line.delete()
            self._path = []
