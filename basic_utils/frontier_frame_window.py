"""Bounded original-frame evidence for current frontier cells; no ROS dependency."""
from collections import OrderedDict
from dataclasses import dataclass, field
import math
from typing import Any, Optional

@dataclass
class Frame:
    stamp: int
    image: Any
    camera_x: float
    camera_y: float
    camera_yaw: float
    resolution: float
    origin_x: float
    origin_y: float
    max_distance: float
    free_cells: set
    camera_pose: tuple
    robot_pose: tuple
    header: Any = None
    score: Optional[float] = None
    sent_ids: set = field(default_factory=set)
    failures: int = 0

class FrameWindow:
    def __init__(self, capacity=20):
        self.capacity=capacity
        self.frames=OrderedDict()
    def add(self, frame):
        if frame.stamp in self.frames:
            return False  # Keep the original, never replace with annotated RGB.
        self.frames[frame.stamp]=frame
        self.frames=OrderedDict(sorted(self.frames.items()))
        while len(self.frames)>self.capacity:self.frames.popitem(last=False)
        return True
    def snapshot(self):
        return list(self.frames.values())

def project_cells(frame, cells):
    """Project onto each boundary cell using the original observed free neighbors.

    A frontier is unknown itself. An actual free ray cell sharing an edge is its
    observed support; the cluster centroid need not be visible. Never consult
    the current occupancy map to reinterpret a historical view.
    """
    result=[]
    half_fov=math.radians(79./2.)
    for token,ix,iy in cells:
        if token in frame.sent_ids:continue
        if not any((ix+dx,iy+dy) in frame.free_cells for dx,dy in ((1,0),(-1,0),(0,1),(0,-1))):continue
        x=frame.origin_x+(ix+.5)*frame.resolution
        y=frame.origin_y+(iy+.5)*frame.resolution
        dx,dy=x-frame.camera_x,y-frame.camera_y
        distance=math.hypot(dx,dy)
        if distance<1e-6:continue
        delta=math.atan2(dy,dx)-frame.camera_yaw
        angle=abs(math.atan2(math.sin(delta),math.cos(delta)))
        if angle>=half_fov:continue
        angular=math.cos(angle/half_fov*math.pi/2.)**2
        # Positive at sensor range; a hard zero there would hide range frontiers.
        weight=angular/(1.+(distance/max(frame.max_distance,frame.resolution))**2)
        if weight>1e-9:result.append((token,weight))
    return result

def evaluate_frame(frame,cells,infer):
    """Infer only when this original image supports unsent current frontier cells."""
    projected=project_cells(frame,cells)
    if not projected:return None
    if frame.score is None:
        value=float(infer(frame.image))
        if not math.isfinite(value) or not -1.<=value<=1.:raise ValueError('Invalid semantic cosine')
        frame.score=value
    return frame.score,projected
