"""Reconstructed geometric IK for an idealized yaw–pitch–pitch–pitch chain.
The six joint values follow the sign convention in the recovered solver.
Not a robot controller, and not a general six-degree-of-freedom pose solver.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Geometry:
    upper_arm: float
    forearm: float
    tool: float
    base_height: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.upper_arm,self.forearm,self.tool,self.base_height)):
            raise ValueError('Geometry must be finite')
        if min(self.upper_arm,self.forearm) <= 0 or self.tool < 0:
            raise ValueError('Invalid link lengths')


def solve_ik(x, y, z, g: Geometry):
    """Return six joint angles in radians, or None for unreachable/singular targets.
    Uses a radial tool pitch and the elbow branch from the recovered code.
    """
    if not all(math.isfinite(v) for v in (x,y,z)):
        return None
    r = math.hypot(x,y)
    h = z-g.base_height
    if math.hypot(r,h) < 1e-12:
        return None
    yaw = math.atan2(y,x)
    pitch = math.atan2(h,r)
    rw = r-g.tool*math.cos(pitch)
    zw = h-g.tool*math.sin(pitch)
    c = math.hypot(rw,zw)
    if c < 1e-12 or c > g.upper_arm+g.forearm+1e-12 or c < abs(g.upper_arm-g.forearm)-1e-12:
        return None
    clamp = lambda v: max(-1.0,min(1.0,v))
    elbow = math.pi-math.acos(clamp((g.upper_arm**2+g.forearm**2-c*c)/(2*g.upper_arm*g.forearm)))
    shoulder = -(math.atan2(zw,rw)+math.acos(clamp((g.upper_arm**2+c*c-g.forearm**2)/(2*g.upper_arm*c))))
    wrist = -pitch-shoulder-elbow
    return yaw,shoulder,elbow,0.0,wrist,0.0


def forward_position(q, g: Geometry):
    """Idealized planar-chain FK, independent of a robot-specific URDF."""
    yaw, shoulder, elbow, roll1, wrist, roll2 = q
    if abs(roll1)>1e-12 or abs(roll2)>1e-12:
        raise ValueError('This simplified model assumes joints 4 and 6 are zero')
    a,b,c = -shoulder,-(shoulder+elbow),-(shoulder+elbow+wrist)
    r = g.upper_arm*math.cos(a)+g.forearm*math.cos(b)+g.tool*math.cos(c)
    z = g.base_height+g.upper_arm*math.sin(a)+g.forearm*math.sin(b)+g.tool*math.sin(c)
    return r*math.cos(yaw),r*math.sin(yaw),z
