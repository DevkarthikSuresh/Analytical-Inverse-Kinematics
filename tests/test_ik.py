import math
import random
import unittest
from analytic_ik import Geometry, solve_ik, forward_position

# Illustrative dimensions, not a robot-specific URDF.
G = Geometry(.30,.30,.10,.20)

class IKTests(unittest.TestCase):
    def test_recovered_targets(self):
        for p in [(0.4,0,.4),(.3,.1,.5),(.2,-.1,.3),(.35,.05,.35),(.25,-.05,.45)]:
            q=solve_ik(*p,G)
            self.assertIsNotNone(q)
            self.assertLess(math.dist(p,forward_position(q,G)),1e-10)
    def test_random_reachable_targets(self):
        rng=random.Random(42)
        for _ in range(250):
            yaw=rng.uniform(-math.pi,math.pi)
            pitch=rng.uniform(-1.4,1.4)
            d=rng.uniform(.12,.69)
            r=d*math.cos(pitch)
            p=(r*math.cos(yaw),r*math.sin(yaw),G.base_height+d*math.sin(pitch))
            q=solve_ik(*p,G)
            self.assertIsNotNone(q)
            self.assertLess(math.dist(p,forward_position(q,G)),1e-10)
    def test_unreachable(self):
        self.assertIsNone(solve_ik(2,0,.2,G))
        self.assertIsNone(solve_ik(.1,0,.2,G))
        self.assertIsNone(solve_ik(float('nan'),0,.2,G))
    def test_inner_reach(self):
        self.assertIsNone(solve_ik(.12,0,.2,Geometry(.4,.1,.1,.2)))
    def test_tool_pitch(self):
        p=(.3,.1,.5)
        q=solve_ik(*p,G)
        self.assertAlmostEqual(-(q[1]+q[2]+q[4]),math.atan2(p[2]-G.base_height,math.hypot(*p[:2])))

if __name__=='__main__': unittest.main()
