"""Idealized numerical self-consistency and timing; not a KDL comparison."""
import math
import statistics
import time
from analytic_ik import Geometry, solve_ik, forward_position

def main():
    g=Geometry(.30,.30,.10,.20)
    targets=[(.4,0,.4),(.3,.1,.5),(.2,-.1,.3),(.35,.05,.35),(.25,-.05,.45)]
    errors=[1000*math.dist(p,forward_position(solve_ik(*p,g),g)) for p in targets]
    timings=[]
    for _ in range(5):
        start=time.perf_counter_ns()
        for i in range(10000): solve_ik(*targets[i%len(targets)],g)
        timings.append((time.perf_counter_ns()-start)/10000/1000)
    print('Idealized FK position errors (mm):',errors)
    print('Median analytical solve time (microseconds):',statistics.median(timings))
    print('Not robot/URDF validation; no KDL speedup inferred.')

if __name__=='__main__': main()
