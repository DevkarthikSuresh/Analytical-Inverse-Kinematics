import math
import time
import numpy as np
import matplotlib.pyplot as plt
import PyKDL
import kdl_parser_py.urdf as kdl_parser

# --- 1. YOUR VERIFIED CUSTOM SOLVER ---
class RobotIK:
    def __init__(self):
        # EXACT Dimensions from gemini.urdf
        self.L1 = 0.3   # Upper Arm
        self.L2 = 0.3   # Forearm
        self.L_tool = 0.1 # Distance from Wrist Center to EE
        self.base_h = 0.2 # Base height

    def solve_ik(self, x, y, z):
        try:
            # 1. Base Yaw
            theta1 = math.atan2(y, x)
            
            # 2. Wrist Center
            r_ee = math.sqrt(x**2 + y**2)
            z_adj = z - self.base_h
            pitch_ee = math.atan2(z_adj, r_ee)
            
            r_wc = r_ee - self.L_tool * math.cos(pitch_ee)
            z_wc = z_adj - self.L_tool * math.sin(pitch_ee)
            
            # 3. Elbow
            C = math.sqrt(r_wc**2 + z_wc**2)
            if C > (self.L1 + self.L2): return None

            cos_q3 = (self.L1**2 + self.L2**2 - C**2) / (2 * self.L1 * self.L2)
            theta3_val = math.acos(np.clip(cos_q3, -1, 1))
            
            # Standard Elbow Logic
            theta3 = -(math.pi - theta3_val)

            # 4. Shoulder
            alpha = math.atan2(z_wc, r_wc)
            cos_beta = (self.L1**2 + C**2 - self.L2**2) / (2 * self.L1 * C)
            beta = math.acos(np.clip(cos_beta, -1, 1))
            
            theta2_math = alpha + beta 
            
            # URDF Conversion (Horizontal Zero)
            theta2 = -theta2_math
            theta3 = -theta3

            # 5. Orientation
            theta5 = -pitch_ee - (theta2 + theta3)

            return [math.degrees(t) for t in [theta1, theta2, theta3, 0, theta5, 0]]
        except: return None

# --- 2. SETUP & VERIFICATION ---
def setup_kdl(urdf_path):
    with open(urdf_path, 'r') as f:
        urdf_s = f.read()
    ok, tree = kdl_parser.treeFromString(urdf_s.encode('utf-8'))
    if not ok: return None, None, None
    chain = tree.getChain("world", "ee_link")
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)
    return chain, fk_solver, ik_solver

def verify_position_error(target_pt, joint_angles_deg, fk_solver, chain):
    if joint_angles_deg is None: return 9999.0
    q = PyKDL.JntArray(chain.getNrOfJoints())
    for i, angle in enumerate(joint_angles_deg):
        q[i] = math.radians(angle)
    end_frame = PyKDL.Frame()
    fk_solver.JntToCart(q, end_frame)
    actual_pos = np.array([end_frame.p.x(), end_frame.p.y(), end_frame.p.z()])
    return np.linalg.norm(actual_pos - np.array(target_pt)) * 1000.0 

# --- 3. MAIN ---
if __name__ == "__main__":
    urdf_path = "gemini.urdf"
    chain, fk_solver, ik_solver = setup_kdl(urdf_path)
    custom_ik = RobotIK()
    
    # Let's add more points for a better graph
    test_points = [
        (0.4, 0.0, 0.4), 
        (0.3, 0.1, 0.5), 
        (0.2, -0.1, 0.3),
        (0.35, 0.05, 0.35),
        (0.25, -0.05, 0.45)
    ]
    
    kdl_errors = []
    custom_errors = []
    labels = []

    print(f"\n{'Target Point':<20} | {'KDL Err (mm)':<12} | {'Custom Err (mm)':<15}")
    print("-" * 55)

    for i, pt in enumerate(test_points):
        # Solve & Verify Custom
        custom_deg = custom_ik.solve_ik(*pt)
        err_custom = verify_position_error(pt, custom_deg, fk_solver, chain)
        custom_errors.append(err_custom)
        
        # Solve & Verify KDL
        q_kdl = PyKDL.JntArray(6)
        ik_solver.CartToJnt(PyKDL.JntArray(6), PyKDL.Frame(PyKDL.Vector(*pt)), q_kdl)
        kdl_deg = [math.degrees(q_kdl[j]) for j in range(6)]
        err_kdl = verify_position_error(pt, kdl_deg, fk_solver, chain)
        kdl_errors.append(err_kdl)
        
        labels.append(f"Pt {i+1}")
        print(f"{str(pt):<20} | {err_kdl:.4f}       | {err_custom:.4f}")

    # --- PLOTTING ERROR BAR CHART ---
    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    rects1 = ax.bar(x - width/2, kdl_errors, width, label='Orocos KDL (LMA)', color='blue', alpha=0.7)
    rects2 = ax.bar(x + width/2, custom_errors, width, label='GeoIK (Analytical)', color='red', alpha=0.7)

    ax.set_ylabel('Position Error (mm)')
    ax.set_title('Inverse Kinematics Accuracy Comparison (Lower is Better)')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    
    # Add grid on Y axis
    ax.yaxis.grid(True, linestyle='--', alpha=0.7)

    # Optional: If errors are TINY, use log scale to see them
    # ax.set_yscale('log') 

    plt.tight_layout()
    plt.show()

    # Speed Benchmark
    print("\nRunning Speed Benchmark...")
    start = time.time()
    for _ in range(10000): custom_ik.solve_ik(0.3, 0.1, 0.3)
    custom_fps = 10000 / (time.time() - start)
    
    start = time.time()
    q_dummy = PyKDL.JntArray(6)
    target = PyKDL.Frame(PyKDL.Vector(0.3, 0.1, 0.3))
    for _ in range(10000): ik_solver.CartToJnt(PyKDL.JntArray(6), target, q_dummy)
    kdl_fps = 10000 / (time.time() - start)
    
    print(f"Speedup: {custom_fps/kdl_fps:.1f}x FASTER")
