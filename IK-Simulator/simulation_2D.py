import math
import matplotlib.pyplot as plt

class Link:
    def __init__(self, pos_start, length):
        self.length = length 
        self.point_start = list(pos_start)
        # Start facing straight up along the Y-axis
        self.point_end = [self.point_start[0], self.point_start[1] + length]


class RobotArm:
    def __init__(self, links):
        self.links = links

    def rotate_point(self, point, pivot, angle_deg):
        """Helper to rotate a 2D point around a pivot point."""
        radians = math.radians(angle_deg * -1)
        cos = math.cos(radians)
        sin = math.sin(radians)

        dx = point[0] - pivot[0]
        dy = point[1] - pivot[1]

        nx = cos * dx - sin * dy + pivot[0]
        ny = sin * dx + cos * dy + pivot[1]
        return [round(nx, 4), round(ny, 4)]

    def rotate(self, link_index, angleInDegrees):
        """Rotates the target link AND rotates all downstream links around the same pivot."""
        pivot = self.links[link_index].point_start
        
        # Rotate the end of the current link
        self.links[link_index].point_end = self.rotate_point(
            self.links[link_index].point_end, pivot, angleInDegrees
        )

        # CRITICAL FIX: Rotate BOTH start and end points of all children around the parent pivot
        for i in range(link_index + 1, len(self.links)):
            self.links[i].point_start = self.rotate_point(self.links[i].point_start, pivot, angleInDegrees)
            self.links[i].point_end = self.rotate_point(self.links[i].point_end, pivot, angleInDegrees)

    def calc_distance(self, p1, p2):
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    def inverse_kinematics(self, target_pos, max_iterations=30, tolerance=0.01):
        history = []
        history.append([(list(l.point_start), list(l.point_end)) for l in self.links])
        
        for iteration in range(max_iterations):
            ee_pos = self.links[-1].point_end
            if self.calc_distance(ee_pos, target_pos) < tolerance:
                print(f"Target reached in {iteration} iterations!")
                break

            for i in reversed(range(len(self.links))):
                current_link = self.links[i]
                joint_center = current_link.point_start
                ee_pos = self.links[-1].point_end

                vector_ee = [ee_pos[0] - joint_center[0], ee_pos[1] - joint_center[1]]
                vector_target = [target_pos[0] - joint_center[0], target_pos[1] - joint_center[1]]

                mag_ee = math.sqrt(vector_ee[0]**2 + vector_ee[1]**2)
                mag_target = math.sqrt(vector_target[0]**2 + vector_target[1]**2)

                if mag_ee < 0.001 or mag_target < 0.001:
                    continue

                v_ee_norm = [vector_ee[0] / mag_ee, vector_ee[1] / mag_ee]
                v_target_norm = [vector_target[0] / mag_target, vector_target[1] / mag_target]

                dot_product = v_ee_norm[0] * v_target_norm[0] + v_ee_norm[1] * v_target_norm[1]
                dot_product = max(-1.0, min(1.0, dot_product))
                
                angle_rad = math.acos(dot_product)
                angle_deg = math.degrees(angle_rad)

                cross_product = (v_ee_norm[0] * v_target_norm[1]) - (v_ee_norm[1] * v_target_norm[0])
                if cross_product < 0:
                    angle_deg = -angle_deg

                if abs(angle_deg) > 0.0001:
                    self.rotate(i, angle_deg)
                    history.append([(list(l.point_start), list(l.point_end)) for l in self.links])
                    
        return history

def visualize_arm_movement(history, target_pos):
    fig, ax = plt.subplots(figsize=(6, 6))
    
    # Plot initial position
    for idx, (start, end) in enumerate(history[0]):
        ax.plot([start[0], end[0]], [start[1], end[1]], 'r--', alpha=0.6, label='Initial State' if idx == 0 else "")

    # Plot progression trails
    for step in history[1:-1]:
        for start, end in step:
            ax.plot([start[0], end[0]], [start[1], end[1]], 'gray', alpha=0.1)

    # Plot final successful convergence state
    for idx, (start, end) in enumerate(history[-1]):
        ax.plot([start[0], end[0]], [start[1], end[1]], 'b-o', linewidth=3, label='Final Arm Position' if idx == 0 else "")
        
    ax.plot(target_pos[0], target_pos[1], 'gX', markersize=12, label='Target Destination')
    ax.set_xlim(-1.5, 3.5)
    ax.set_ylim(-1.5, 3.5)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':')
    ax.legend(loc='upper left')
    ax.set_title('Corrected CCD Inverse Kinematics')
    plt.show()

# --- Run Test Configuration ---
links = [Link([0.0, 0.0], 1.0), Link([0.0, 1.0], 1.0), Link([0.0, 2.0], 1.0)]
arm = RobotArm(links) 

target_pos = [2.0, 0.0]
arm_history = arm.inverse_kinematics(target_pos)
visualize_arm_movement(arm_history, target_pos)
