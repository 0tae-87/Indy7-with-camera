import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import ikpy.chain
import numpy as np
import threading
import os
from scipy.spatial.transform import Rotation as R

class Indy7TCPControl(Node):
    def __init__(self):
        super().__init__('indy7_tcp_control')
        
        # URDF setup
        urdf_path = os.path.join(os.getcwd(), "urdf/indy7.urdf")
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path, 
            base_elements=["world"],
            last_link_vector=[0, 0, 0.06]  # TCP offset from link6
        )

        # Joint names
        self.arm_joints = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.gripper_joints = ['left_finger_joint', 'right_finger_joint']
        self.all_joints = self.arm_joints + self.gripper_joints

        # Active links mask for arm joints only
        num_links = len(self.chain.links)
        mask = [False] * num_links
        for i, link in enumerate(self.chain.links):
            if any(name in link.name for name in self.arm_joints):
                mask[i] = True
        self.chain.active_links_mask = mask

        # Current positions
        self.current_arm_pos = [0.0] * 6
        self.current_full_joints = [0.0] * num_links
        self.current_gripper_pos = [0.0, 0.0]
        
        # TCP offset (gripper base + finger length when fully extended)
        # Adjust this based on your actual gripper configuration
        self.tcp_offset = np.array([0.0, 0.0, 0.16])  # 0.06 (base) + 0.10 (finger)

        # Publisher & Subscriber
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        self.get_logger().info('TCP Control Node Started')
        self.get_logger().info(f'TCP offset: {self.tcp_offset}')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if name in self.arm_joints:
                idx = self.arm_joints.index(name)
                self.current_arm_pos[idx] = msg.position[i]
                for j, link in enumerate(self.chain.links):
                    if name in link.name:
                        self.current_full_joints[j] = msg.position[i]
            if name in self.gripper_joints:
                g_idx = self.gripper_joints.index(name)
                self.current_gripper_pos[g_idx] = msg.position[i]

    def get_current_tcp_pose(self):
        """Calculate current TCP position in world frame"""
        fk_result = self.chain.forward_kinematics(self.current_full_joints)
        tcp_pos = fk_result[:3, 3] + fk_result[:3, :3] @ self.tcp_offset
        return tcp_pos

    def send_command(self, positions, duration_sec):
        """Send joint trajectory command"""
        msg = JointTrajectory()
        msg.joint_names = self.all_joints
        
        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start.sec = duration_sec
        p.time_from_start.nanosec = 0
        
        msg.points.append(p)
        self.publisher.publish(msg)

    def control_gripper(self, dist):
        """Control gripper opening distance"""
        safe_val = float(np.clip(dist, 0.001, 0.038))
        target = self.current_arm_pos + [safe_val, safe_val]
        self.send_command(target, 2)
        print(f"✓ Gripper distance: {safe_val:.3f}m")

    def go_home(self):
        """Move to home position"""
        home = [0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0]
        target = home + self.current_gripper_pos
        self.send_command(target, 4)
        print("✓ Moving to home position")

    def move_tcp_to_xyz(self, x, y, z, roll=None, pitch=None, yaw=None):
        """
        Move TCP to specified XYZ coordinates with optional RPY orientation
        
        Args:
            x, y, z: Target TCP position in world frame (meters)
            roll, pitch, yaw: Target orientation in radians (None = keep current)
        """
        try:
            target_pos = np.array([x, y, z])
            
            # Create orientation matrix from RPY if provided
            if roll is not None or pitch is not None or yaw is not None:
                # Get current orientation if not all angles provided
                fk_current = self.chain.forward_kinematics(self.current_full_joints)
                current_rot = R.from_matrix(fk_current[:3, :3])
                current_rpy = current_rot.as_euler('xyz', degrees=False)
                
                # Use provided angles or keep current
                final_roll = roll if roll is not None else current_rpy[0]
                final_pitch = pitch if pitch is not None else current_rpy[1]
                final_yaw = yaw if yaw is not None else current_rpy[2]
                
                # Create rotation matrix
                orientation = R.from_euler('xyz', [final_roll, final_pitch, final_yaw]).as_matrix()
                
                # Create target transformation matrix
                target_matrix = np.eye(4)
                target_matrix[:3, :3] = orientation
                target_matrix[:3, 3] = target_pos
                
                # Use target_orientation parameter instead of passing matrix
                ik_result = self.chain.inverse_kinematics(
                    target_position=target_pos,
                    target_orientation=orientation,
                    orientation_mode='all',
                    initial_position=self.current_full_joints
                )
            else:
                # Position only IK
                ik_result = self.chain.inverse_kinematics(
                    target_position=target_pos,
                    initial_position=self.current_full_joints
                )
            
            # Extract arm joint positions from IK result
            arm_positions = [ik_result[i] for i, m in enumerate(self.chain.active_links_mask) if m]
            
            # Validate joint limits
            if not self.validate_joint_positions(arm_positions):
                print("⚠ Warning: Some joints may be near limits")
            
            # Send command
            target = arm_positions + self.current_gripper_pos
            self.send_command(target, 3)
            
            # Calculate achieved TCP position for verification
            fk_check = self.chain.forward_kinematics(ik_result)
            achieved_tcp = fk_check[:3, 3] + fk_check[:3, :3] @ self.tcp_offset
            error = np.linalg.norm(achieved_tcp - target_pos)
            
            print(f"✓ Moving TCP to [{x:.3f}, {y:.3f}, {z:.3f}]")
            if roll is not None or pitch is not None or yaw is not None:
                print(f"  RPY: [R={final_roll:.3f}, P={final_pitch:.3f}, Y={final_yaw:.3f}] rad")
            print(f"  Position error: {error*1000:.2f}mm")
            
            return True
            
        except Exception as e:
            print(f"✗ IK Error: {e}")
            import traceback
            traceback.print_exc()
            return False

    def validate_joint_positions(self, positions):
        """Check if joint positions are within safe limits"""
        # Joint limits from URDF (radians)
        limits = [
            (-3.054, 3.054),  # joint0
            (-3.054, 3.054),  # joint1
            (-3.054, 3.054),  # joint2
            (-3.054, 3.054),  # joint3
            (-3.054, 3.054),  # joint4
            (-3.752, 3.752),  # joint5
        ]
        
        valid = True
        for i, (pos, (low, high)) in enumerate(zip(positions, limits)):
            if pos < low or pos > high:
                print(f"  Joint {i} out of range: {pos:.3f} (limits: {low:.3f} to {high:.3f})")
                valid = False
        
        return valid

    def show_current_pose(self):
        """Display current TCP position and orientation"""
        tcp_pos = self.get_current_tcp_pose()
        fk_result = self.chain.forward_kinematics(self.current_full_joints)
        rot = R.from_matrix(fk_result[:3, :3])
        rpy = rot.as_euler('xyz', degrees=False)
        
        print(f"\n📍 Current TCP Position:")
        print(f"   X: {tcp_pos[0]:.4f}m")
        print(f"   Y: {tcp_pos[1]:.4f}m")
        print(f"   Z: {tcp_pos[2]:.4f}m")
        print(f"\n🔄 Current TCP Orientation (rad):")
        print(f"   Roll:  {rpy[0]:.4f}")
        print(f"   Pitch: {rpy[1]:.4f}")
        print(f"   Yaw:   {rpy[2]:.4f}")
        print(f"\n📐 Current Joint Positions (rad):")
        for i, pos in enumerate(self.current_arm_pos):
            print(f"   Joint{i}: {pos:.4f}")
        print()




def input_loop(node):
    """Interactive command loop"""
    print("\n" + "="*60)
    print("  INDY7 TCP CONTROL")
    print("="*60)
    print("\nCommands:")
    print("  x y z              - Move TCP to XYZ (meters)")
    print("  x y z r p y        - Move TCP to XYZ with RPY (radians)")
    print("  home               - Move to home position")
    print("  open               - Open gripper (0.04m)")
    print("  close              - Close gripper (0.001m)")
    print("  grip [dist]        - Set gripper distance (0.001-0.04m)")
    print("  pos                - Show current TCP position & orientation")
    print("  exit               - Exit program")
    print("="*60 + "\n")
    
    while rclpy.ok():
        try:
            cmd = input("indy7> ").strip().lower()
            
            if not cmd:
                continue
            elif cmd == 'exit':
                break
            elif cmd == 'home':
                node.go_home()
            elif cmd == 'open':
                node.control_gripper(0.04)
            elif cmd == 'close':
                node.control_gripper(0.001)
            elif cmd == 'pos':
                node.show_current_pose()
            elif cmd.startswith('grip'):
                parts = cmd.split()
                if len(parts) == 2:
                    dist = float(parts[1])
                    node.control_gripper(dist)
                else:
                    print("Usage: grip [distance]")
            else:
                # Parse coordinates
                coords = list(map(float, cmd.split()))
                if len(coords) == 3:
                    # XYZ only
                    node.move_tcp_to_xyz(coords[0], coords[1], coords[2])
                elif len(coords) == 6:
                    # XYZ + RPY
                    node.move_tcp_to_xyz(coords[0], coords[1], coords[2], 
                                        coords[3], coords[4], coords[5])
                else:
                    print("⚠ Invalid input. Enter 3 (XYZ) or 6 (XYZ+RPY) values")
                    
        except ValueError:
            print("⚠ Invalid input. Please enter numbers.")
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except Exception as e:
            print(f"⚠ Error: {e}")


def main():
    rclpy.init()
    node = Indy7TCPControl()
    
    # Give time for initial joint state
    import time
    time.sleep(1.0)
    
    # Start ROS2 spinning in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Run interactive loop in main thread
    input_loop(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()