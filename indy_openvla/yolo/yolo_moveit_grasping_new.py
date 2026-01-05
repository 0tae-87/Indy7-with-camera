import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from ultralytics import YOLO
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
import torch
import threading
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_geometry_msgs import do_transform_point
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import ikpy.chain
import ikpy.utils.plot as plot_utils

class YoloMoveItGrasping(Node):
    def __init__(self):
        super().__init__('yolo_moveit_grasping')
        
        # GPU 사용 시도 (torchvision CUDA 지원 필요)
        if torch.cuda.is_available():
            try:
                # CUDA 테스트
                test_tensor = torch.zeros(1).cuda()
                self.device = 'cuda'
                self.get_logger().info("Running on GPU (CUDA).")
            except:
                self.device = 'cpu'
                self.get_logger().info("CUDA available but failed, using CPU.")
        else:
            self.device = 'cpu'
            self.get_logger().info("Running on CPU.")
        
        # Custom cube detection model
        model_path = '/home/wimait2/YT_ws/indy7/src/indy-ros2/indy_openvla/yolo/cube_weights.pt'
        self.model = YOLO(model_path).to(self.device)
        self.get_logger().info("Using custom cube detection model")
        self.bridge = CvBridge()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.image_sub = self.create_subscription(Image, '/indy7/zed2i/image', self.image_callback, 1)
        self.point_sub = self.create_subscription(PointCloud2, '/indy7/zed2i/points', self.point_callback, 1)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.latest_pcd = None
        self.lock = threading.Lock()
        self.detections = {}
        self.selected_color = None
        self.current_joint_positions = [0.0] * 6
        
        self.arm_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Load URDF chain for IK
        urdf_path = '/home/wimait2/YT_ws/indy7/src/indy-ros2/indy_openvla/urdf/indy7_v2.urdf'
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["world"],
            last_link_vector=[0, 0, 0.06]  # TCP offset from link6
        )
        
        # Set active links mask
        arm_joints = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        num_links = len(self.chain.links)
        mask = [False] * num_links
        for i, link in enumerate(self.chain.links):
            if any(name in link.name for name in arm_joints):
                mask[i] = True
        self.chain.active_links_mask = mask
        
        self.get_logger().info("Press r/y/g/b to select, m to move, h to go home, a to descend and grasp")

    def point_callback(self, msg):
        self.latest_pcd = msg
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        arm_joints = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        for i, name in enumerate(msg.name):
            if name in arm_joints:
                idx = arm_joints.index(name)
                self.current_joint_positions[idx] = msg.position[i]

    def image_callback(self, img_msg):
        if not self.lock.acquire(blocking=False): return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            
            # YOLO detection
            results = self.model(cv_image, verbose=False, imgsz=416, conf=0.2)
            
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    cx, cy = int((x1 + x2) // 2), int((y1 + y2) // 2)
                    label = self.model.names[int(box.cls[0])]
                    conf = float(box.conf[0])
                    
                    # Detect cube color from ROI and relabel
                    roi = cv_image[y1:y2, x1:x2]
                    color = self.detect_color(roi)
                    
                    # Draw bounding box with color
                    box_color = self.get_box_color(color)
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), box_color, 2)
                    
                    if self.latest_pcd is not None:
                        try:
                            # Get all points in bounding box
                            width = self.latest_pcd.width
                            height = self.latest_pcd.height
                            
                            # Read entire point cloud once
                            pts = list(point_cloud2.read_points(
                                self.latest_pcd,
                                field_names=("x", "y", "z"),
                                skip_nans=False
                            ))
                            
                            # Extract points within bounding box
                            points_in_box = []
                            for v in range(y1, y2):
                                for u in range(x1, x2):
                                    if 0 <= u < width and 0 <= v < height:
                                        index = v * width + u
                                        if index < len(pts):
                                            point = pts[index]
                                            if not (np.isnan(point[0]) or np.isinf(point[0])):
                                                # Convert to regular array
                                                points_in_box.append([float(point[0]), float(point[1]), float(point[2])])
                            
                            # Calculate average position
                            if len(points_in_box) > 0:
                                avg_point = np.mean(points_in_box, axis=0)
                                self.get_logger().info(f"Avg point in camera frame: ({avg_point[0]:.3f}, {avg_point[1]:.3f}, {avg_point[2]:.3f})")
                                is_horiz = (x2 - x1) > (y2 - y1)
                                pose = self.get_v2_base_pose(avg_point, is_horiz)
                                if pose and color:
                                    self.get_logger().info(f"{color} in base frame: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f})")
                                    self.detections[color] = pose
                                    cv2.putText(cv_image, f"{color} {conf:.2f}", (x1, y1-10), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                        except Exception as e:
                            import traceback
                            self.get_logger().error(f"Error: {e}\n{traceback.format_exc()}")

            cv2.imshow("YOLO Detection", cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'): 
                self.selected_color = 'red'
                self.get_logger().info("Selected RED")
            elif key == ord('y'): 
                self.selected_color = 'yellow'
                self.get_logger().info("Selected YELLOW")
            elif key == ord('g'): 
                self.selected_color = 'green'
                self.get_logger().info("Selected GREEN")
            elif key == ord('b'): 
                self.selected_color = 'blue'
                self.get_logger().info("Selected BLUE")
            elif key == ord('m'):
                self.get_logger().info(f"M pressed. selected_color={self.selected_color}, detections={list(self.detections.keys())}")
                if self.selected_color and self.selected_color in self.detections:
                    pose = self.detections[self.selected_color]
                    self.move_to_pose(pose)
                else:
                    self.get_logger().warn(f"Cannot move: selected={self.selected_color}, available={list(self.detections.keys())}")
            elif key == ord('h'):
                self.go_home()
                self.get_logger().info("Moving to home position")
            elif key == ord('a'):
                self.descend_and_grasp()
                self.get_logger().info("Descending to grasp")
        finally:
            self.lock.release()

    def detect_color(self, roi):
        """Detect dominant color in ROI using HSV thresholding"""
        if roi.size == 0:
            return None
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Color ranges in HSV
        red_mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        red_mask = red_mask1 | red_mask2
        
        yellow_mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))
        green_mask = cv2.inRange(hsv, np.array([40, 100, 100]), np.array([80, 255, 255]))
        blue_mask = cv2.inRange(hsv, np.array([100, 100, 100]), np.array([130, 255, 255]))
        
        # Count pixels
        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)
        blue_count = cv2.countNonZero(blue_mask)
        
        # Return dominant color
        max_count = max(red_count, yellow_count, green_count, blue_count)
        if max_count > roi.size * 0.1:  # At least 10% of ROI
            if max_count == red_count:
                return "red"
            elif max_count == yellow_count:
                return "yellow"
            elif max_count == green_count:
                return "green"
            elif max_count == blue_count:
                return "blue"
        return None
    
    def get_box_color(self, color):
        """Get BGR color for bounding box"""
        if color == "red":
            return (0, 0, 255)
        elif color == "yellow":
            return (0, 255, 255)
        elif color == "green":
            return (0, 255, 0)
        elif color == "blue":
            return (255, 0, 0)
        return (0, 255, 0)

    def get_v2_base_pose(self, cam_point, is_horiz):
        try:
            p = PointStamped()
            p.header.frame_id = "zed2i_camera_frame"
            p.point.x, p.point.y, p.point.z = cam_point
            
            trans = self.tf_buffer.lookup_transform('link0', p.header.frame_id, rclpy.time.Time())
            base_p = do_transform_point(p, trans)
            
            target_yaw = 0.0 if is_horiz else 1.57
            q = R.from_euler('xyz', [0.0, 1.57, target_yaw]).as_quat()
            
            return {
                'x': base_p.point.x, 
                'y': base_p.point.y, 
                'z': base_p.point.z, 
                'q': q
            }
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def move_to_pose(self, pose):
        """Move to pose using IK"""
        # Open gripper first
        self.open_gripper()
        
        # Target position
        target_pos = np.array([pose['x'], pose['y'], pose['z'] + 0.25])
        
        # Gripper pointing down: roll=0, pitch=180deg (3.14 rad), yaw=0
        target_orientation = R.from_euler('xyz', [0.0, 3.14, 0.0]).as_matrix()
        
        # Use current joint positions as initial guess
        current_full_joints = [0.0] * len(self.chain.links)
        for i, link in enumerate(self.chain.links):
            for j, name in enumerate(['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']):
                if name in link.name:
                    current_full_joints[i] = self.current_joint_positions[j]
        
        # Compute IK with orientation
        ik_result = self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orientation,
            orientation_mode='all',
            initial_position=current_full_joints
        )
        
        # Extract arm joint positions
        arm_positions = [ik_result[i] for i, m in enumerate(self.chain.active_links_mask) if m]
        
        # Validate joint limits
        if not self.validate_joint_limits(arm_positions):
            self.get_logger().error("Joint limits exceeded! Aborting motion.")
            return
        
        # Send to robot
        msg = JointTrajectory()
        msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = Duration(sec=3)
        msg.points = [point]
        
        self.arm_pub.publish(msg)
        self.get_logger().info(f"Moving to {self.selected_color} at ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']+0.25:.2f})")
    
    def open_gripper(self):
        """Open gripper to 4cm"""
        msg = JointTrajectory()
        msg.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.04, 0.04]
        point.time_from_start = Duration(sec=2)
        msg.points = [point]
        
        self.gripper_pub.publish(msg)
        self.get_logger().info("Gripper opened")
    
    def go_home(self):
        """Move to home position and cycle gripper"""
        home = [0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0]
        
        msg = JointTrajectory()
        msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        point = JointTrajectoryPoint()
        point.positions = home
        point.time_from_start = Duration(sec=4)
        msg.points = [point]
        
        self.arm_pub.publish(msg)
        
        # Cycle gripper after reaching home
        import time
        time.sleep(4.5)
        self.open_gripper()
        time.sleep(2.5)
        self.close_gripper()
    
    def descend_and_grasp(self):
        """Descend 15cm and close gripper"""
        # Get current TCP position
        current_full_joints = [0.0] * len(self.chain.links)
        for i, link in enumerate(self.chain.links):
            for j, name in enumerate(['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']):
                if name in link.name:
                    current_full_joints[i] = self.current_joint_positions[j]
        
        fk_result = self.chain.forward_kinematics(current_full_joints)
        current_pos = fk_result[:3, 3]
        
        # Target: 15cm down
        target_pos = current_pos.copy()
        target_pos[2] -= 0.15
        
        # Keep same orientation (gripper down)
        target_orientation = R.from_euler('xyz', [0.0, 3.14, 0.0]).as_matrix()
        
        # Compute IK
        ik_result = self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orientation,
            orientation_mode='all',
            initial_position=current_full_joints
        )
        
        # Extract arm joint positions
        arm_positions = [ik_result[i] for i, m in enumerate(self.chain.active_links_mask) if m]
        
        # Send to robot
        msg = JointTrajectory()
        msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = Duration(sec=2)
        msg.points = [point]
        
        self.arm_pub.publish(msg)
        
        # Close gripper after 2.5 seconds
        import time
        time.sleep(2.5)
        self.close_gripper()
    
    def close_gripper(self):
        """Close gripper"""
        msg = JointTrajectory()
        msg.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.001, 0.001]
        point.time_from_start = Duration(sec=2)
        msg.points = [point]
        
        self.gripper_pub.publish(msg)
        self.get_logger().info("Gripper closed")
    
    def validate_joint_limits(self, positions):
        """Check if joint positions are within safe limits"""
        limits = [
            (-3.054, 3.054),
            (-3.054, 3.054),
            (-3.054, 3.054),
            (-3.054, 3.054),
            (-3.054, 3.054),
            (-3.752, 3.752),
        ]
        
        for i, (pos, (low, high)) in enumerate(zip(positions, limits)):
            if pos < low or pos > high:
                self.get_logger().error(f"Joint {i} out of range: {pos:.3f} (limits: {low:.3f} to {high:.3f})")
                return False
        return True



def main():
    rclpy.init()
    node = YoloMoveItGrasping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()