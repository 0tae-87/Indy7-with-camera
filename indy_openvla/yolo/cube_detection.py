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
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from scipy.spatial.transform import Rotation as R

class IndyV2YoloDetector(Node):
    def __init__(self):
        super().__init__('indy_v2_yolo_detector')
        
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
        
        self.latest_pcd = None
        self.lock = threading.Lock()
        self.target_pose = None
        self.last_results = []  # 마지막 탐지 결과 저장
        
        self.get_logger().info("Indy7 v2 YOLO Detector Started.")

    def point_callback(self, msg):
        self.latest_pcd = msg

    def image_callback(self, img_msg):
        if not self.lock.acquire(blocking=False): return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            
            # YOLO detection
            results = self.model(cv_image, verbose=False, imgsz=416, conf=0.2)
            
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    label = self.model.names[int(box.cls[0])]
                    conf = float(box.conf[0])
                    
                    # Detect cube color from ROI and relabel
                    roi = cv_image[y1:y2, x1:x2]
                    color = self.detect_color(roi)
                    # If Roboflow model already detects 'cube', add color prefix
                    if 'cube' in label.lower():
                        display_label = f"{color}_{label}" if color else label
                    else:
                        display_label = f"{color}_cube" if color else label
                    
                    # Draw bounding box with color
                    box_color = self.get_box_color(color)
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), box_color, 2)
                    
                    if self.latest_pcd is not None:
                        try:
                            # Get point at (cx, cy) from organized point cloud
                            width = self.latest_pcd.width
                            height = self.latest_pcd.height
                            
                            if 0 <= cx < width and 0 <= cy < height:
                                # Calculate index in organized point cloud
                                index = cy * width + cx
                                
                                # Read single point
                                pts = list(point_cloud2.read_points(
                                    self.latest_pcd,
                                    field_names=("x", "y", "z"),
                                    skip_nans=False
                                ))
                                
                                if index < len(pts):
                                    cam_point = pts[index]
                                    if not np.isnan(cam_point[0]) and not np.isinf(cam_point[0]):
                                        is_horiz = (x2 - x1) > (y2 - y1)
                                        self.target_pose = self.get_v2_base_pose(cam_point, is_horiz)
                                        if self.target_pose:
                                            cv2.putText(cv_image, f"{display_label} {conf:.2f} Z={self.target_pose['z']:.2f}m", 
                                                        (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                                        else:
                                            cv2.putText(cv_image, f"{display_label} {conf:.2f}", (x1, y1-10), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                                    else:
                                        cv2.putText(cv_image, f"{display_label} {conf:.2f}", (x1, y1-10), 
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                        except Exception as e:
                            self.get_logger().error(f"Point cloud error: {e}")
                            cv2.putText(cv_image, f"{display_label} {conf:.2f}", (x1, y1-10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                    else:
                        cv2.putText(cv_image, f"{display_label} {conf:.2f}", (x1, y1-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

            cv2.imshow("YOLO Detection", cv_image)
            cv2.waitKey(1)
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
            return None



def main():
    rclpy.init()
    node = IndyV2YoloDetector()
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