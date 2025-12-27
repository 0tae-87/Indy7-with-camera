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
        
        # YOLOv8n (nano) 사용 - 빠르고 가벼움
        self.model = YOLO('yolov8n.pt').to(self.device)
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
            
            # 매 프레임 YOLO 실행
            results = self.model(cv_image, verbose=False, imgsz=416, conf=0.2)
            
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    label = self.model.names[int(box.cls[0])]
                    conf = float(box.conf[0])
                    
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    if self.latest_pcd is not None:
                        try:
                            pts = list(point_cloud2.read_points(
                                self.latest_pcd, 
                                field_names=("x", "y", "z"), 
                                uvs=[[cx, cy]]
                            ))
                            if pts and not np.isnan(pts[0][0]):
                                is_horiz = (x2 - x1) > (y2 - y1)
                                self.target_pose = self.get_v2_base_pose(pts[0], is_horiz)
                                if self.target_pose:
                                    cv2.putText(cv_image, f"{label} {conf:.2f} Z={self.target_pose['z']:.2f}m", 
                                                (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                else:
                                    cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1-10), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            else:
                                cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1-10), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        except:
                            cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1-10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    else:
                        cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Indy7 v2 Detection", cv_image)
            cv2.waitKey(1)
        finally:
            self.lock.release()

    def get_v2_base_pose(self, cam_point, is_horiz):
        try:
            p = PointStamped()
            # URDF에 정의된 카메라 광학 프레임 사용 [cite: 101]
            p.header.frame_id = "zed2i_left_camera_optical_frame"
            p.point.x, p.point.y, p.point.z = cam_point
            
            # 💡 v2 프레임 이름인 'link0'를 타겟으로 변환 
            # 카메라의 0.08m X축 오프셋이 이 변환 과정에서 자동으로 계산됩니다. [cite: 100]
            trans = self.tf_buffer.lookup_transform('link0', p.header.frame_id, rclpy.time.Time())
            base_p = do_transform_point(p, trans)
            
            # Orientation: 그리퍼가 바닥을 향하도록(Pitch 90도) 설정 [cite: 30-38, 68-69]
            # 물체 방향에 따라 Yaw를 0 또는 90도 회전 [cite: 68-69]
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