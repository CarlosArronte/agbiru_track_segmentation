import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Bool
from geometry_msgs.msg import PoseArray
import numpy as np
import json

from ament_index_python.packages import get_package_share_directory
from gabiru_track.segmentation import segment_track_by_curvature

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        self.publisher_ = self.create_publisher(String, 'gabiru/segments', 1000)
        self.subscription = self.create_subscription(
            PoseArray,
            '/optimal_path',
            self.path_callback,
            10)
        self.can_recive_best_path_publisher= self.create_publisher(Bool, '/can_recive_best_path', 10)
        self.can_recive_best_path = False
        self.published = False  # Para publicar solo una vez
        self.start_published = False
        self.timer = self.create_timer(1.0, self.publish_start)
        self.waypoints = None
        self.get_logger().info("Esperando waypoints en el tópico /optimal_path...")

    def publish_start(self):
        if self.start_published:
            return
        self.start_published = True
        msg = Bool()
        msg.data = True
        self.can_recive_best_path_publisher.publish(msg)
        self.get_logger().info("Publicado can_recive_best_path=True")
        self.can_recive_best_path = True

    def path_callback(self, msg):
        if self.published or not self.can_recive_best_path:            
            return

        # Convertir los waypoints del mensaje PoseArray a un array de numpy
        waypoints = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            waypoints.append([x, y])
        self.waypoints = np.array(waypoints)

        # Procesar y publicar los segmentosージャ

        self.load_segment_and_publish()

    def load_segment_and_publish(self):
        if self.published or self.waypoints is None:
            return
        
        # Esperar al suscriptor
        while self.publisher_.get_subscription_count() == 0:
            self.get_logger().info("Esperando a que se conecte el optimizador...")
            rclpy.spin_once(self, timeout_sec=2)

        # Segmentar los waypoints recibidos
        segments = segment_track_by_curvature(self.waypoints)

        for i, segment in enumerate(segments):
            tipo = segment["tipo"]
            indices = segment["indices"]
            segment_wp = self.waypoints[indices, :].tolist()  # Los wp reales del segmento como lista de listas

            data = {
                "segment_id": i,
                "tipo": tipo,
                "waypoints": segment_wp
            }
            msg = String()
            msg.data = json.dumps(data)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publicado segmento {i}: tipo={tipo}, puntos={len(indices)}")

        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()