import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Bool
from geometry_msgs.msg import PoseArray
import numpy as np
import json
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from gabiru_track.segmentation import segment_track_by_curvature

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        self.srv = self.create_service(Trigger,'ready_to_recive_path',self.ready_callback) #1             
        self.client = self.create_client(Trigger,'ready_to_recive_optimization')#3
        self.publisher_ = self.create_publisher(String, 'gabiru/segments', 1000) #4 
        
        self.published = False  # Para publicar solo una vez
        
        self.waypoints = None
        self.get_logger().info("Esperando waypoints en el tópico /optimal_path...")

    def ready_callback(self,request, response):
        self.subscription = self.create_subscription(#2
            PoseArray,
            '/optimal_path',
            self.path_callback,
            10)
        response.success = True
        response.message = "PathOptimizer listo para recibir datos"
        self.get_logger().info("Servicio ready_to_optimize: Suscripción a /processed_data activada")
        return response   

    def path_callback(self, msg):
        if self.published:            
            return

        # Convertir los waypoints del mensaje PoseArray a un array de numpy
        waypoints = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            waypoints.append([x, y])
        self.waypoints = np.array(waypoints)
        self.wait_for_ready()
        

    def wait_for_ready(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the Optimizer Node start")
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.ready_response_callback)

    def ready_response_callback(self,future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"PathExecutor listo: {response.message}")
                self.load_segment_and_publish()
            else:
                self.get_logger().error(f"Fallo en la preparación: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error al llamar al servicio: {e}")

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