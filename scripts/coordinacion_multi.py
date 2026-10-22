#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class AntiVicsekCoordinator(Node):
    def __init__(self):
        super().__init__('anti_vicsek_coordinator')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        # Parámetros configurables
        self.declare_parameter('namespace', 'tb1')
        self.declare_parameter('target_robot', 'tb2')
        self.declare_parameter('comm_range', 3.0) # Distancia de activación

        self.ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.target = self.get_parameter('target_robot').get_parameter_value().string_value
        self.r_comm = self.get_parameter('comm_range').get_parameter_value().double_value

        self.my_pose = None
        self.neighbor_pose = None

        # Suscripciones (Nótese que NO llevan / inicial para que sean relativas al namespace del nodo)
        self.create_subscription(Odometry, 'odom', self.my_odom_callback, 10)
        self.create_subscription(Odometry, f'/{self.target}/odom', self.neighbor_odom_callback, 10)

        # Publicador
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer (10Hz)
        self.create_timer(0.5, self.control_loop)
        self.get_logger().info(f"Anti-Vicsek activo para {self.ns} evitando a {self.target}")

    def my_odom_callback(self, msg):
        self.my_pose = msg.pose.pose

    def neighbor_odom_callback(self, msg):
        self.neighbor_pose = msg.pose.pose

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.my_pose is None or self.neighbor_pose is None:
            return

        dx = self.my_pose.position.x - self.neighbor_pose.position.x
        dy = self.my_pose.position.y - self.neighbor_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        # --- LÓGICA DE PRIORIDAD JERÁRQUICA ---
        # Solo el robot con "menor prioridad" (nombre mayor alfabéticamente) cede el paso.
        # tb2 > tb1, por lo tanto tb2 es quien esquiva.
        
        my_priority_id = self.ns  # Ejemplo: 'tb1' o 'tb2'
        neighbor_id = self.target  # Ejemplo: 'tb2' o 'tb1'

        if dist < 1.2: #Originalmente 1.2
            if my_priority_id > neighbor_id:
                # Yo soy tb2, el vecino es tb1. Yo debo esquivar.
                self.get_logger().info(f"Cediendo paso a {neighbor_id}", throttle_duration_sec=2.0)
                self.apply_anti_vicsek_logic(dx, dy)
            else:
                # Yo soy tb1, tengo prioridad. No publico nada en cmd_vel
                # para que Nav2 siga su curso sin interrupciones.
                pass
    def apply_anti_vicsek_logic(self, dx, dy):
        # Invertimos dx y dy para generar un vector de REPULSIÓN, no de atracción
        target_yaw = math.atan2(-dy, -dx) 
        my_yaw = self.get_yaw(self.my_pose.orientation)

        yaw_error = target_yaw - my_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()
        twist.angular.z = 0.8 * yaw_error 
        twist.linear.x = 0.0 
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AntiVicsekCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# --- ESTAS DOS LÍNEAS SON LAS QUE FALTAN ---
if __name__ == '__main__':
    main()