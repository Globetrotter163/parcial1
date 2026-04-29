"""
inverse_kinematics.py — brazo_custom (ROS 2)

Cadena cinemática derivada del URDF:
  joint_1: rot Z, origen [0,0,0],     RPY [0,0,0]      → q1 gira en plano XY
  joint_2: rot Z, origen [2,0,0],     RPY [π/2,0,0]    → q2 eleva (eje efectivo Y global)
  joint_3: rot Z, origen [1,0,0]rel,  RPY [−π/2,0,0]   → q3 vuelve al plano XY (cancelación)
  joint_ee: fixed,  origen [0,0,1]rel

Longitudes de eslabón:
  L1 = 2.0  (link_1, de base a joint_2)
  L2 = 1.0  (link_2, de joint_2 a joint_3)
  L3 = 1.0  (end-effector, de joint_3 a EE)

Cinemática directa derivada con matrices homogéneas T = T1 * T2 * T3 * T_ee
donde cada Ti = Trans(origin) * Rot_RPY * Rot(eje_local, qi).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np


# ── Utilidades de matrices homogéneas ──────────────────────────────────────────

def rot_x(a):
    """Matriz de rotación pura en X."""
    ca, sa = np.cos(a), np.sin(a)
    return np.array([
        [1,  0,   0,  0],
        [0,  ca, -sa, 0],
        [0,  sa,  ca, 0],
        [0,  0,   0,  1]], dtype=float)


def rot_z(a):
    """Matriz de rotación pura en Z."""
    ca, sa = np.cos(a), np.sin(a)
    return np.array([
        [ca, -sa, 0, 0],
        [sa,  ca, 0, 0],
        [0,   0,  1, 0],
        [0,   0,  0, 1]], dtype=float)


def trans(x, y, z):
    """Matriz de traslación pura."""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]], dtype=float)


# ── Cinemática directa ─────────────────────────────────────────────────────────

# Longitudes del URDF
L1 = 2.0   # joint_1 → joint_2  (origin.x del joint_2)
L2 = 1.0   # joint_2 → joint_3  (origin.x del joint_3)
L3 = 1.0   # joint_3 → EE       (origin.z del joint_ee)


def forward_kinematics(q):
    """
    Calcula la posición del end-effector dado el vector de joints q = [q1, q2, q3].

    Cada transformación parcial:
      T_j1 = Trans(0,0,0) * I_RPY * Rot_Z(q1)          → rotación base
      T_j2 = Trans(L1,0,0) * Rot_X(π/2) * Rot_Z(q2)   → elevación
      T_j3 = Trans(L2,0,0) * Rot_X(-π/2) * Rot_Z(q3)  → wrist planar
      T_ee = Trans(0,0,L3)                              → offset al EE (fixed)
    """
    q1, q2, q3 = q

    T1 = rot_z(q1)
    T2 = trans(L1, 0, 0) @ rot_x(np.pi / 2) @ rot_z(q2)
    T3 = trans(L2, 0, 0) @ rot_x(-np.pi / 2) @ rot_z(q3)
    T_ee = trans(0, 0, L3)

    T = T1 @ T2 @ T3 @ T_ee
    return T[:3, 3]


def jacobian_numerical(q, eps=1e-6):
    """
    Jacobiano numérico 3×3 por diferencias finitas.
    Más robusto que el analítico ante cambios de URDF.
    """
    p0 = forward_kinematics(q)
    J = np.zeros((3, 3))
    for i in range(3):
        dq = np.zeros(3)
        dq[i] = eps
        J[:, i] = (forward_kinematics(q + dq) - p0) / eps
    return J


# ── Nodo ROS 2 ────────────────────────────────────────────────────────────────

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber 
        self.target_sub = self.create_subscription(
            Point, 'target_position', self.target_callback, 10)

        # Posición inicial
        self.q = np.array([0.0, 0.0, 0.0])

        home = forward_kinematics(np.zeros(3))
        self.get_logger().info(f'Posición home (q=[0,0,0]): {home}')
        self.target_pos = home + np.array([0.5, 0.5, 0.2])

        # IK cfgs
        self.step_size = 0.5        
        self.max_iterations = 200
        self.tolerance = 0.005      
        self.damping_factor = 0.05   


        self.q_min = np.full(3, -np.pi)
        self.q_max = np.full(3,  np.pi)

        self.timer = self.create_timer(0.1, self.update_joints)


    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(
            f'Nuevo objetivo recibido: [{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}]')

    def update_joints(self):
        current_pos = forward_kinematics(self.q)
        error = self.target_pos - current_pos
        error_norm = np.linalg.norm(error)

        self.get_logger().info(
            f'EE actual: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]  '
            f'Error: {error_norm:.4f} m')

        if error_norm > self.tolerance:
            J = jacobian_numerical(self.q)

            JtJ = J.T @ J
            lam = self.damping_factor * np.eye(3)
            delta_q = np.linalg.solve(JtJ + lam, J.T @ error)
            self.q = np.clip(
                self.q + delta_q * self.step_size,
                self.q_min,
                self.q_max)

        # Publicar 
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.q.tolist()
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()