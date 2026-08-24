import numpy as np
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class ParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        # 1. Parameter initialisieren
        self.num_particles = 500
        self.initial_pose = [0.0, 0.0, 0.0]  # Start: x, y, theta

        # Rauschparameter für das Odometrie-Bewegungsmodell (alphas)
        self.alpha1 = 0.05  # Rotationsrauschen aus Rotation
        self.alpha2 = 0.05  # Rotationsrauschen aus Translation
        self.alpha3 = 0.10  # Translationsrauschen aus Translation
        self.alpha4 = 0.05  # Translationsrauschen aus Rotation

        # 2. Partikelmenge initialisieren (N Partikel um die Startpose + Gaußsches Rauschen)
        self.particles = np.zeros((self.num_particles, 3))
        self.init_particles(self.initial_pose, std_pos=0.05, std_theta=0.02)

        # 3. Odometrie-Zwischenspeicher
        self.last_odom_pose = None

        # 4. Subscriber & Publisher
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.pose_array_pub = self.create_publisher(
            PoseArray,
            '/particlecloud',
            10
        )

        self.get_logger().info('ParticleFilter Node erfolgreich gestartet.')

    def init_particles(self, initial_pose, std_pos=0.05, std_theta=0.02):
        """Initialisiert N Partikel um die Startpose mit Gaußschem Rauschen."""
        self.particles[:, 0] = np.random.normal(initial_pose[0], std_pos, self.num_particles)
        self.particles[:, 1] = np.random.normal(initial_pose[1], std_pos, self.num_particles)
        self.particles[:, 2] = np.random.normal(initial_pose[2], std_theta, self.num_particles)
        # Winkel auf [-pi, pi] normalisieren
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]), np.cos(self.particles[:, 2]))

    def normalize_angle(self, angle):
        """Hilfsfunktion zur Normalisierung von Winkeln auf [-pi, pi]."""
        return np.arctan2(np.sin(angle), np.cos(angle))

    def odom_callback(self, msg: Odometry):
        # Aktuelle Roboterpose aus der Odometrie extrahieren
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

        current_odom_pose = np.array([x, y, theta])

        if self.last_odom_pose is None:
            self.last_odom_pose = current_odom_pose
            self.publish_particles(msg.header.stamp)
            return

        # 1. Odometrie-Differenzen berechnen (rot1, trans, rot2)
        dx = current_odom_pose[0] - self.last_odom_pose[0]
        dy = current_odom_pose[1] - self.last_odom_pose[1]
        dtheta = current_odom_pose[2] - self.last_odom_pose[2]

        trans = math.sqrt(dx**2 + dy**2)
        
        # Schwellenwertprüfung: Nur updaten, wenn Bewegung stattfand
        if trans > 0.001 or abs(self.normalize_angle(dtheta)) > 0.001:
            rot1 = self.normalize_angle(math.atan2(dy, dx) - self.last_odom_pose[2])
            rot2 = self.normalize_angle(current_odom_pose[2] - self.last_odom_pose[2] - rot1)

            # 2. Bewegungsmodell (Prediction) auf alle Partikel anwenden
            self.predict(rot1, trans, rot2)
            self.last_odom_pose = current_odom_pose

        # 3. Partikel in RViz anzeigen
        self.publish_particles(msg.header.stamp)

    def predict(self, rot1, trans, rot2):
        """Wendet das Odometrie-Bewegungsmodell mit Gaußschem Rauschen an."""
        # Standardabweichungen für die einzelnen Bewegungskomponenten
        std_rot1 = np.sqrt(self.alpha1 * rot1**2 + self.alpha2 * trans**2)
        std_trans = np.sqrt(self.alpha3 * trans**2 + self.alpha4 * (rot1**2 + rot2**2))
        std_rot2 = np.sqrt(self.alpha1 * rot2**2 + self.alpha2 * trans**2)

        # Verrauschte Steuerungsbefehle generieren (vektorisiert für alle N Partikel)
        noisy_rot1 = rot1 + np.random.normal(0, std_rot1, self.num_particles)
        noisy_trans = trans + np.random.normal(0, std_trans, self.num_particles)
        noisy_rot2 = rot2 + np.random.normal(0, std_rot2, self.num_particles)

        # Partikelposen aktualisieren
        self.particles[:, 0] += noisy_trans * np.cos(self.particles[:, 2] + noisy_rot1)
        self.particles[:, 1] += noisy_trans * np.sin(self.particles[:, 2] + noisy_rot1)
        self.particles[:, 2] += noisy_rot1 + noisy_rot2

        # Winkel im Bereich [-pi, pi] halten
        self.particles[:, 2] = self.normalize_angle(self.particles[:, 2])

    def publish_particles(self, stamp):
        """Erstellt ein PoseArray und publiziert es für RViz."""
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'  # oder 'map', je nach TF-Struktur

        for particle in self.particles:
            pose = Pose()
            pose.position.x = float(particle[0])
            pose.position.y = float(particle[1])
            pose.position.z = 0.0

            q = quaternion_from_euler(0.0, 0.0, float(particle[2]))
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            msg.poses.append(pose)

        self.pose_array_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()