import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors
import csv

class LidarICPNode(Node):
    def __init__(self):
        super().__init__('lidar_icp_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(Pose, '/robot_pose', 10)  # Publisher for Pose message
        self.prev_scan = None  # Store the previous scan
        self.pose = np.array([1,3,0])  # Initial pose: [x, y, theta]
        self.csv_file = open('robot_pose.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'theta'])  # Write CSV header
        self.get_logger().info('Lidar ICP Node has started.')

    def lidar_callback(self, msg):
        # Convert LaserScan data to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        points = np.array([
            ranges * np.cos(angles),
            ranges * np.sin(angles)
        ]).T

        # Remove invalid points (e.g., range = 0)
        points = points[np.linalg.norm(points, axis=1) > 0]

        if self.prev_scan is not None:
            # Apply ICP to calculate the relative transformation
            delta_pose = self.icp(self.prev_scan, points)
            self.pose = self.update_pose(self.pose, delta_pose)
            self.get_logger().info(f'Updated Pose: {self.pose}')
            # Save the pose to the CSV file
            self.csv_writer.writerow(self.pose)

            # Create Pose message
            pose_msg = Pose()
            pose_msg.position.x = self.pose[0]
            pose_msg.position.y = self.pose[1]
            pose_msg.position.z = 0.0  # Assuming flat 2D plane

            # Convert theta to quaternion
            quat = self.euler_to_quaternion(self.pose[2])
            pose_msg.orientation.x = quat[0]
            pose_msg.orientation.y = quat[1]
            pose_msg.orientation.z = self.pose[2]
            pose_msg.orientation.w = quat[3]

            # Publish the pose
            self.publisher.publish(pose_msg)

        self.prev_scan = points

    def icp(self, source, target, max_iterations=200, tolerance=1e-15):
        # ICP implementation (same as in the original code)
        """
        Perform ICP between source and target point clouds.
        Args:
            source: np.ndarray of shape (N, 2) - previous scan
            target: np.ndarray of shape (M, 2) - current scan
            max_iterations: Maximum number of ICP iterations
            tolerance: Convergence tolerance
        Returns:
            np.ndarray: Estimated transformation [dx, dy, dtheta]
        """
        source_copy = source.copy()
        transformation = np.eye(3)

        for i in range(max_iterations):
            # Find the nearest neighbors between the source and target
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target)
            distances, indices = nbrs.kneighbors(source_copy)

            # Compute the centroids of the matched points
            target_matched = target[indices[:, 0]]
            source_centroid = np.mean(source_copy, axis=0)
            target_centroid = np.mean(target_matched, axis=0)

            # Subtract centroids to align the points
            source_centered = source_copy - source_centroid
            target_centered = target_matched - target_centroid

            # Compute the optimal rotation using SVD
            H = np.dot(source_centered.T, target_centered)
            U, _, Vt = np.linalg.svd(H)
            R_opt = np.dot(Vt.T, U.T)

            # Ensure R_opt is a proper rotation matrix
            if np.linalg.det(R_opt) < 0:
                Vt[1, :] *= -1
                R_opt = np.dot(Vt.T, U.T)

            # Compute the translation
            t_opt = target_centroid - np.dot(source_centroid, R_opt)

            # Update the transformation matrix
            current_transform = np.eye(3)
            current_transform[:2, :2] = R_opt
            current_transform[:2, 2] = t_opt
            transformation = np.dot(current_transform, transformation)

            # Apply the transformation to the source points
            source_copy = (np.dot(R_opt, source_copy.T).T + t_opt)

            # Check for convergence
            mean_error = np.mean(distances)
            if mean_error < tolerance:
                break

        # Extract translation and rotation (angle) from the final transformation
        dx = transformation[0, 2]
        dy = transformation[1, 2]
        dtheta = np.arctan2(transformation[1, 0], transformation[0, 0])

        return np.array([-dx, -dy, -dtheta])


    def update_pose(self, pose, delta_pose):
        """
        Update the robot's pose based on the delta pose.
        Args:
            pose: Current pose [x, y, theta]
            delta_pose: Change in pose [dx, dy, dtheta]
        Returns:
            np.ndarray: Updated pose [x, y, theta]
        """
        dx, dy, dtheta = delta_pose
        theta = pose[2]

        # Update the pose with respect to the robot's current orientation
        x_new = pose[0] + dx * np.cos(theta) - dy * np.sin(theta)
        y_new = pose[1] + dx * np.sin(theta) + dy * np.cos(theta)
        theta_new = self.normalize_angle_rad(pose[2] + dtheta)

        return np.array([x_new, y_new, theta_new])

    def normalize_angle_rad(self, angle):
        """Normalize angle to range [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    # def normalize_angle_angle(self, angle):
    #     """Normalize angle to range [0,360]."""
    #     while angle > 360:
    #         angle -= 360
    #     while angle <0:
    #         angle += 360
    #     return angle
    def euler_to_quaternion(self, theta):
        """Convert Euler angle (theta) to quaternion."""
        r = R.from_euler('z', theta)
        quat = r.as_quat()
        return quat

    def destroy_node(self):
        # Close the CSV file when shutting down
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarICPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
