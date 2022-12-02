import numpy as np
import open3d as o3d
from math import cos, sin, radians


class Kinematics:

    def __init__(self):
        # Fixture in relation to the 3d scanner
        self.T_fixture_3dscanner = self.populate_transform([40.5, 34.15, 30.25])
        # Fixture in relation to the mover
        self.T_fixture_moverc = self.populate_transform([25, 0, 99])
        # Galvo in relation to the mover
        self.T_galvo_moverc = self.populate_transform([60, 5, 245], 0, 180, 90)
        # Mover placed at the center of tile relative to the tile frame
        self.T_moverc_tile = self.populate_transform([120, 120, 2])
        # Mover placed at the edge of the tile on +X axis towards the Galvo laser
        self.T_moverg_tile = self.populate_transform([160, 120, 1], 0, 0, 180)
        # Mover placed at the edge of the tile on +Y axis towards the motor
        self.T_moverm_tile = self.populate_transform([120, 160, 2], 0, 0, 90)
        # Mover placed at the edge of the tile on -X axis towards the 3d scanner
        self.T_mover3d_tile = self.populate_transform([80, 120, 2])
        self.transformed_pts = []

    def populate_transform(self, translation, alpha=0, beta=0, gamma=0):
        """
        Function to populate the transformation between two frames using Euler angle set convention given by rotation
        R_xyz(alpha, beta, gamma) and translation [tx, ty, tz]
        :param translation: translation on x, y, and z
        :param alpha: rotation on x
        :param beta: rotation on y
        :param gamma: rotation on z
        :return: transformation
        """
        alpha = radians(alpha)
        beta = radians(beta)
        gamma = radians(gamma)
        return np.asarray([[cos(beta)*cos(gamma), -cos(beta)*sin(gamma), sin(beta), translation[0]],
                [sin(alpha)*sin(beta)*cos(gamma) + cos(alpha)*sin(gamma), -sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), -sin(alpha)*cos(beta), translation[1]],
                [-cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma), cos(alpha)*cos(beta), translation[2]],
                [0, 0, 0, 1]])

    def get_pt_ref_to_galvo(self, T_object_pt_3dscanner, T_feedback_from_mover):
        """
        Function to compute the relation between one point on the object and the Galvo laser scanner, given any
        change (translation and rotation) from the reference frame to any current frame
        :param T_object_3dscanner: object relative to the 3d scanner transformation
        :param T_feedback_from_mover: current position of mover relative to the tile center point (reference frame)
        :return: point on object relative to the galvo laser scanner head transformation
        """
        # Get fixture relative to object
        T_fixture_object = np.dot(np.linalg.inv(T_object_pt_3dscanner), self.T_fixture_3dscanner)

        # Get object relative to mover
        T_object_moverc = np.dot(self.T_fixture_moverc, np.linalg.inv(T_fixture_object))

        # Get the amount in mm on x and y between previous and current mover position
        T_how_much_it_moved = np.dot(np.linalg.inv(self.T_moverc_tile), T_feedback_from_mover)

        # Get object relative to the galvo head
        return np.dot(np.dot(np.linalg.inv(self.T_galvo_moverc), T_how_much_it_moved), T_object_moverc)

    def get_pts_ref_to_galvo(self, T_object_pts_3dscanner, T_feedback_from_mover):
        """
        Function to compute object points into the frame of the galvo laser scanner
        :param T_object_pts_3dscanner: List of transformations for each point on the object with respect to the galvo head
        :param T_feedback_from_mover: Current absolute position of the mover
        :return: Transformed points (only x and y values)
        """
        for pt in T_object_pts_3dscanner:
            output_transformation = self.get_pt_ref_to_galvo(pt, T_feedback_from_mover)
            self.transformed_pts.append(output_transformation[:, 3][:-2])
        return self.transformed_pts

# Example how to use the class, can also be called from another file
def  main():
    # Read .ply file
    source_cloud = o3d.io.read_point_cloud("clouds/KinematicTest.ply")  # Read the point cloud

    # Convert from PointCloud type into numpy array
    source_cloud_np = np.asarray(source_cloud.points)

    # Make an instance of the class called Kinematics
    kinematics = Kinematics()

    # These are test points, collect 3 points manually from the 3d scanner point cloud and see if laser goes there
    #T_object_pt_3dscanner1 = kinematics.populate_transform([-31.65, 6.37, 18.34])
    #T_object_pt_3dscanner2 = kinematics.populate_transform([-15.91, 34.81, 11.9])
    #T_object_pt_3dscanner3 = kinematics.populate_transform([-6.45, 61.21, 5.13])
    #T_object_pts_3dscanner = [T_object_pt_3dscanner1, T_object_pt_3dscanner2, T_object_pt_3dscanner3]

    # Get object points transformations relative to 3d scanner
    input_transformations = []
    for pt in source_cloud_np:
        input_transformations.append(kinematics.populate_transform(pt))

    # Get object points relative to the galvo head
    transformed_pts = kinematics.get_pts_ref_to_galvo(input_transformations, kinematics.T_moverg_tile)

    # Save points to XML file...

if __name__ == "__main__":
    main()