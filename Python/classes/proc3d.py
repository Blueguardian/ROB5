import open3d as o3d
import os
import numpy as np
import copy
from math import cos, sin, radians

class Proc3D:

    laserpoints = np.array([])

    def __init__(self, path):
        self.__PATH = path
        self.object_type = ""
        self.points = o3d.geometry.PointCloud()
        self.centerlinecloud = o3d.geometry.PointCloud()


    @classmethod
    def compute_transform_fc_fd(cls, current_frame, desired_frame):
        """
        Get required transformation to get from current frame to desired frame
        :param current_frame: numpy array, containing transformation matrix of the current frame
        :param desired_frame: numpy array, containing transformation matrix of the desired frame
        :return: numpy array, transformation matrix between the two
        """
        tr = current_frame.transpose() * desired_frame
        return np.dot(np.linalg.inv(current_frame), desired_frame)

    @classmethod
    def populate_transform(cls, tx, ty, tz, alpha, beta, gamma):
        """
        Euler angles rotation on x (alpha), y (beta), z (gamma) and translation on x (tx), y (ty), z (tz)
        Euler angles assume that each rotation is done based on the previous rotation unlike fixed angles which
        rotate according to the initial frame
        :param tx: Double, translation along x-axis
        :param ty: Double, translation along y-axis
        :param tz: Double, translation along z-axis
        :param alpha: Rotation around x
        :param beta: Rotation around y
        :param gamma: Rotation around z
        :return: numpy array, transformation matrix for the transform
        """
        alpha = radians(alpha)
        beta = radians(beta)
        gamma = radians(gamma)
        return np.asarray([[cos(beta) * cos(gamma), -cos(beta) * sin(gamma), sin(beta), tx],
                        [sin(alpha) * sin(beta) * cos(gamma) + cos(alpha) * sin(gamma),
                        -sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma), -sin(alpha) * cos(beta),
                        ty],
                        [-cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
                        cos(alpha) * sin(beta) * sin(gamma) + sin(alpha) * cos(gamma), cos(alpha) * cos(beta), tz],
                        [0, 0, 0, 1]])

    def interpolation3D(self, pt1, pt2, y):
        x0 = pt1[0]
        y0 = pt1[1]
        z0 = pt1[2]
        x1 = pt2[0]
        y1 = pt2[1]
        z1 = pt2[2]
        # Calculate x given y, here the largest change in magnitude = y
        x = x0 + (x1 - x0) * ((y - y0) / (y1 - y0))
        # Calculate z given y
        z = z0 + (z1 - z0) * ((y - y0) / (y1 - y0))
        return x, z

    def numpylist_sliced_x_value(self, listlist, x_value):

        listlist = copy.deepcopy(listlist)
        indekser = []

        for i in range(0, len(listlist)):

            if x_value > 0:
                if listlist[i, 0] > x_value:
                    indekser.append(i)

            if x_value < 0:
                if listlist[i, 0] < x_value:
                    indekser.append(i)

        a = np.delete(listlist, indekser, axis=0)

        return a

    def load_points(self):
        """
        Load in points to the object from the .ply files provided in the folder
        get the intensity of the points and remove those above threshold.
        Lastly it will determine whether the object is a planar object or not, while
        it could determine the objects shape in general, this is not needed for
        this project
        ::__PATH:: Internal path to .ply file directory
        """

        intensity_threshold = 0.4
        if os.path.isfile(os.path.join(self.__PATH, 'pointcloud_0.ply')):
            source = o3d.io.read_point_cloud(os.path.join(self.__PATH, 'pointcloud_0.ply'))
            sourcepoints = np.asarray(source.points)
            temp_points = np.asarray(sourcepoints)
            temp_color = np.asarray(source.colors)
            intensity = np.asarray(source.colors)
            temp_cloud = np.asarray(source.points)[intensity[:, 0] <= intensity_threshold]

            temp_points = self.numpylist_sliced_x_value(temp_points, 20.0)
            self.points.points = o3d.utility.Vector3dVector(temp_points)
            self.points.colors = o3d.utility.Vector3dVector(temp_color)
            self.centerlinecloud = copy.deepcopy(self.points)
            self.centerlinecloud.points = o3d.utility.Vector3dVector(temp_cloud)

            o3d.visualization.draw_geometries([self.points])

            # Determine if the object is a plane or not
            points, indices = self.centerlinecloud.segment_plane(distance_threshold=3, ransac_n=3, num_iterations=100000)
            if temp_cloud.shape[0]*0.7 < np.ma.size(indices, axis=0):
                self.object_type = "plane"
            else:
                self.object_type = "other"
            return True
        else:
            print("No points available")
            return False

    def process_points(self, obj_type):
        """
        Process points to obtain the transformed points for the laser cell
        :return: Internal update of
        """

        self.laserpoints = np.array([])
        # euclidean clustering clustering
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(self.centerlinecloud.cluster_dbscan(eps=0.5, min_points=10, print_progress=True))

        # Pick the biggest cluster
        centerlinePoints = np.asarray(self.centerlinecloud.points)
        centerlinePoints = centerlinePoints[labels > -1]
        labels = labels[labels > -1]
        biggestLabel = np.bincount(labels).argmax()
        # Remove the unused labels twice because the vector must not contain negative for bincount to work.
        print(biggestLabel)
        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")

        # # concatenate biggest cluster into the original pointcloud
        centerlinePoints = centerlinePoints[labels == biggestLabel]

        # expand area to contain the
        # first define the box containing the weld seam, find the max and min of x and y
        Xmax, Ymax, Zmax = centerlinePoints.max(axis=0)
        Xmin, Ymin, Zmin = centerlinePoints.min(axis=0)

        # define the width for the extra points away from the weld seam
        dist2WideSeam = 4
        Cbool = (np.array(Xmax + dist2WideSeam > np.asarray(self.points.points)[:, 0], dtype=bool) & np.array(
            np.asarray(self.points.points)[:, 0] > Xmin - dist2WideSeam, dtype=bool))

        points = np.asarray(self.points.points)
        points = points[Cbool]
        self.points.points = o3d.utility.Vector3dVector(points)

        # colors = np.asarray(self.points.colors)
        # colors = colors[Cbool]
        # self.points.colors = o3d.utility.Vector3dVector(colors)
        if points.shape[0] < 500:
            return False
        else:

            Xmax, Ymax, Zmax = np.asarray(self.points.points).max(axis=0)
            Xmin, Ymin, Zmin = np.asarray(self.points.points).min(axis=0)
            #hight = np.average(np.asarray(self.points.points)[:, 2])
            height_1 = np.asarray(self.points.points)[1, 2]
            ratio_ymin =  Zmin / height_1
            hight = np.average(np.asarray(self.points.points)[:10, 2])

            inter_points_1 = np.array([])
            inter_points_2 = np.array([])
            if ratio_ymin > 0.8:
                inter_points_1 = np.array([[Xmin, Ymin, Zmin], [Xmin, Ymax, Zmax]])
                inter_points_2 = np.array([[Xmax, Ymin, Zmin], [Xmax, Ymax, Zmax]])
                self.laserpoints = np.array([[Xmin, Ymin, Zmin], [Xmax, Ymin, Zmin]])
            else:
                inter_points_1 = np.array([[Xmin, Ymin, Zmax], [Xmin, Ymax, Zmin]])
                inter_points_2 = np.array([[Xmax, Ymin, Zmax], [Xmax, Ymax, Zmin]])
                self.laserpoints = np.array([[Xmin, Ymin, Zmax], [Xmax, Ymin, Zmax]])



            for dx in np.arange(Ymin, Ymax, 0.03):
                i_x, i_z = self.interpolation3D(inter_points_1[0], inter_points_1[1], dx)
                self.laserpoints = np.append(self.laserpoints, [[i_x, dx, i_z]], axis=0)
                i_x, i_z = self.interpolation3D(inter_points_2[0], inter_points_2[1], dx)
                self.laserpoints = np.append(self.laserpoints, [[i_x, dx, i_z]], axis=0)

            self.laserpoints = np.append(self.laserpoints, [[Xmin, Ymax, Zmax]], axis=0)
            self.laserpoints = np.append(self.laserpoints, [[Xmax, Ymax, Zmax]], axis=0)
            # The points forming the corners of the rectangle

            random_name = o3d.geometry.PointCloud()
            random_name.points = o3d.utility.Vector3dVector(self.laserpoints)
            o3d.visualization.draw_geometries([self.points, random_name])

            return True


    def output_points(self):
        return self.laserpoints



