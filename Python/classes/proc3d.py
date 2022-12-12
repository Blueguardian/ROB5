import open3d as o3d
import os
import numpy as np
import copy
from classes.OddOproc import OddOProc

class Proc3D:

    laserpoints = np.array([])

    def __init__(self, path):
        self.ObjHandle = OddOProc()
        self.__PATH = path
        self.object_type = ""
        self.points = o3d.geometry.PointCloud()
        self.centerlinecloud = o3d.geometry.PointCloud()

        self.intensity_threshold = 0.4 # [%] - Lower = Darker
        self.bearing_offset = 20.0 # [mm]
        self.plane_percentage = 0.7 # [%]
        self.dist2WideSeam = 4 # [mm]
        self.PointsRemainder = 100 # number of remaining points (threshold for acceptance/need for cleaning)
        self.step_size = 0.55 # [mm] Length of steps between points

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

    def numpylist_sliced_x_value(self, cloud, x_value):

        cloud = copy.deepcopy(cloud)
        indices = []

        for i in range(0, len(cloud)):

            if x_value > 0:
                if cloud[i, 0] > x_value:
                    indices.append(i)

            if x_value < 0:
                if cloud[i, 0] < x_value:
                    indices.append(i)

        sliced_cloud = np.delete(cloud, indices, axis=0)

        return sliced_cloud

    def load_points(self):
        """
        Load in points to the object from the .ply files provided in the folder
        get the intensity of the points and remove those above threshold.
        Lastly it will determine whether the object is a planar object or not, while
        it could determine the objects shape in general, this is not needed for
        this project
        ::__PATH:: Internal path to .ply file directory
        """

        if os.path.isfile(os.path.join(self.__PATH, 'pointcloud_0.ply')):
            source = o3d.io.read_point_cloud(os.path.join(self.__PATH, 'pointcloud_0.ply'))
            intensity = np.asarray(source.colors)

            # slice the fixture part
            # sliced_points = self.numpylist_sliced_x_value(temp_cloud, self.bearing_offset)
            sliced_points = np.asarray(source.points)[np.asarray(source.points)[:, 0] < self.bearing_offset]
            sliced_colors = np.asarray(source.colors)[np.asarray(source.points)[:, 0] < self.bearing_offset]

            ThreshedPoints = sliced_points[sliced_colors[:, 0] <= self.intensity_threshold]
            Threshedcolors = sliced_colors[sliced_colors[:, 0] <= self.intensity_threshold]

            self.points.points = o3d.utility.Vector3dVector(ThreshedPoints)
            self.points.colors = o3d.utility.Vector3dVector(Threshedcolors)
            # remove points with high intensity

            self.centerlinecloud = copy.deepcopy(self.points)

            o3d.visualization.draw_geometries([self.points])

            # Determine if the object is a plane or not
            points, indices = self.centerlinecloud.segment_plane(distance_threshold=3, ransac_n=3, num_iterations=100000)
            if sliced_points.shape[0]*self.plane_percentage < np.ma.size(indices, axis=0):
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
        :return: True if there are still points with low intensity, i.e. require cleaning
        """
        if self.object_type == 'plane':
            self.laserpoints = np.array([])
            # euclidean clustering
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

            # concatenate biggest cluster into the original pointcloud
            centerlinePoints = centerlinePoints[labels == biggestLabel]
            self.points.points = o3d.utility.Vector3dVector(centerlinePoints)

            # first define the box containing the weld seam, find the max and min of x and y
            Xmax, Ymax, Zmax = centerlinePoints.max(axis=0)
            Xmin, Ymin, Zmin = centerlinePoints.min(axis=0)

            # define the width for the extra points away from the weld seam
            Cbool = (np.array(Xmax + self.dist2WideSeam > np.asarray(self.points.points)[:, 0], dtype=bool) & np.array(
                np.asarray(self.points.points)[:, 0] > Xmin - self.dist2WideSeam, dtype=bool))

            points = np.asarray(self.points.points)
            points = points[Cbool]
            self.points.points = o3d.utility.Vector3dVector(points)

            if points.shape[0] < self.PointsRemainder:
                return False
            else:

                Xmax, Ymax, Zmax = np.asarray(self.points.points).max(axis=0)
                Xmin, Ymin, Zmin = np.asarray(self.points.points).min(axis=0)
                height_ymax = np.asarray(self.points.points)[np.argmax(np.asarray(self.points.points)[0, 1])][2]
                ratio_ymin =  Zmin / height_ymax

                inter_points_1 = np.array([])
                inter_points_2 = np.array([])
                if ratio_ymin < 0.8:
                    inter_points_1 = np.array([[Xmin, Ymin, Zmax], [Xmin, Ymax, Zmin]])
                    inter_points_2 = np.array([[Xmax, Ymin, Zmax], [Xmax, Ymax, Zmin]])
                    self.laserpoints = np.array([[Xmin, Ymin, Zmin], [Xmax, Ymin, Zmin]])
                else:
                    inter_points_1 = np.array([[Xmin, Ymin, Zmin], [Xmin, Ymax, Zmax]])
                    inter_points_2 = np.array([[Xmax, Ymin, Zmin], [Xmax, Ymax, Zmax]])
                    self.laserpoints = np.array([[Xmin, Ymin, Zmax], [Xmax, Ymin, Zmax]])

                for dx in np.arange(Ymin, Ymax, self.step_size):
                    i_x, i_z = self.interpolation3D(inter_points_1[0], inter_points_1[1], dx)
                    self.laserpoints = np.append(self.laserpoints, [[i_x, dx, i_z]], axis=0)
                    i_x, i_z = self.interpolation3D(inter_points_2[0], inter_points_2[1], dx)
                    self.laserpoints = np.append(self.laserpoints, [[i_x, dx, i_z]], axis=0)

                # The points forming the corners of the rectangle
                if ratio_ymin < 0.8:
                    self.laserpoints = np.append(self.laserpoints, [[Xmin, Ymin, Zmin], [Xmax, Ymin, Zmin]], axis=0)
                else:
                    self.laserpoints = np.append(self.laserpoints, [[Xmin, Ymax, Zmax], [Xmax, Ymax, Zmax]], axis=0)

                edge_points = o3d.geometry.PointCloud()
                edge_points.points = o3d.utility.Vector3dVector(self.laserpoints)
                o3d.visualization.draw_geometries([self.points, edge_points])

                return True
        else:
            # load in source file and remove the bearing from the point cloud
            source = o3d.io.read_point_cloud("Give_Pointcloud_0025_Neutral.ply")
            source.points = o3d.utility.Vector3dVector(self.numpylist_sliced_x_value(np.asarray(source.points), 30))

            # load in CAD file as target
            CAD_mesh = o3d.io.read_triangle_mesh("NEW.ply")
            # sample same amount of points from mesh as the source cloud contains
            target = CAD_mesh.sample_points_uniformly(number_of_points=len(source.points))
            # are able to remove the information about the inner parts of the tube from the point cloud
            target = self.ObjHandle.camera_circle(target)

            # downsample based on the size of bounding box(voxel) - points laying inside this bounding box is combined
            # into a single point. Downsample to reduce the amount of processing power needed in further operations.
            voxel_size = 1
            source_down, source_fpfh = self.ObjHandle.preprocess_point_cloud(source, voxel_size)
            target_down, target_fpfh = self.ObjHandle.preprocess_point_cloud(target, voxel_size)

            # PERFORMING GLOBAL REGISTRATION AS INITIALIZATION
            result_ransac = self.ObjHandle.execute_global_registration(source_down, target_down,
                                                           source_fpfh, target_fpfh,
                                                           voxel_size)

            # PERFORMING LOCAL REGISTRATION -> Local registration rely on the rough alignment from global registration
            # as initialization
            reg_p2p = self.ObjHandle.ICP_pp(source_down, target_down, result_ransac.transformation)

            # draw alignment
            self.ObjHandle.draw_registration_result(source, target, reg_p2p.transformation)

            # invert the transformation to get the transformation needed to the linescanner frame
            trans_to_scanner = np.linalg.inv(reg_p2p.transformation)

            # create points on square path on neck of tube
            square_points = self.ObjHandle.square_path()

            # transform the points into the scanner frame
            square_xcoord, square_ycoord, square_points_trans = self.ObjHandle.trans_path(square_points, trans_to_scanner)

            # define desired radius of circular path on tube
            RX = 10
            RY = 15

            # create points on circular path
            circle_points = self.ObjHandle.combined_circle_path(RX, RY)

            # transform the points into the linescanner frame
            circ_xcoord, circ_ycoord, cicle_points_trans = self.ObjHandle.trans_path(circle_points, trans_to_scanner)

            # combine the coordinates from each path into two lists, this is the desired structure to send to the galvoscanner
            xcoord = square_xcoord + circ_xcoord
            ycoord = square_ycoord + circ_ycoord

            self.laserpoints = np.asarray([[xcoord[0], ycoord[0]]], axis=0)
            for i in range(1, len(xcoord)-1):
                self.laserpoints = np.append(self.laserpoints, [[xcoord[i], ycoord[i]]], axis=0)


            # combine points from both paths for visualizing
            points = square_points_trans + cicle_points_trans
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            # pick colour so that easy recognizable in next visualization step
            cloud.paint_uniform_color([1, 0.706, 0])

            # visualize scan and paths created
            o3d.visualization.draw_geometries([source, cloud])

            return True


    def output_points(self):
        return self.laserpoints



