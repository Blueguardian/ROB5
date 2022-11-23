import open3d as o3d
import os
import numpy as np
import copy

class Proc3D:

    laserpoints = np.array([])

    def __init__(self, path):
        self.__PATH = path
        self.__OBJECT_TYPE = ""
        self.points = o3d.geometry.PointCloud()
        self.centerlinecloud = o3d.geometry.PointCloud()

    def load_points(self):
        """
        Load in points to the object from the .ply files provided in the folder
        get the intensity of the points and remove those above threshold.
        Lastly it will determine whether the object is a planar object or not, while
        it could determine the objects shape in general, this is not needed for
        this project
        ::__PATH:: Internal path to .ply file directory
        ::
        """


        #  Load in all points from the .ply file and register them in the
        #  provided instance fields after thresholding the points with
        #  regard to the intensity of their colors.
        intensity_threshold = 0.4
        if os.path.isfile(os.path.join(self.__PATH, 'pointcloud_0.ply')):
            source = o3d.io.read_point_cloud(os.path.join(self.__PATH, 'pointcloud_0.ply'))
            sourcepoints = np.asarray(source.points)
            temp_points = np.asarray(sourcepoints)
            temp_color = np.asarray(source.colors)
            intensity = np.asarray(source.colors)
            temp_cloud = np.asarray(source.points)[intensity[:, 0] <= intensity_threshold]
            Xmax, Ymax, Zmax = np.asarray(source.points).max(axis=0)
            Xmin, Ymin, Zmin = np.asarray(source.points).min(axis=0)


            for file in os.listdir(self.__PATH):
                source = o3d.io.read_point_cloud(os.path.join(self.__PATH, file))

                Xmax_temp, Ymax_temp, Zmax = np.asarray(source.points).max(axis=0)
                Xmin_temp, Ymin_temp, Zmin = np.asarray(source.points).min(axis=0)
                source.translate((0, (Ymax - Ymin), 0))
                Ymax = Ymax + (Ymax_temp - Ymin_temp)

                sourcepoints = np.asarray(source.points)
                temp_points = np.append(temp_points, sourcepoints, axis=0)
                intensity = np.asarray(source.colors)
                temp_cloud = np.append(temp_cloud, (np.asarray(source.points)[intensity[:, 0] <= intensity_threshold]), axis=0)
                temp_color = np.append(temp_color, source.colors, axis=0)
            self.points.points = o3d.utility.Vector3dVector(temp_points)
            self.points.colors = o3d.utility.Vector3dVector(temp_color)
            self.centerlinecloud = copy.deepcopy(self.points)
            self.centerlinecloud.points = o3d.utility.Vector3dVector(temp_cloud)

            # Determine if the object is a plane or not
            points, index = self.points.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
            if np.ma.size((np.asarray(self.points.points)), axis=0)/np.ma.size(np.asarray(self.points.points), axis=0) < np.ma.size(points, axis=0)/np.ma.size(np.asarray(self.points.points)):
                self.__OBJECT_TYPE = "plane"

            else:
                self.__OBJECT_TYPE = "other"
            return True
        else:
            print("No points available")
            return False




    def process_points(self):
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
        # colors = np.array([0, 0, 1], dtype=float)
        # colors = np.tile(colors, (centerlinePoints.shape[0], 1))
        # colorcheck = np.asarray(self.points.colors)
        # # source.points = o3d.utility.Vector3dVector(np.concatenate((np.asarray(source.points), centerlinePoints + 0.1)))

        # source.colors = o3d.utility.Vector3dVector(np.concatenate((np.asarray(source.colors), colors)))

        # expand area to contain the
        # first define the box containing the weld seam, find the max and min of x and y
        Xmax, Ymax, Zmax = centerlinePoints.max(axis=0)
        Xmin, Ymin, Zmin = centerlinePoints.min(axis=0)

        # define the width for the extra points away from the weld seam
        dist2WideSeam = 4
        Cbool = (np.array(Xmax + dist2WideSeam > np.asarray(self.points.points)[:, 0], dtype=bool) & np.array(
            np.asarray(self.points.points)[:, 0] > Xmin - dist2WideSeam, dtype=bool))

        o3d.visualization.draw_geometries([self.points])

        points = np.asarray(self.points.points)
        points = points[Cbool]
        self.points.points = o3d.utility.Vector3dVector(points)

        colors = np.asarray(self.points.colors)
        colors = colors[Cbool]
        self.points.colors = o3d.utility.Vector3dVector(colors)

        o3d.visualization.draw_geometries([self.points])

        Xmax, Ymax, Zmax = np.asarray(self.points.points).max(axis=0)
        Xmin, Ymin, Zmin = np.asarray(self.points.points).min(axis=0)
        hight = np.average(np.asarray(self.points.points)[:, 2])

        o3d.visualization.draw_geometries([self.points])

        self.laserpoints = np.array([[Xmin, Ymin, hight],
                                     [Xmin, Ymax, hight]])

        for dx in np.arange(Xmin, Xmax, 0.03):
            self.laserpoints = np.append(self.laserpoints, [[dx, Ymin, hight]], axis=0)
            self.laserpoints = np.append(self.laserpoints, [[dx, Ymax, hight]], axis=0)

        self.laserpoints = np.append(self.laserpoints, [[Xmax, Ymin, hight]], axis=0)
        self.laserpoints = np.append(self.laserpoints, [[Xmax, Ymax, hight]], axis=0)
        # The points forming the corners of the rectangle

        test_cloud = o3d.geometry.PointCloud()
        test_cloud.points = o3d.utility.Vector3dVector(self.laserpoints)
        test_cloud.paint_uniform_color([0, 0, 0.5])
        o3d.visualization.draw_geometries([self.points, test_cloud])


    def output_points(self):
        return self.laserpoints



