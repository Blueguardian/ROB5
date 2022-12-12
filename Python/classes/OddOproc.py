import copy
import math
import open3d as o3d
import numpy as np

class OddOProc:

    def __init__(self):
        # NB Scanner plot measurements is in mm
        # LSS is spot size of the laser
        self.LSS = 0.55

        # CREATE POINTS IN A SQUARE ON NECK OF TUBE

        #   x
        #   ^
        #   |
        #   |_____ > z (defined as z in the CAD file)

        # Placement of points

        #        EDGES[0]    ------    EDGES[1]
        #        |                            |
        #        EDGES[2]   -------    EDGES[3]

        # the edges are decided on from observing sampled CAD file.
        # y is chosen as an arbitrary value
        self.EDGES = [[59.7, 19, 22.0], [59.7, 19, 35.3], [50.19, 19, 22.0], [50.19, 19, 35.3]]

        # need to take into account the spot size when defining the edges of the desired area.
        self.zmin_send = self.EDGES[0][2] + self.LSS / 2
        self.zmax_send = self.EDGES[3][2] - self.LSS / 2


    # FUCTION DISPLAY 3D
    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                        zoom=0.4459,
                                        front=[0.9288, -0.2951, -0.2242],
                                        lookat=[1.6784, 2.0612, 1.4451],
                                        up=[-0.3402, -0.9189, -0.1996])

    # FUNCTION DOWNSAMPLING
    def preprocess_point_cloud(self, pcd, voxel_size):
        print(":: Downsample with a voxel size %.3f." % voxel_size)
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh


    # FUNCTION GLOBAL REGISTRATION
    def execute_global_registration(self, source_down, target_down, source_fpfh,
                                    target_fpfh, voxel_size):
        distance_threshold = voxel_size * 6.5
        print(":: RANSAC registration on downsampled point clouds.")
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                #o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                #    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        return result

    # FUNCTION LOCAL REGISTRATION
    def ICP_pp(self, source, target, trans):
        threshold = 7
        print(":: Point-to-point ICP registration is applied")
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
            # higher max iteration result in better alignment, but takes longer
            , o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        return reg_p2p

    # transformation from CAD to linescanner frame
    def trans_path(self, definedPoints, transformation):

        trans = np.asarray(transformation)
        # to be able to do the transformation, need to append 1 to each point
        for i in range(0, len(definedPoints)):
            definedPoints[i].append(1)

        a = np.asarray(definedPoints)

        transformatedPoints = []

        Xcoord_list = []
        Zcoord_list = []

        # goes through the list of points and transform each point one by one
        for coordinates in a:
            # point needs to be transposed before matrix multiplication
            coordinates.transpose()
            multiplication = np.matmul(trans, coordinates)
            # appends the transformed point to the new list over points without the previously appended 1.
            transformatedPoints.append([multiplication[0], multiplication[1], multiplication[2]])
            Xcoord_list.append(multiplication[0])
            Zcoord_list.append(multiplication[2])

        return Xcoord_list, Zcoord_list, transformatedPoints

    def square_path(self):

        # 0.55 * NUMSWIPES = (zmax-zmin)
        # number of times the laser need to swipe the area is rounded off to nearest integer,
        # this is the number of z values we need to calculate.

        NUMSWIPES = int((self.zmax_send-self.zmin_send) / self.LSS)
        ZLASERINDEKSER = [self.zmin_send]

        # calculate the z values of the points
        for i in range(0, int(NUMSWIPES)):
            ZLASERINDEKSER.append(ZLASERINDEKSER[i] + self.LSS)

        xmin_send = self.EDGES[3][0] + self.LSS/2
        xmax_send = self.EDGES[0][0] - self.LSS/2

        # make a list of the points
        punktliste_square = []

        for i in range(0, len(ZLASERINDEKSER)):

            punktliste_square.append([xmin_send, 30, ZLASERINDEKSER[i]])
            punktliste_square.append([xmax_send, 30, ZLASERINDEKSER[i]])

        return punktliste_square


    #### CREATE POINTS BASED ON CONNECTED CIRCLES ON TUBE

        #   x
        #   ^
        #   |
        #   |_____ > z

    def combined_circle_path(self, RXmax, RZmax):
    # RXmax = 10
    # RZmax = 15.0

    # the path goes over an area that is composed of two circles. Circle on the left has fixed radius, and at
    # some point on the right connects to a circle that has increasing radius.
    #                         RX
    #            (                                 )
    #        (                                                )
    #    (       EDGES[0]    midtpunkt   EDGES[1]      RZ               )
    #                |                      |
    #                |                      |
    #                |                      |
    #                |                      |
    #             EDGES[2]   -------    EDGES[3]

        # define the middle point of the circles
        midtpunkt = self.zmin_send + (self.zmax_send - self.zmin_send) / 2

        zmin_send_CIRC = midtpunkt - RXmax
        zmax_send_CIRC = midtpunkt + RZmax

        NUMSWIPES = int((zmax_send_CIRC - zmin_send_CIRC) / self.LSS)

        ZLASERINDEKSER = [zmin_send_CIRC]

        # calculate the z values of the points
        for i in range(0, int(NUMSWIPES)):
            ZLASERINDEKSER.append(ZLASERINDEKSER[i] + self.LSS)

        xmin_sen_circ = self.EDGES[0][0]

        XLASERINDEKSER = []

        flag = 1
        RNY = 0
        increment = 0

        for i in range(0, len(ZLASERINDEKSER)):
            if ZLASERINDEKSER[i] <= midtpunkt:
                #         xval
                #      (   | \ Rx
                #   (  zval|___\middle point

                # calculate x values using Pythagorean theorem, where the x value we wish to find is
                # the opposite side. Middle point - zval will give us the length of the adjacent side.
                # Rx is the length of the hypotenuse.

                XLASERINDEKSER.append(xmin_sen_circ + (RXmax ** 2 - (midtpunkt - ZLASERINDEKSER[i]) ** 2) ** (1 / 2))

            # when z value is bigger then the middle point the length of the adjacent side is calculated as
            # zval - middlepoint

            if ZLASERINDEKSER[i] > midtpunkt and ZLASERINDEKSER[i] < (midtpunkt + RXmax * (1 / 3)):
                XLASERINDEKSER.append(xmin_sen_circ + (RXmax ** 2 - ((ZLASERINDEKSER[i] - midtpunkt) ** 2)) ** (1 / 2))

            # when z value is bigger than ca middlevalue + 1/3 of RY radius want to switch method of caclulating x values
            # now the radius should increase for each calculated x value.

            if ZLASERINDEKSER[i] > (midtpunkt + RXmax * (1 / 3)):
                # The number of x values left to calculate decides how much the radius should increase
                # for each new x value to reach the max radius in z direction, RZmax.
                # This increment should be calculated only once, therefore a flag is used.
                if flag == 1:
                    iterations_left = len(ZLASERINDEKSER) - i
                    increment = (RZmax - RXmax) / iterations_left
                    flag = 0

                    RNY = RXmax

                XLASERINDEKSER.append(xmin_sen_circ + (RNY ** 2 - ((ZLASERINDEKSER[i] - midtpunkt) ** 2)) ** (1 / 2))
                RNY = RNY + increment
                # increment * iterations_left = (RZmax - RXmax)

        punktliste_circ = []

        for i in range(0, len(XLASERINDEKSER)):
            punktliste_circ.append([xmin_sen_circ, 30, ZLASERINDEKSER[i]])
            punktliste_circ.append([XLASERINDEKSER[i], 30, ZLASERINDEKSER[i]])

        return punktliste_circ

    def camera_circle(self, pcd):

        # Define parameters used for hidden_point_removal.
        diameter = np.linalg.norm(
            np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
        radius = diameter * 100

        # list for saving indexes from point removal
        maplist = []

        for i in range(1, 9):

            # camera follows a circle around the object
            ycamera = math.sin((math.pi / 180) * (45 * i)) * diameter
            zcamera = math.cos((math.pi / 180) * (45 * i)) * diameter

            # one picture from the -x and x direction

            camera = [0, ycamera, zcamera]

            if i == 8:
                camera = [diameter, 0, 0]

            # Get all points that are visible from given view point.
            _, pt_map = pcd.hidden_point_removal(camera, radius)

            maplist = maplist + pt_map

            pcd_surface_additions = pcd.select_by_index(maplist)

        return pcd_surface_additions