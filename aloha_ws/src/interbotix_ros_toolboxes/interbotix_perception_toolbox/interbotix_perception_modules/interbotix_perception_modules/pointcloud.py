# Copyright 2024 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from typing import List, Tuple, Union

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Quaternion, TransformStamped
from interbotix_common_modules import angle_manipulation as ang
from interbotix_common_modules.common_robot.robot import InterbotixRobotNode
from interbotix_common_modules.py_common import load_from_ros_params_file, save_to_ros_params_file
from interbotix_perception_msgs.msg import ClusterInfo
from interbotix_perception_msgs.srv import ClusterInfoArray, FilterParams
import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import SetBool
import tf2_ros
from tf_transformations import euler_from_quaternion


class InterbotixPointCloudInterface:
    """API to tune filter parameters to get position estimates of objects seen by the camera."""

    def __init__(self, filter_ns='pc_filter', node_inf: InterbotixRobotNode = None, args=None):
        """
        Construct pointcloud filter parameter tuner.

        :param filter_ns: namespace where the ROS parameters needed by the module are located
        :param node: reference to the InterbotixRobotNode on which to build this interface.
        :details: Note that the default reference frame for the pointcloud is used (usually
            something like 'camera_depth_optical_frame')
        """
        if node_inf is None:
            raise NotImplementedError('Passing node_inf as None is not implemented.')
        self.node_inf = node_inf

        self.filter_params = FilterParams.Request()
        self.node_inf.declare_parameter(
            name='filter_params',
            value=os.path.join(
                get_package_share_directory('interbotix_perception_modules'),
                'config',
                'filter_params.yaml'
            )
        )
        self.param_filepath = self.node_inf.get_parameter(
            'filter_params').get_parameter_value().string_value
        self.node_inf.get_logger().info(f"FilterParam filepath: '{self.param_filepath}'")

        self.srv_set_params = self.node_inf.create_client(
            FilterParams,
            f'/{filter_ns}/set_filter_params'
        )
        self.srv_enable_pipeline = self.node_inf.create_client(
            SetBool,
            f'/{filter_ns}/enable_pipeline'
        )
        self.srv_get_cluster_positions = self.node_inf.create_client(
            ClusterInfoArray,
            f'/{filter_ns}/get_cluster_positions'
        )
        while (
            not self.srv_set_params.wait_for_service(1.0)
            and not self.srv_enable_pipeline.wait_for_service(1.0)
            and not self.srv_get_cluster_positions.wait_for_service(1.0)
        ):
            self.node_inf.loginfo(
                f"Waiting for services '{self.srv_set_params.srv_name}', "
                f"'{self.srv_enable_pipeline.srv_name}', "
                f"and '{self.srv_get_cluster_positions.srv_name}' to "
                'come up.'
            )
        self.br = tf2_ros.TransformBroadcaster(self.node_inf)

        # declare and get params
        self.node_inf.declare_parameter('x_filter_min', -0.25)
        self.node_inf.declare_parameter('x_filter_max', 0.25)
        self.node_inf.declare_parameter('y_filter_min', -0.25)
        self.node_inf.declare_parameter('y_filter_max', 0.25)
        self.node_inf.declare_parameter('z_filter_min', 0.25)
        self.node_inf.declare_parameter('z_filter_max', 0.75)
        self.node_inf.declare_parameter('voxel_leaf_size', 0.004)
        self.node_inf.declare_parameter('plane_max_iter', 50)
        self.node_inf.declare_parameter('plane_dist_thresh', 0.005)
        self.node_inf.declare_parameter('ror_radius_search', 0.01)
        self.node_inf.declare_parameter('ror_min_neighbors', 5)
        self.node_inf.declare_parameter('cluster_tol', 0.02)
        self.node_inf.declare_parameter('cluster_min_size', 50)
        self.node_inf.declare_parameter('cluster_max_size', 1000)
        self.load_params_from_ros_params()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node_inf)

        self.node_inf.get_logger().info('Initialized InterbotixPointCloudInterface.')

    def get_params(self) -> dict:
        """
        Convert from a FilterParams Service type to a Python Dictionary.

        :return: python dictionary containing the data from the FilterParams service message
        """
        param_dict = {}
        param_dict['x_filter_min'] = float(self.filter_params.x_filter_min)
        param_dict['x_filter_max'] = float(self.filter_params.x_filter_max)
        param_dict['y_filter_min'] = float(self.filter_params.y_filter_min)
        param_dict['y_filter_max'] = float(self.filter_params.y_filter_max)
        param_dict['z_filter_min'] = float(self.filter_params.z_filter_min)
        param_dict['z_filter_max'] = float(self.filter_params.z_filter_max)
        param_dict['voxel_leaf_size'] = float(self.filter_params.voxel_leaf_size)
        param_dict['plane_max_iter'] = int(self.filter_params.plane_max_iter)
        param_dict['plane_dist_thresh'] = float(self.filter_params.plane_dist_thresh)
        param_dict['ror_radius_search'] = float(self.filter_params.ror_radius_search)
        param_dict['ror_min_neighbors'] = int(self.filter_params.ror_min_neighbors)
        param_dict['cluster_tol'] = float(self.filter_params.cluster_tol)
        param_dict['cluster_min_size'] = int(self.filter_params.cluster_min_size)
        param_dict['cluster_max_size'] = int(self.filter_params.cluster_max_size)
        return param_dict

    def set_params(self, param_dict: dict) -> None:
        """
        Convert from a Python dictionary to a FilterParams Service message.

        :param param_dict: Python Dictionary to convert to a FilterParams message
        """
        self.filter_params.x_filter_min = float(param_dict['x_filter_min'])
        self.filter_params.x_filter_max = float(param_dict['x_filter_max'])
        self.filter_params.y_filter_min = float(param_dict['y_filter_min'])
        self.filter_params.y_filter_max = float(param_dict['y_filter_max'])
        self.filter_params.z_filter_min = float(param_dict['z_filter_min'])
        self.filter_params.z_filter_max = float(param_dict['z_filter_max'])
        self.filter_params.voxel_leaf_size = float(param_dict['voxel_leaf_size'])
        self.filter_params.plane_max_iter = int(param_dict['plane_max_iter'])
        self.filter_params.plane_dist_thresh = float(param_dict['plane_dist_thresh'])
        self.filter_params.ror_radius_search = float(param_dict['ror_radius_search'])
        self.filter_params.ror_min_neighbors = int(param_dict['ror_min_neighbors'])
        self.filter_params.cluster_tol = float(param_dict['cluster_tol'])
        self.filter_params.cluster_min_size = int(param_dict['cluster_min_size'])
        self.filter_params.cluster_max_size = int(param_dict['cluster_max_size'])

    params = property(fget=get_params, fset=set_params)

    def get_x_filter_min(self) -> float:
        """
        Get the current minimum 'x' value.

        :return: x_filter_min [m]
        """
        return self.filter_params.x_filter_min

    def set_x_filter_min(self, x_filter_min: float):
        """
        Filter out any data point in a pointcloud with an 'x' value less than 'x_filter_min'.

        :param x_filter_min: desired minimum 'x' value [m]
        """
        self.filter_params.x_filter_min = x_filter_min
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    x_filter_min = property(fget=get_x_filter_min, fset=set_x_filter_min)

    def get_x_filter_max(self) -> float:
        """
        Get the current maximum 'x' value.

        :return: x_filter_max [m]
        """
        return self.filter_params.x_filter_max

    def set_x_filter_max(self, x_filter_max: float):
        """
        Filter out any data point in a pointcloud with an 'x' value more than 'x_filter_max'.

        :param x_filter_max: desired maximum 'x' value [m]
        """
        self.filter_params.x_filter_max = x_filter_max
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    x_filter_max = property(fget=get_x_filter_max, fset=set_x_filter_max)

    def get_y_filter_min(self) -> float:
        """
        Get the current minimum 'y' value.

        :return: y_filter_min [m]
        """
        return self.filter_params.y_filter_min

    def set_y_filter_min(self, y_filter_min: float):
        """
        Filter out any data point in a pointcloud with a 'y' value less than 'y_filter_min'.

        :param y_filter_min: desired minimum 'y' value [m]
        """
        self.filter_params.y_filter_min = y_filter_min
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    y_filter_min = property(fget=get_y_filter_min, fset=set_y_filter_min)

    def get_y_filter_max(self) -> float:
        """
        Get the current maximum 'y' value.

        :return: y_filter_max [m]
        """
        return self.filter_params.y_filter_max

    def set_y_filter_max(self, y_filter_max: float):
        """
        Filter out any data point in a pointcloud with a 'y' value more than 'y_filter_max'.

        :param y_filter_max: desired maximum 'y' value [m]
        """
        self.filter_params.y_filter_max = y_filter_max
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    y_filter_max = property(fget=get_y_filter_max, fset=set_y_filter_max)

    def get_z_filter_min(self) -> float:
        """
        Get the current minimum 'z' value.

        :return: z_filter_min [m]
        """
        return self.filter_params.z_filter_min

    def set_z_filter_min(self, z_filter_min: float):
        """
        Filter out any data point in a pointcloud with a 'z' value less than 'z_filter_min'.

        :param z_filter_min: desired minimum 'z' value [m]
        """
        self.filter_params.z_filter_min = z_filter_min
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    z_filter_min = property(fget=get_z_filter_min, fset=set_z_filter_min)

    def get_z_filter_max(self) -> float:
        """
        Get the current maximum 'z' value.

        :return: z_filter_max [m]
        """
        return self.filter_params.z_filter_max

    def set_z_filter_max(self, z_filter_max: float):
        """
        Filter out any data point in a pointcloud with a 'z' value more than 'z_filter_max'.

        :param z_filter_max: desired maximum 'z' value [m]
        """
        self.filter_params.z_filter_max = z_filter_max
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    z_filter_max = property(fget=get_z_filter_max, fset=set_z_filter_max)

    def get_voxel_leaf_size(self) -> float:
        """
        Get the current voxel size.

        :return: voxel_leaf_size [m]
        """
        return self.filter_params.voxel_leaf_size

    def set_voxel_leaf_size(self, voxel_leaf_size: float):
        """
        Set the voxel leaf size.

        :param voxel_leaf_size: desired voxel size [m] that applies to the x, y, and z dimensions
        """
        self.filter_params.voxel_leaf_size = voxel_leaf_size
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    voxel_leaf_size = property(fget=get_voxel_leaf_size, fset=set_voxel_leaf_size)

    def get_plane_max_iter(self) -> int:
        """
        Get the current max number of iterations for the planar segmentation algorithm.

        :return: plane_max_iter
        """
        return self.filter_params.plane_max_iter

    def set_plane_max_iter(self, plane_max_iter: int):
        """
        Set the maximum number of iterations the sample consensus method should run.

        :param plane_max_iter: desired max iterations (default is 50)
        """
        self.filter_params.plane_max_iter = int(plane_max_iter)
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    plane_max_iter = property(fget=get_plane_max_iter, fset=set_plane_max_iter)

    def get_plane_dist_thresh(self) -> float:
        """
        Get the current max distance from the plane model to be considered part of the plane.

        :return: plane_dist_thresh [m]
        """
        return self.filter_params.plane_dist_thresh

    def set_plane_dist_thresh(self, plane_dist_thresh: float):
        """
        Set the plane distance threshold.

        :param plane_dist_thresh: desired max distance [m] (default is about 0.01 meters)
        """
        self.filter_params.plane_dist_thresh = plane_dist_thresh
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    plane_dist_thresh = property(fget=get_plane_dist_thresh, fset=set_plane_dist_thresh)

    def get_ror_radius_search(self) -> float:
        """
        Get the current radius used when searching for nearest neighbor points.

        :return: ror_radius_search [m]
        """
        return self.filter_params.ror_radius_search

    def set_ror_radius_search(self, ror_radius_search: float):
        """
        Set the radius around any given point to search for neighbors.

        :param ror_radius_search: desired radius [m] (default is about 0.01 meters)
        """
        self.filter_params.ror_radius_search = ror_radius_search
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    ror_radius_search = property(fget=get_ror_radius_search, fset=set_ror_radius_search)

    def get_ror_min_neighbors(self) -> int:
        """
        Get the current minimum number of neighbors to search for for a point not to be removed.

        :return: ror_min_neighbors
        """
        return self.filter_params.ror_min_neighbors

    def set_ror_min_neighbors(self, ror_min_neighbors: int):
        """
        Set radius of removal minimum neighbors.

        :param ror_min_neighbors: desired min neighbors (default is 5)
        """
        self.filter_params.ror_min_neighbors = int(ror_min_neighbors)
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    ror_min_neighbors = property(fget=get_ror_min_neighbors, fset=set_ror_min_neighbors)

    def get_cluster_tol(self) -> float:
        """
        Get the current cluster tolerance.

        :return: cluster_tol [m]
        """
        return self.filter_params.cluster_tol

    def set_cluster_tol(self, cluster_tol: float):
        """
        Set the cluster tolerance.

        :param cluster_tol: desired cluster tolerance [m] (default is about 0.02 meters)
        """
        self.filter_params.cluster_tol = cluster_tol
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    cluster_tol = property(fget=get_cluster_tol, fset=set_cluster_tol)

    def get_cluster_min_size(self) -> int:
        """
        Get the current minimum cluster size.

        :return: cluster_min_size [number of points in the pointcloud cluster]
        """
        return self.filter_params.cluster_min_size

    def set_cluster_min_size(self, cluster_min_size: int):
        """
        Set the minimum size that a cluster must be to be considered a cluster.

        :param cluster_min_size: desired minimum size [number of points in a cluster's pointcloud]
        """
        self.filter_params.cluster_min_size = int(cluster_min_size)
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    cluster_min_size = property(fget=get_cluster_min_size, fset=set_cluster_min_size)

    def get_cluster_max_size(self) -> int:
        """
        Get the current maximum cluster size.

        :return: cluster_max_size [number of points in the pointcloud cluster]
        """
        return self.filter_params.cluster_max_size

    def set_cluster_max_size(self, cluster_max_size: int):
        """
        Set the maximum size that a cluster can be to be considered a cluster.

        :param cluster_max_size: desired maximum size [number of points in a cluster's pointcloud]
        """
        self.filter_params.cluster_max_size = int(cluster_max_size)
        future = self.srv_set_params.call_async(self.filter_params)
        self.node_inf.wait_until_future_complete(future)

    cluster_max_size = property(fget=get_cluster_max_size, fset=set_cluster_max_size)

    def get_filepath(self) -> str:
        """
        Get the absolute filepath to the config file containing all the filter parameters.

        :return: param_filepath - string with the absolute filepath
        """
        return self.param_filepath

    def enable_pipeline(self, enable: bool):
        """
        Enable/Disable the pointcloud filtering process.

        :param: enable - boolean that if `True`, enables the pipeline, and if `False`, disables the
            pipeline
        """
        future = self.srv_enable_pipeline.call_async(enable)
        self.node_inf.wait_until_future_complete(future)

    def get_cluster_positions(
        self,
        num_samples: int = 5,
        period: float = 0.1,
        ref_frame: str = None,
        sort_axis: str = 'y',
        reverse: bool = False,
        is_parallel: bool = True
    ) -> Tuple[bool, Union[None, List[dict]]]:
        """
        Get the estimated positions of all pointcloud clusters.

        :param num_samples: number of times to run the pipeline to get the cluster positions; these
            samples are then averaged together to get a more accurate result
        :param period: number of seconds to wait between sampling (give time for the backend to get
            updated with a new pointcloud)
        :param ref_frame: the desired reference frame the cluster positions should be transformed
            into; if unspecified, the camera's depth frame is used
        :param sort_axis: the axis of the 'ref_frame' by which to sort the cluster positions when
            returning them to the user
        :param reverse: if `False`, cluster positions are sorted in ascending order along the axis
            specified by 'sort_axis'; if `True`, the positions are sorted in descending order
        :param is_parallel: if `False`, the cluster positions returned to the user represent the
            centroids of each cluster w.r.t. the 'ref_frame'; if `True`, the cluster positions
            returned to the user represent the centroids of each cluster, but positioned at the top
            of each cluster w.r.t. the 'ref_frame'; set this to `True` if the 'ref_frame' is
            parallel to the surface that the objects are on
        :return: `True` if the algorithm succeeded or `False` otherwise. If `False`, the
            'final_clusters' list is empty, but if `True`, a list of dictionaries representing each
            cluster is returned to the user
        """
        future_get_cluster_pos = self.srv_get_cluster_positions.call_async(
            ClusterInfoArray.Request()
        )
        self.node_inf.wait_until_future_complete(future_get_cluster_pos)
        root_clusters: List[ClusterInfo] = future_get_cluster_pos.result().clusters
        num_clusters = len(root_clusters)
        if num_clusters == 0:
            self.node_inf.get_logger().warning('No clusters found...')
            return False, []
        cluster_frame: str = root_clusters[0].frame_id

        # Calculate the average for each cluster based on 'num_samples' samples
        avg_clusters: List[ClusterInfo] = []
        for i in range(num_clusters):
            avg_clusters.append(ClusterInfo())
        for _ in range(num_samples):
            clusters: List[ClusterInfo] = self.srv_get_cluster_positions.call_async(
                ClusterInfoArray.Request()
            )
            self.node_inf.wait_until_future_complete(future_get_cluster_pos)
            clusters: List[ClusterInfo] = future_get_cluster_pos.result().clusters
            if len(clusters) != num_clusters:
                self.get_logger().warning(
                    f'Found {len(clusters)} clusters instead of {num_clusters} clusters...'
                )
                return False, []
            valid_indices = list(range(num_clusters))
            for cluster in clusters:
                for indx in valid_indices:
                    if (
                        (
                            abs(
                                root_clusters[indx].position.x - cluster.position.x
                            ) < self.filter_params.cluster_tol) and
                        (
                            abs(
                                root_clusters[indx].position.y - cluster.position.y
                            ) < self.filter_params.cluster_tol) and
                        (
                            abs(
                                root_clusters[indx].position.z - cluster.position.z
                            ) < self.filter_params.cluster_tol)
                    ):
                        avg_clusters[indx].position.x += cluster.position.x / num_samples
                        avg_clusters[indx].position.y += cluster.position.y / num_samples
                        avg_clusters[indx].position.z += cluster.position.z / num_samples
                        avg_clusters[indx].color.r += cluster.color.r / num_samples
                        avg_clusters[indx].color.g += cluster.color.g / num_samples
                        avg_clusters[indx].color.b += cluster.color.b / num_samples
                        avg_clusters[indx].min_z_point.x += cluster.min_z_point.x / num_samples
                        avg_clusters[indx].min_z_point.y += cluster.min_z_point.y / num_samples
                        avg_clusters[indx].min_z_point.z += cluster.min_z_point.z / num_samples
                        avg_clusters[indx].num_points += int(cluster.num_points / num_samples)
                        valid_indices.remove(indx)
                        break
                    elif (indx == num_clusters - 1):
                        self.node_inf.logwarn((
                            'Could not match the cluster. Please tune the filter parameters such '
                            "that all spherical 'object markers' are constant in their respective "
                            'clusters and do not flicker.'
                        ))
                        return False, []
            self.node_inf.get_clock().sleep_for(Duration(seconds=period))

        # Get the transform from the 'ref_frame' to the cluster frame (i.e. the camera's depth
        # frame) - known as T_rc
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame=ref_frame,
                source_frame=cluster_frame,
                time=Time(),
                timeout=Duration(seconds=4.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            self.node_inf.logerror(
                f"Failed to look up the transform from '{ref_frame}' to '{cluster_frame}'."
            )
            return False, []
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        T_rc = ang.pose_to_transformation_matrix([x, y, z, rpy[0], rpy[1], rpy[2]])

        # Sort the clusters from left to right w.r.t. the camera's depth frame
        sorted_cam_clusters = sorted(avg_clusters, key=lambda cluster: cluster.position.x)

        # TODO
        # if is_parallel is True... find 'yaw' of each cluster relative to a virtual frame located
        # at 'cluster_frame' (with the 'x' axis pointed in the same direction, but with the 'z'
        # axis pointing straight up) using OpenCV (order is left to right) set this 'yaw' to the
        # yaw variable in each cluster; it will be added to the yaw part of T_rc before the
        # transform is published

        # Transform the clusters to be w.r.t. the 'ref_frame' instead of the camera's depth frame
        for cluster in sorted_cam_clusters:
            # p_co is the cluster's position w.r.t. the camera's depth frame
            # p_ro is the cluster's position w.r.t. the desired reference frame
            p_co = [cluster.position.x, cluster.position.y, cluster.position.z, 1]
            p_ro = np.dot(T_rc, p_co)
            if (is_parallel):
                # p_comin is the minimum point of the cluster (in the 'z' direction) w.r.t. the
                # camera's depth frame; it is assumed that this point lies at the top or very near
                # the top of the cluster p_romin is the same point w.r.t. the desired reference
                # frame; the 'z' element of this point replaces the 'z' element in p_ro; thus, a tf
                # frame published at this point should appear at the 'top-center' of the cluster
                p_comin = [cluster.min_z_point.x, cluster.min_z_point.y, cluster.min_z_point.z, 1]
                p_romin = np.dot(T_rc, p_comin)
                p_ro[2] = p_romin[2]
            cluster.position.x = p_ro[0]
            cluster.position.y = p_ro[1]
            cluster.position.z = p_ro[2]

        # Sort the clusters based on user input
        if sort_axis == 'x':
            def key(cluster: ClusterInfo): return cluster.position.x
        elif sort_axis == 'y':
            def key(cluster: ClusterInfo): return cluster.position.y
        elif sort_axis == 'z':
            def key(cluster: ClusterInfo): return cluster.position.z
        else:
            def key(cluster: ClusterInfo): return cluster.position.y
            self.node_inf.logwarn((
                f"'{sort_axis}' is not a valid sorting axis. Set the 'sort_axis' "
                "argument to 'x', 'y', or 'z'. Defaulting to 'y'."
            ))
        sorted_ref_clusters = sorted(sorted_cam_clusters, key=key, reverse=reverse)

        # publish transforms to the /tf tree for debugging purposes (only once)
        final_trans: List[TransformStamped] = []
        cluster_num = 1
        time_now = self.node_inf.get_clock().now().to_msg()
        for cluster in sorted_ref_clusters:
            trans = TransformStamped()
            trans.header.frame_id = ref_frame
            trans.header.stamp = time_now
            trans.child_frame_id = f'cluster_{str(cluster_num)}'
            trans.transform.translation.x = cluster.position.x
            trans.transform.translation.y = cluster.position.y
            trans.transform.translation.z = cluster.position.z
            trans.transform.rotation = Quaternion(x=0., y=0., z=0., w=1.)
            final_trans.append(trans)
            cluster_num += 1
        self.br.sendTransform(final_trans)

        # create a list of Python dictionaries to return to the user
        final_clusters = []
        for indx in range(num_clusters):
            name = final_trans[indx].child_frame_id
            x = final_trans[indx].transform.translation.x
            y = final_trans[indx].transform.translation.y
            z = final_trans[indx].transform.translation.z
            yaw = 0
            r = sorted_ref_clusters[indx].color.r
            g = sorted_ref_clusters[indx].color.g
            b = sorted_ref_clusters[indx].color.b
            num_points = sorted_ref_clusters[indx].num_points
            cluster = {
                'name': name,
                'position': [x, y, z],
                'yaw': yaw,
                'color': [r, g, b],
                'num_points': num_points
            }
            final_clusters.append(cluster)
        return True, final_clusters

    def load_params(self, filepath: str = None) -> None:
        """
        Get params from the specified filepath.

        :param filepath: YAML config file to get the params; if `None`, the default
            filepath specified by the 'filter_params' ROS parameter is used
        """
        if filepath is None:
            filepath = self.get_filepath()
        self.set_params(load_from_ros_params_file(filepath))

    def load_params_from_ros_params(self) -> None:
        """Load filter parameters from the given ROS params."""
        self.filter_params.x_filter_min = self.node_inf.get_parameter(
            'x_filter_min').get_parameter_value().double_value
        self.filter_params.x_filter_max = self.node_inf.get_parameter(
            'x_filter_max').get_parameter_value().double_value
        self.filter_params.y_filter_min = self.node_inf.get_parameter(
            'y_filter_min').get_parameter_value().double_value
        self.filter_params.y_filter_max = self.node_inf.get_parameter(
            'y_filter_max').get_parameter_value().double_value
        self.filter_params.z_filter_min = self.node_inf.get_parameter(
            'z_filter_min').get_parameter_value().double_value
        self.filter_params.z_filter_max = self.node_inf.get_parameter(
            'z_filter_max').get_parameter_value().double_value
        self.filter_params.voxel_leaf_size = self.node_inf.get_parameter(
            'voxel_leaf_size').get_parameter_value().double_value
        self.filter_params.plane_max_iter = self.node_inf.get_parameter(
            'plane_max_iter').get_parameter_value().integer_value
        self.filter_params.plane_dist_thresh = self.node_inf.get_parameter(
            'plane_dist_thresh').get_parameter_value().double_value
        self.filter_params.ror_radius_search = self.node_inf.get_parameter(
            'ror_radius_search').get_parameter_value().double_value
        self.filter_params.ror_min_neighbors = int(self.node_inf.get_parameter(
            'ror_min_neighbors').get_parameter_value().integer_value)
        self.filter_params.cluster_tol = self.node_inf.get_parameter(
            'cluster_tol').get_parameter_value().double_value
        self.filter_params.cluster_min_size = self.node_inf.get_parameter(
            'cluster_min_size').get_parameter_value().integer_value
        self.filter_params.cluster_max_size = self.node_inf.get_parameter(
            'cluster_max_size').get_parameter_value().integer_value

    def save_params(self, filepath: str = None) -> None:
        """
        Save params to the specified filepath.

        :param filepath: YAML config file to save the params; if `None`, the default
            filepath specified by the 'filter_params' ROS parameter is used
        """
        if filepath is None:
            filepath = self.param_filepath
        save_to_ros_params_file(self.get_params(), filepath)

    # def spin_until_future_complete(self, future: rclpy.Future) -> None:
    #     """
    #     Spin this node until the given future is complete.

    #     :param future: The future to wait for
    #     """
    #     rclpy.spin_until_future_complete(self.node_inf, future=future, timeout_sec=0.1)
