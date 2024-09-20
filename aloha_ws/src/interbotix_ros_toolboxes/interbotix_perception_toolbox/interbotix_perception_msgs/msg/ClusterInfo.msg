# This message is used specifically in the interbotix_perception_modules package
#
# Message that holds information about each cluster

string frame_id                     # parent frame of the cluster (usually 'camera_depth_optical_frame' or similar)
geometry_msgs/Point position        # x, y, z position of the cluster
float32 yaw                         # yaw [rad] of the cluster; x-axis of the cluster should align
                                    # with the major-axis of a best-fit ellipse (currently not supported)
std_msgs/ColorRGBA color            # average RGB values (0 - 255) for the whole cluster
geometry_msgs/Point min_z_point     # point with the min 'z' value of the cluster
int32 num_points                    # number of points in the cluster
