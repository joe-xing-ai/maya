from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import subprocess
import shlex
from PIL import Image
import io
from nanomsg import Socket, PUB, SUB_SUBSCRIBE, SUB, REQ, PAIR
from datetime import datetime
import numpy as np
import time

import tensorflow as tf

FLAGS = tf.app.flags.FLAGS

try:
    from seer.message_pb2 import Message, Point3D, BoundingBox, RoadSegment, LaneMarker, LaneMarkers, BoundingBox2D,\
        RoadDivider, RoadBound, RoadBounds, TrafficLightStopPoints, TrafficLightStopPoint, TrafficLights, TrafficLight,\
        LidarPointCloud, HDMap  # NOQA
except ImportError:
    from message_pb2 import Message, Point3D, BoundingBox, RoadSegment, LaneMarker, LaneMarkers, BoundingBox2D,\
        RoadDivider, RoadBound, RoadBounds, TrafficLightStopPoints, TrafficLightStopPoint, TrafficLights, TrafficLight,\
        LidarPointCloud, HDMap  # NOQA

DELIMITER = ':'


def wake_up_maya(start_position=None, constraints_for_training=None):
    # start the maya simulator process
    maya_command_line = "%s" % FLAGS.maya_path

    if constraints_for_training is None:
        maya_sim_args = FLAGS.constraints_for_training
    else:
        maya_sim_args = constraints_for_training

    maya_sim_args = maya_sim_args.split(";")

    for args in range(len(maya_sim_args)):
        maya_command_line += " --constraint %s" % maya_sim_args[args]

    maya_command_line += " --road_width %s" % FLAGS.road_width
    maya_command_line += " --road_length %s" % FLAGS.road_length

    num_lanes = 3

    maya_command_line += " --num_lane_right %s" % num_lanes
    maya_command_line += " --num_lane_left %s" % num_lanes

    maya_command_line += " --log_port %s" % FLAGS.log_port
    maya_command_line += " --log_port_image %s" % FLAGS.log_port_image
    maya_command_line += " --log_port_lidar %s" % FLAGS.log_port_lidar
    maya_command_line += " --log_port_hd_map %s" % FLAGS.log_port_hd_map

    maya_command_line += " --control_port %s" % FLAGS.control_port
    maya_command_line += " --prediction_port %s" % FLAGS.prediction_port

    if FLAGS.dump_png:
        maya_command_line += " --dump_png"
        maya_command_line += " --dump_png_frequency %s" % FLAGS.dump_png_frequency

    if FLAGS.dump_lidar:
        maya_command_line += " --dump_lidar"
        maya_command_line += " --dump_lidar_frequency %s" % FLAGS.dump_lidar_frequency

    if FLAGS.dump_hd_map:
        maya_command_line += " --dump_hd_map"
        maya_command_line += " --dump_hd_map_frequency %s" % FLAGS.dump_hd_map_frequency

    if FLAGS.bbox is False:
        maya_command_line += " --no_bbox"

    if FLAGS.vehicle_name is False:
        maya_command_line += " --no_vehicle_name"

    if start_position is None:
        maya_command_line += " --start_position %s" % FLAGS.start_position
    else:
        maya_command_line += " --start_position %s" % start_position

    if FLAGS.skeleton is True:
        maya_command_line += " --skeleton"

    maya_command_line += " --log_frequency %s" % FLAGS.log_frequency

    maya_command_line += " --log_lidar_frequency %s" % FLAGS.log_lidar_frequency

    maya_command_line += " --intent_render_format %s" % FLAGS.intent_render_format

    maya_command_line += " --camera_distance %s" % FLAGS.camera_distance
    maya_command_line += " --camera_height %s" % FLAGS.camera_height
    maya_command_line += " --camera_look_at_height %s" % FLAGS.camera_look_at_height

    if FLAGS.batch_mode:
        maya_command_line += " -batchmode"
        maya_command_line += " -nographics"

    tf.logging.info("waking maya now: %s !!!!" % maya_command_line)
    args = shlex.split(maya_command_line)
    proc = subprocess.Popen(args)  # Success!
    return proc


def maya_to_sleep(proc):
    # now shut down maya simulator
    tf.logging.info("shutting down the maya simulator with process id %s now ......" % proc.pid)
    proc.terminate()


def load_point_3d(array):
    """
    :param array: either 3x1, or 1x3, or 3, shape of array, as long as its a triplet
    :return:
    """
    array = array.flatten()
    pts = Point3D()
    if len(array) == 3:
        pts.x, pts.y, pts.z = \
          array[0], array[1], array[2]
    else:
        pts.x, pts.y, pts.z = \
            array[0], array[1], 0.0
    return pts


def load_bound_box(center_array, eight_corners_array):
    my_bb = BoundingBox()
    center = load_point_3d(center_array)
    bottom_left_rear = load_point_3d(eight_corners_array[0, :])  #
    bottom_right_rear = load_point_3d(eight_corners_array[1, :])
    bottom_right_front = load_point_3d(eight_corners_array[2, :])
    bottom_left_front = load_point_3d(eight_corners_array[3, :])  #
    top_left_rear = load_point_3d(eight_corners_array[4, :])
    top_right_rear = load_point_3d(eight_corners_array[5, :])
    top_right_front = load_point_3d(eight_corners_array[6, :])
    top_left_front = load_point_3d(eight_corners_array[7, :])
    my_bb.center.CopyFrom(center)
    my_bb.bottom_left_rear.CopyFrom(bottom_left_rear)
    my_bb.bottom_right_rear.CopyFrom(bottom_right_rear)
    my_bb.bottom_right_front.CopyFrom(bottom_right_front)
    my_bb.bottom_left_front.CopyFrom(bottom_left_front)
    my_bb.top_left_rear.CopyFrom(top_left_rear)
    my_bb.top_right_rear.CopyFrom(top_right_rear)
    my_bb.top_right_front.CopyFrom(top_right_front)
    my_bb.top_left_front.CopyFrom(top_left_front)

    return my_bb


def load_road_segment(name, road_coordinates):
    """
    :param road_coordinates: 4x3 array
    :return: road segment that has 4 corners consisting a polygon
    """
    my_road_seg = RoadSegment()
    my_road_seg.name = name
    my_road_seg.left_rear.CopyFrom(load_point_3d(road_coordinates[0, :]))
    my_road_seg.right_rear.CopyFrom(load_point_3d(road_coordinates[1, :]))
    my_road_seg.right_front.CopyFrom(load_point_3d(road_coordinates[2, :]))
    my_road_seg.left_front.CopyFrom(load_point_3d(road_coordinates[3, :]))
    return my_road_seg


def load_road_divider(road_coordinates):
    my_road_divider = RoadDivider()
    my_road_divider.left_rear.CopyFrom(load_point_3d(road_coordinates[0, :]))
    my_road_divider.right_rear.CopyFrom(load_point_3d(road_coordinates[1, :]))
    my_road_divider.right_front.CopyFrom(load_point_3d(road_coordinates[2, :]))
    my_road_divider.left_front.CopyFrom(load_point_3d(road_coordinates[3, :]))
    return my_road_divider


def load_road_bounds(bounds_array):
    bounds_obj = RoadBounds()
    num_bounds = int(bounds_array.shape[0] / 4)
    for itr in range(num_bounds):
        road_bound = RoadBound()

        road_bound.left_rear.CopyFrom(load_point_3d(bounds_array[itr * 4, :]))
        road_bound.right_rear.CopyFrom(load_point_3d(bounds_array[itr * 4 + 1, :]))
        road_bound.right_front.CopyFrom(load_point_3d(bounds_array[itr * 4 + 2, :]))
        road_bound.left_front.CopyFrom(load_point_3d(bounds_array[itr * 4 + 3, :]))

        bounds_obj.list_of_road_bounds.extend([road_bound])
    return bounds_obj


def load_lane_markers(lane_markers_array):
    """
    :param lane_markers_array: 4x3 array, 4 lane markers, each lane has a center
    :return:  lane markers
    """
    lane_markers_obj = LaneMarkers()
    num_lane_marker = int(lane_markers_array.shape[0] / 4)
    for itr in range(num_lane_marker):
        lane_marker = LaneMarker()

        lane_marker.left_rear.CopyFrom(load_point_3d(lane_markers_array[itr * 4, :]))
        lane_marker.right_rear.CopyFrom(load_point_3d(lane_markers_array[itr * 4 + 1, :]))
        lane_marker.right_front.CopyFrom(load_point_3d(lane_markers_array[itr * 4 + 2, :]))
        lane_marker.left_front.CopyFrom(load_point_3d(lane_markers_array[itr * 4 + 3, :]))

        lane_markers_obj.list_of_lane_markers.extend([lane_marker])
    return lane_markers_obj


def load_traffic_light_stop_points(stop_points_array):
    stop_points_obj = TrafficLightStopPoints()
    num_stop_points = int(stop_points_array.shape[0] / 1)
    for itr in range(num_stop_points):
        stop_point = TrafficLightStopPoint()
        stop_point.center.CopyFrom(load_point_3d(stop_points_array[itr * 1, :]))
        stop_points_obj.list_of_traffic_light_stop_points.extend([stop_point])
    return stop_points_obj


def load_traffic_lights(list_of_colors):
    lights_obj = TrafficLights()
    num_lights = int(len(list_of_colors))
    for itr in range(num_lights):
        light = TrafficLight()
        light.color = list_of_colors[itr]
        lights_obj.list_of_traffic_light.extend([light])
    return lights_obj


def load_2d_bounding_boxes(racer_name, nine_points):
    bounding_box_2d_obj = BoundingBox2D()
    bounding_box_2d_obj.name = racer_name
    bounding_box_2d_obj.bounding_box_2d.CopyFrom(load_bound_box(nine_points[0, :], nine_points[1:, :]))
    return bounding_box_2d_obj


def parse_lidar_data(data):
    change_from_left_handed_to_right_handed = -1
    data = data.decode("utf-8")
    data = data.split(DELIMITER)
    ts = data[1]
    lp = LidarPointCloud()
    data = np.array(data[2:], dtype=float).reshape(-1, 3)
    data[:, -1] *= change_from_left_handed_to_right_handed
    for p in range(data.shape[0]):
        lp.list_of_points.extend([load_point_3d(data[p, :])])

    return ts, lp


def parse_hd_map(data):
    change_from_left_handed_to_right_handed = -1
    data = data.decode("utf-8")
    data = data.split(DELIMITER)
    my_dictionary = {}
    ptr = 1
    while ptr < len(data):
        hd = HDMap()

        # road
        road_segment_name = data[ptr]
        ptr += 1

        road_coordinates = np.array([float(i) for i in data[ptr:ptr + 12]]).reshape(4, 3)
        road_coordinates[:, -1] *= change_from_left_handed_to_right_handed
        hd.road_segment.CopyFrom(load_road_segment(road_segment_name, road_coordinates))
        ptr += 12

        # road bounds: 2 x 4 = 8 points, 8 x 3 = 24
        road_bounds = np.array([float(i) for i in data[ptr:ptr + 24]]).reshape(8, 3)
        road_bounds[:, -1] *= change_from_left_handed_to_right_handed
        hd.road_bounds.CopyFrom(load_road_bounds(road_bounds))
        ptr += 24

        # road divider 1 x 4 = 4 points, 4 x 3 = 12
        road_divider = np.array([float(i) for i in data[ptr:ptr + 12]]).reshape(4, 3)
        road_divider[:, -1] *= change_from_left_handed_to_right_handed
        hd.road_divider.CopyFrom(load_road_divider(road_divider))
        ptr += 12

        # lanes 4 x 4 = 16 points, 16 x 3 = 48
        lane_coordinates = np.array([float(i) for i in data[ptr:ptr + 48]]).reshape(16, 3)
        lane_coordinates[:, -1] *= change_from_left_handed_to_right_handed
        hd.lane_markers.CopyFrom(load_lane_markers(lane_coordinates))
        ptr += 48

        my_dictionary[road_segment_name] = hd

    return my_dictionary


def parse_image(image):
    content = image[6:]
    ts = int(content[0:13])
    image = content[13:]
    image_new = np.array(Image.open(io.BytesIO(image)))
    return ts, image_new


def parse_data(data, plot_sw=False):
    """
    notice the x-y-z that is read from maya is left-handed coordinate system, here we convert it to right-handed
    :param data:
    :param plot_sw:
    :return:
    """
    my_message = Message()
    change_from_left_handed_to_right_handed = -1.0
    data = data.decode("utf-8")
    data = data.split(DELIMITER)
    # logger.info("length of elements (element 0 is the token) %s" % len(data))
    my_message.tag = data[1]

    my_message.timestamp = data[2]
    timestamp_readable = datetime.fromtimestamp(int(my_message.timestamp) / 1000.).strftime('%Y-%m-%d %H:%M:%S.%f')
    my_message.driver_name = data[3]
    my_message.vehicle_name = data[4]

    my_message.car_center.x, my_message.car_center.y, my_message.car_center.z = \
        float(data[5]), float(data[6]), float(data[7])

    # 9 point bbox
    nine_points = data[8:35]
    nine_points = [float(i) for i in nine_points]

    center = np.array(nine_points[0:3]).reshape(1, 3)
    center[:, -1] *= change_from_left_handed_to_right_handed

    eight_corners = np.array(nine_points[3:]).reshape(8, 3)
    eight_corners[:, -1] *= change_from_left_handed_to_right_handed

    my_message.car_bounding_box.CopyFrom(load_bound_box(center, eight_corners))

    # tire
    my_message.car_tire.right_front_tire_angle, my_message.car_tire.left_front_tire_angle, \
        my_message.car_tire.right_rear_tire_angle, my_message.car_tire.left_rear_tire_angle =\
        [float(i) for i in data[35:39]]

    # velocity
    v_x, v_y, v_z = [float(i) for i in data[39:42]]
    v_z *= -1
    my_message.car_velocity.vx = v_x
    my_message.car_velocity.vy = v_y
    my_message.car_velocity.vz = v_z

    # road name
    road_segment_name = data[42]
    my_message.road_segment_name = road_segment_name

    # traffic light data 2 stop points x 3 = 6, 1 light color, total 7
    lane_coordinates = np.array([float(i) for i in data[43:67]]).reshape(8, 3)
    lane_coordinates[:, -1] *= change_from_left_handed_to_right_handed
    my_message.traffic_light_stop_points.CopyFrom(load_traffic_light_stop_points(lane_coordinates))

    # 4 traffic light colors
    my_message.traffic_lights.CopyFrom(load_traffic_lights(data[67:71]))

    # controls of the car
    my_message.motor_input = float(data[71])
    my_message.brake_input = float(data[72])
    my_message.steer_input = float(data[73])
    my_message.hand_brake_input = float(data[74])

    my_message.left_light = int(data[75])
    my_message.right_light = int(data[76])

    # dash board camera image
    extra_length = 77
    if len(data) > extra_length:
        # 1 racer name + 9 points x-y = 1 + 9*2 = 19
        num_agents = int((len(data) - extra_length) / 19)
        for agt in range(num_agents):
            racer_name = data[extra_length + agt * 19]
            nine_points = data[extra_length + agt * 19 + 1: extra_length + (agt + 1) * 19]
            nine_points = np.array([float(i) for i in nine_points]).reshape([9, 2])  # x - col, y - row (top to bottom)
            my_message.car_bounding_boxes_2d.extend([load_2d_bounding_boxes(racer_name, nine_points)])

    return my_message



