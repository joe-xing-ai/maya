import tensorflow as tf
import time

from nanomsg import Socket, PUB, SUB_SUBSCRIBE, SUB, REQ, PAIR

from utilities import wake_up_maya, parse_data, parse_lidar_data, parse_hd_map, parse_image, maya_to_sleep

FLAGS = tf.app.flags.FLAGS

tf.logging.set_verbosity("INFO")

tf.flags.DEFINE_string("maya_path", "car_race.app/Contents/MacOS/car_race", "path to maya simulation engine")  # NOQA
tf.flags.DEFINE_string("constraints_for_training", "0,0;1,1", "the constraints points for training the RL")
tf.flags.DEFINE_string("start_position", "1:124.716:-566.066:-0.4:0:0.0:0.0:0.0:"
                                         "9:124.716:-454.06600000000003:-0.4:4:0.0:0.0:0.0:"
                                         "8:119.04599999999999:-150.06600000000003:-0.4:5:0.0:180.0:0.0:"
                                         "7:119.04599999999999:-174.06600000000003:-0.4:2:0.0:180.0:0.0:"
                                         "6:114.976:-198.06600000000003:-0.4:4:0.0:180.0:0.0:"
                                         "5:119.04599999999999:-350.06600000000003:-0.4:2:0.0:180.0:0.0:"
                                         "4:124.716:-374.06600000000003:-0.4:5:0.0:0.0:0.0:"
                                         "3:128.786:-462.06600000000003:-0.4:4:0.0:0.0:0.0:"
                                         "2:124.716:-694.066:-0.4:4:0.0:0.0:0.0",
                       "start position of agents")
tf.flags.DEFINE_float("road_width", 45., "road width")
tf.flags.DEFINE_float("road_length", 1000., "road length")
tf.flags.DEFINE_float("lane_width", 6.5, "lane width")
tf.flags.DEFINE_string("log_port", "ipc:///tmp/test_log.ipc", "port for reading out data from simulator")
tf.flags.DEFINE_string("log_port_image", "ipc:///tmp/test_log_image.ipc", "port for reading out image from simulator")
tf.flags.DEFINE_string("log_port_lidar", "ipc:///tmp/test_log_lidar.ipc", "port for reading out lidar from simulator")
tf.flags.DEFINE_string("log_port_hd_map", "ipc:///tmp/test_log_hd_map.ipc", "port for reading out hd map from simulator")  # NOQA
tf.flags.DEFINE_string("control_port", "ipc:///tmp/test_control.ipc", "port for sending commands to simulator")
tf.flags.DEFINE_string("prediction_port", "ipc:///tmp/test_prediction.ipc", "port for sending predictions to simulator")
tf.flags.DEFINE_integer("log_frequency", 20, "every N fixUpdates (10 ms) to publish a message")
tf.flags.DEFINE_integer("log_lidar_frequency", 100, "every N fixUpdates (10 ms) to update lidar point cloud")

tf.flags.DEFINE_bool("dump_png", False, "to dump image out from simulator")
tf.flags.DEFINE_integer("dump_png_frequency", 500, "every N broadcasts to dump a png image out")

tf.flags.DEFINE_bool("dump_lidar", False, "to dump lidar out from simulator")
tf.flags.DEFINE_integer("dump_lidar_frequency", 500, "every N broadcasts to dump a lidar out")
tf.flags.DEFINE_bool("dump_hd_map", False, "to dump hd map out from simulator")
tf.flags.DEFINE_integer("dump_hd_map_frequency", 5000, "every N broadcasts to dump a hd map out")

tf.flags.DEFINE_boolean("bbox", True, "not show bbox on image")
tf.flags.DEFINE_boolean("vehicle_name", True, "not show vehicle name on image")
tf.flags.DEFINE_float("camera_distance", 15, "follow camera distance to ego vehicle")
tf.flags.DEFINE_float("camera_height", 3, "follow camera height")
tf.flags.DEFINE_float("camera_look_at_height", 0.5, "follow camera look at height")
tf.flags.DEFINE_bool("skeleton", False, "display skeleton mode or not")
tf.flags.DEFINE_string("intent_render_format", "text", "control maya simulator to render text or gui format of intent prediction")  # NOQA
tf.flags.DEFINE_bool("batch_mode", False, "run in batch mode, no graphics")


def test_vehicle_data_logging():
    proc = wake_up_maya(start_position=None, constraints_for_training=None)

    # read data
    socket = Socket(SUB)
    socket.set_string_option(SUB, SUB_SUBSCRIBE, b"Publish Counter")
    socket.connect(FLAGS.log_port)

    # send control commands
    socket_out = Socket(PAIR)
    socket_out.connect(FLAGS.control_port)

    num_of_agents = 9
    num_of_steps = 30

    for steps in range(num_of_steps):
        for ptr in range(num_of_agents):
            vehicle_data = socket.recv()
            msg = parse_data(vehicle_data)
            tf.logging.info("%s" % msg)
        control_signal = "%s:%s" % (int(time.time() * 1000), 0)
        motor = 0.5
        brake = 0.
        steer = 0.
        hand_brake = 0.
        left_light = 0
        right_light = 0
        control_signal += ":%s:%s:%s:%s:%s:%s:%s" % ("my_ego_vehicle", motor, brake, steer, hand_brake,
                                                     left_light, right_light)
        socket_out.send(control_signal.encode())

    maya_to_sleep(proc)


def test_lidar_logging():
    proc = wake_up_maya(start_position=None, constraints_for_training=None)

    socket = Socket(SUB)
    socket.set_string_option(SUB, SUB_SUBSCRIBE, b"lidar")
    socket.connect(FLAGS.log_port_lidar)
    lidar = socket.recv()
    ts, point_cloud = parse_lidar_data(lidar)
    tf.logging.info("timestamp: %s, point cloud: %s" % (ts, point_cloud))
    maya_to_sleep(proc)


def test_hd_map_logging():
    proc = wake_up_maya(start_position=None, constraints_for_training=None)

    socket = Socket(SUB)
    socket.set_string_option(SUB, SUB_SUBSCRIBE, b"hd_map")
    socket.connect(FLAGS.log_port_hd_map)
    hd_map = socket.recv()
    my_hd_map = parse_hd_map(hd_map)

    tf.logging.info("%s" % my_hd_map)
    maya_to_sleep(proc)


def test_image_logging():
    proc = wake_up_maya(start_position=None, constraints_for_training=None)

    socket = Socket(SUB)
    socket.set_string_option(SUB, SUB_SUBSCRIBE, b"image")
    socket.connect(FLAGS.log_port_image)
    image = socket.recv()
    ts, image = parse_image(image)
    tf.logging.info("ts: %s image shape: %s" % (ts, image.shape))
    maya_to_sleep(proc)


def main():
    if FLAGS.dump_lidar:
        test_lidar_logging()
    elif FLAGS.dump_hd_map:
        test_hd_map_logging()
    elif FLAGS.dump_png:
        test_image_logging()
    else:
        test_vehicle_data_logging()


if __name__ == "__main__":
    main()
