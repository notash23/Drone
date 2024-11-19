import os
import cv2
import numpy as np
import tensorflow as tf
from djitellopy import Tello
from object_detection.builders import model_builder
from object_detection.utils import config_util
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
from flying.utils.PID import PID, Position
from flying.utils.State import State

PIPELINE_CONFIG = os.path.join('resources', 'pipeline.config')
CHECKPOINT = os.path.join('resources', 'ckpt-0')
LABEL_MAP = os.path.join('resources', 'label_map.pbtxt')

state= State.FOLLOW

# Load pipeline config and build a detection model
configs = config_util.get_configs_from_pipeline_file(PIPELINE_CONFIG)
detection_model = model_builder.build(model_config=configs['model'], is_training=False)
ckpt = tf.compat.v2.train.Checkpoint(model=detection_model)
ckpt.restore(CHECKPOINT).expect_partial()
category_index = label_map_util.create_category_index_from_labelmap(LABEL_MAP)

pid = PID(
    k_p=20,
    k_i=0,
    k_d=0,
    robot_position=Position(0.5, 0.5, 0.14)
)

@tf.function
def detect_fn(image):
    image, shapes = detection_model.preprocess(image)
    prediction_dict = detection_model.predict(image, shapes)
    p_detections = detection_model.postprocess(prediction_dict, shapes)
    return p_detections

d = Tello()
d.connect()

print(f"Battery Status: {d.get_battery()}")

# Start recording
d.streamon()
frame_read = d.get_frame_read()

d.takeoff()

while True:
    img = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)
    input_tensor = tf.convert_to_tensor(np.expand_dims(img, 0), dtype=tf.float32)
    detections = detect_fn(input_tensor)

    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy()
                  for key, value in detections.items()}
    detections['num_detections'] = num_detections

    # detection_classes should be ints.
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

    label_id_offset = 1
    image_np_with_detections = img.copy()

    viz_utils.visualize_boxes_and_labels_on_image_array(
        image_np_with_detections,
        detections['detection_boxes'],
        detections['detection_classes'] + label_id_offset,
        detections['detection_scores'],
        category_index,
        use_normalized_coordinates=True,
        max_boxes_to_draw=1,
        min_score_thresh=.8,
        agnostic_mode=False)

    detection = None
    for i in range(detections['detection_boxes'].shape[0]):
        if detections['detection_scores'][i] > 0.8:
            detection = detections['detection_boxes'][i]
            break
    # TODO: Add filter to d component
    # TODO: calculate the position heuristics to find best detection
    # TODO: use position heuristics if no detection
    # TODO: change to state to 'find' if no detection for 10 frames
    # TODO: add collision avoidance
    if detection is not None:
        x_mean = (detection[1]+detection[3])/2
        y_mean = (detection[0]+detection[2])/2
        area = (detection[3]-detection[1])*(detection[2]-detection[0])
        yaw, h_speed, v_speed = pid.get_rc_controls(Position(x_mean, y_mean, area))

        d.send_rc_control(left_right_velocity=0,
                          forward_backward_velocity=int(h_speed)*10,
                          up_down_velocity=0,
                          yaw_velocity=-int(yaw)*5)

    # TODO: try to control lr velocity to orbit around the robot

    cv2.imshow('frame', image_np_with_detections)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        d.end()
        break
