"""
To run this script, you need

 - pipeline config (.config) file
 - checkpoint (.ckpt) file
 - [Optional, if you just want to guess what it is] label map (.pbtxt) file
"""

import os
import time
import cv2
import numpy as np
import tensorflow as tf
from object_detection.builders import model_builder
from object_detection.utils import config_util
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

PIPELINE_CONFIG = os.path.join('Tensorflow', 'workspace','models', 'robot_ssd_mobnet_big', 'pipeline.config')
CHECKPOINT = os.path.join('Tensorflow', 'workspace', 'models', 'robot_ssd_mobnet_big', 'ckpt-16')
LABEL_MAP = os.path.join('Tensorflow', 'workspace', 'annotations', 'label_map.pbtxt')

# Load pipeline config and build a detection model
configs = config_util.get_configs_from_pipeline_file(PIPELINE_CONFIG)
detection_model = model_builder.build(model_config=configs['model'], is_training=False)

# Restore checkpoint
ckpt = tf.compat.v2.train.Checkpoint(model=detection_model)
ckpt.restore(CHECKPOINT).expect_partial()

# Category Label Map
category_index = label_map_util.create_category_index_from_labelmap(LABEL_MAP)

@tf.function
def detect_fn(image):
    image, shapes = detection_model.preprocess(image)
    prediction_dict = detection_model.predict(image, shapes)
    p_detections = detection_model.postprocess(prediction_dict, shapes)
    return p_detections


cap = cv2.VideoCapture(0)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

while cap.isOpened():
    start = time.perf_counter()
    ret, frame = cap.read()
    image_np = np.array(frame)

    input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)
    detections = detect_fn(input_tensor)

    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy()
                  for key, value in detections.items()}
    detections['num_detections'] = num_detections

    # detection_classes should be ints.
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

    label_id_offset = 1
    image_np_with_detections = image_np.copy()

    viz_utils.visualize_boxes_and_labels_on_image_array(
        image_np_with_detections,
        detections['detection_boxes'],
        detections['detection_classes'] + label_id_offset,
        detections['detection_scores'],
        category_index,
        use_normalized_coordinates=True,
        max_boxes_to_draw=5,
        min_score_thresh=.8,
        agnostic_mode=False)

    cv2.putText(image_np_with_detections,f'{round((time.perf_counter()-start)*1000)}ms', (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    cv2.imshow('object detection', cv2.resize(image_np_with_detections, (800, 600)))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
