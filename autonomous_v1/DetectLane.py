import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model

import SteeringAngle.Steering as steering
import SteeringAngle.utils as utils

class DetectLane:
    def __init__(self):
        self.setGPU()
        self.model = load_model('model.h5')

        # initials = [163, 167, 30, 240]
        # utils.initializeTrackbars(initials)
        
    def setGPU(self):
        gpus = tf.config.experimental.list_physical_devices('GPU')
        if gpus:
            try:
                for gpu in gpus:
                    tf.config.experimental.set_virtual_device_configuration(
                        gpu,
                        [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=2048)]
                    )
            except RuntimeError as e:
                print(e)

    class Lanes:
        def __init__(self):
            self.recent_fit = []
            self.avg_fit = []

    def detectLanes(self, image):
        """ Takes in a road image, re-sizes for the model,
        predicts the lane to be drawn from the model then merge with
        original road image.
        """
        # Create lanes object
        lanes = self.Lanes()
        

        # Get image ready for feeding into model
        small_img = cv2.resize(image, (160, 80))
        small_img = np.array(small_img)
        small_img = small_img[None, :, :, :]


        # Make prediction with neural network (un-normalize value by multiplying by 255)
        prediction = self.model.predict(small_img)[0] * 255

        # Add lane prediction to list for averaging
        lanes.recent_fit.append(prediction)
        # Only using last five for average
        if len(lanes.recent_fit) > 5:
            lanes.recent_fit = lanes.recent_fit[1:]

        # Calculate average detection
        lanes.avg_fit = np.mean(np.array([i for i in lanes.recent_fit]), axis=0)


        blanks = np.zeros_like(lanes.avg_fit).astype(np.uint8)
        lane_drawn = np.dstack((lanes.avg_fit, blanks, blanks))

        # Re-size to match the original image
        lane_image = cv2.resize(lane_drawn, (image.shape[1], image.shape[0]))
        # float_lane_image = np.float32(lane_image)
        # rgb_image = cv2.cvtColor(lane_image, cv2.COLOR_BGR2RGB)
        result = cv2.addWeighted(image, 0, lane_image, 1, 0, dtype=cv2.CV_8UC3)
        steeringAngle = steering.getLaneCurve(result, display=2) 

        # Merge the lane drawing onto the original image
        result = cv2.addWeighted(image, 1, lane_image, 1, 0, dtype=cv2.CV_8UC3)

        return result, steeringAngle
