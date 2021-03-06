import numpy as np
import cv2
from styx_msgs.msg import TrafficLight


def classify_light_with_bounding_box(pts, image):

    state = TrafficLight.UNKNOWN
    best_score = None
    if pts is not None and len(pts) > 0:

        pts = np.array(pts, dtype=np.int)
        img_shape = image.shape
        light_pixels = image[min(max(0, pts[0][1]), img_shape[0]):min(max(0, pts[3][1]), img_shape[0]),
                       min(max(0, pts[0][0]), img_shape[1]):min(max(0, pts[3][0]), img_shape[1]),
                       :]

        if light_pixels.shape[0] * light_pixels.shape[1] > 0:

            light_pixels = cv2.cvtColor(light_pixels, cv2.COLOR_RGB2HSV)[:, :, 2]  # we only test with S channel
            # filter_light_pixels = np.array(light_pixels).astype(np.int32) - 200
            # filter_light_pixels = np.clip(filter_light_pixels, 0, 255)
            # mpimg.imsave('tl_detected.png', light_pixels, cmap='gray', origin='upper')
            # mpimg.imsave('tl_detected2.png', filter_light_pixels, cmap='gray', origin='upper')

            states = [TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN]
            shape = light_pixels.shape
            split_h = shape[0] / len(states)

            # print("split_h = ", split_h)

            best_score = 0

            for i, light_state in enumerate(states):
                light_segment = light_pixels[i * split_h:(i + 1) * split_h, :]
                # print("shape= ", light_segment.shape)
                # print(np.where(light_segment > 100))
                score = len(np.where(light_segment > 200)[0])

                # print("index {}, state {}, score {}".format(i, light_state, score))
                if score > best_score:
                    state = light_state
                    best_score = score

    if best_score is not None and best_score < 25:
        state = TrafficLight.UNKNOWN

    return state