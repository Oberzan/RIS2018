import cv2
import numpy as np

class ColorLabeler():

    def __init__(self):
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        lower_blue = np.array([90, 100, 100])
        upper_blue = np.array([130, 255, 255])

        lower_green = np.array([40, 100, 100])
        upper_green = np.array([85, 255, 255])

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])

        self.labels = {
            "Red": (lower_red, upper_red),
            "Blue": (lower_blue, upper_blue),
            "Green": (lower_green, upper_green),
            "Yellow": (lower_yellow, upper_yellow)
        }

    def _calculate_color_match(self, image, color):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        lower_bound, upper_bound = self.labels.get(color)

        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        num_pixels = np.shape(mask)[0] * np.shape(mask)[1]
        num_matching_pixels = np.count_nonzero(mask)

        match_procentage = num_matching_pixels / num_pixels * 100
        return match_procentage

    def extract_color(self, image, match_confidence=15):
        matched_color = None
        best_color_match = 0
        for color in self.labels:
            calculated_match = self._calculate_color_match(image, color)
            if calculated_match > best_color_match:
                matched_color = color
                best_color_match = calculated_match

        if best_color_match > match_confidence:
            return matched_color
        else:
            print("No color with match confidence " + str(match_confidence) + " found.")
            print("Best match :", matched_color, best_color_match)
            return None
