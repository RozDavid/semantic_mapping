import cv2
import numpy as np
import sys
import argparse
import csv


class ColorSegmenter(object):

    def __init__(self, filepath):

        self.labels = []
        self.kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        self.load_color_labels(filepath)


    def load_color_labels(self, filename):

        with open(filename, 'r') as data:
            for line in csv.DictReader(data):

                line_dict = dict()

                src_rgb = np.array([[[int(line["target_r"]), int(line["target_g"]), int(line["target_b"])]]],
                                                dtype=np.uint8)
                line_dict["source_rgb"] = src_rgb.flatten()

                line_dict["mask_rgb"] = np.array(
                    [int(line["label_r"]), int(line["label_g"]), int(line["label_b"])],
                    dtype=np.uint8)
                hsv = cv2.cvtColor(src_rgb, cv2.COLOR_RGB2HSV)

                line_dict["source_hsv"] = hsv.flatten()

                line_dict["label_id"] = int(line["label_id"])

                treshold = float(line["treshold"])
                line_dict["treshold"] = np.array([treshold * 180, treshold * 255, treshold * 255])

                line_dict["name"] = line["name"]
                self.labels.append(line_dict)

    def create_blank_color(self, img_shape, rgb_color=(0, 0, 0)):
        """Create new image(numpy array) filled with certain color in RGB"""
        # Create black blank image
        image = np.zeros((img_shape[0], img_shape[1], 3), np.uint8)

        color = tuple(rgb_color)
        # Fill image with color
        image[:] = color


        return image

    def mask_image(self, image, is_rgb=True):

        # Create color mask background with void_color values
        result_image = self.create_blank_color(image.shape, (200, 200, 200))
        blurred = cv2.blur(image, (3, 3))

        # Convert to HSV
        if is_rgb:
            hsv_image = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
        else:
            hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        for label in self.labels:

            source_color = label["source_hsv"]
            label_clr = label["mask_rgb"]

            min_color = source_color - (label["treshold"] // 2)
            max_color = source_color + (label["treshold"] // 2)

            mask = cv2.inRange(hsv_image, min_color, max_color)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel5)

            # tmp_image = cv2.bitwise_and(label_clr_image, label_clr_image, mask=mask)

            result_image[mask > 0] = label_clr

        return result_image


if __name__ == '__main__':

    img = cv2.imread('./samples/sample_rgb.png')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    parser = argparse.ArgumentParser(description='Segment Images by color channels')

    parser.add_argument('--labels_filepath', type=str, default='./config/eit_clc_segmentation_classes.csv',
                        help='The relative path where the color tresholds are located')

    args = parser.parse_args()

    colorSegmenter = ColorSegmenter(args.labels_filepath)
    result = colorSegmenter.mask_image(img)

    orig = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    result = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
    cv2.imshow('Color masked image', cv2.hconcat([orig, result]))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
