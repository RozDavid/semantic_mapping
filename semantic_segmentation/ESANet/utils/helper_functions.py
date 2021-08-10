import numpy as np
import cv2


def center_crop(self, img, dim):
    width, height = img.shape[1], img.shape[0]  # process crop width and height for max available dimension
    crop_width = dim[0] if dim[0] < img.shape[1] else img.shape[1]
    crop_height = dim[1] if dim[1] < img.shape[0] else img.shape[0]

    mid_x, mid_y = int(width / 2), int(height / 2)
    cw2, ch2 = int(crop_width / 2), int(crop_height / 2)
    crop_img = img[mid_y - ch2:mid_y + ch2, mid_x - cw2:mid_x + cw2]
    return crop_img


def filter_depth(self, rgb_image, depth_image, confidence_image):
    conditions = np.isnan(depth_image) | (depth_image == -np.inf) | (
            depth_image == np.inf)
    filtered_image = np.where(conditions, 0, depth_image)
    filtered_image = cv2.medianBlur(filtered_image, self.median_filter_size)

    return filtered_image
