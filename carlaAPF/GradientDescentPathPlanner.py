import numpy as np
from PIL import Image
import cv2


class Gradient_path_planner:
    def __init__(self, potential_field):
        self.potential_field = potential_field
        self.gradient_path = []

    def holonomic_gradient_descent(self):

        self.gradient_path = []  # reset path
        i, j = len(self.potential_field[0])//2, len(self.potential_field)//2
        self.gradient_path = [[i, j]]  # reset path starting at ego_vehicle

        directions = [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]
        while i in range(0, len(self.potential_field[0])) and j in range(0, len(self.potential_field)):
            temp_i, temp_j = i, j
            for dr, dc in directions:
                if self.potential_field[temp_i][temp_j] > self.potential_field[i + dr][j + dc]:
                    temp_i, temp_j = i + dr, j + dc

            i, j = temp_i, temp_j
            self.gradient_path.append([i, j])

            if self.gradient_path[-1] == self.gradient_path[-2]:
                break  # minima found

        # print("\n\ngradient path", self.gradient_path, "\n\n")
        return self.gradient_path

    def save_image_APF(self):

        grayscale = np.array(self.potential_field, dtype=np.uint8)
        for i in range(len(self.gradient_path)):
            px_x, px_y = self.gradient_path[i]
            grayscale[px_x][px_y] = 255

        apf_image = Image.fromarray(grayscale, mode="L")
        apf_image.save("APF_Image.bmp")

    def show_APF(self):
        img = cv2.imread("APF_Image.bmp")
        resized = cv2.resize(img, (500, 500), interpolation=cv2.INTER_AREA)
        normalized = cv2.normalize(resized, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow("image", normalized)
        cv2.waitKey(1)
