import numpy as np
from PIL import Image
import cv2


class Gradient_path_planner:
    def __init__(self, potential_field):
        self.potential_field = potential_field
        self.gradient_path = []
        self.gradient_heading = []

    def holonomic_gradient_descent(self):

        self.gradient_path = []  # reset path
        i, j = len(self.potential_field[0])//2, len(self.potential_field)//2
        self.gradient_path = [[i, j]]  # reset path starting at ego_vehicle
        self.gradient_heading = [0]  # reset heading to straight forward

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

    def __calculate_phi_max_directions(self,  phi_max, current_heading, radius = 5):
        i, j = len(self.potential_field[0]) // 2, len(self.potential_field) // 2
        mask_x = np.arange(0, len(self.potential_field[0]))
        mask_y = np.arange(0, len(self.potential_field))

        mask = (mask_x[np.newaxis, :] - i) ** 2 + (mask_y[:, np.newaxis] - j) ** 2 < radius ** 2
        circle = np.argwhere(mask == True) - [i, j]
        phi_max_directions = []

        for coords in circle:
            if np.pi / 2 - phi_max / 2 <= np.arctan2(coords[0], coords[1]) <= np.pi / 2 + phi_max / 2:
                phi_max_directions.append(-coords)

        return phi_max_directions
    def phi_max_gradient_descent(self, phi_max):
        self.gradient_path = []  # reset path
        i, j = len(self.potential_field[0]) // 2, len(self.potential_field) // 2

        # path_heading = [0]  # initial heading
        self.gradient_path = [[i, j]]  # reset path starting at ego_vehicle
        self.gradient_heading = [0]

        while i in range(0, len(self.potential_field[0])) and j in range(0, len(self.potential_field)):
            directions = self.__calculate_phi_max_directions(phi_max, self.gradient_heading)
            # print("directions", directions)
            temp_i, temp_j = i, j
            for dr, dc in directions:
                # print("dr,dc", dr,dc)
                if self.potential_field[temp_i][temp_j] > self.potential_field[i + dr][j + dc]:
                    temp_i, temp_j = i + dr, j + dc

            i, j = temp_i, temp_j
            self.gradient_heading.append(np.arctan2(i - self.gradient_path[-1][0], j - self.gradient_path[-1][1]))
            self.gradient_path.append([i, j])

            if self.gradient_path[-1] == self.gradient_path[-2]:
                # print("minima", i,j)
                break  # minima found

        # print("\n\ngradient path", self.gradient_path, "\n\n")
        return self.gradient_path


    def interpolate_results(self, output_size):

        pass
        # coeffs = np.polyfit(local_navpoints_x, local_navpoints_y, 4)


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
