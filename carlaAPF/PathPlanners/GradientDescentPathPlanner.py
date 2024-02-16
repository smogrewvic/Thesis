import numpy as np
from PIL import Image
import cv2
from matplotlib import pyplot as plt


class Gradient_path_planner:
    def __init__(self, potential_field):
        self.potential_field = potential_field
        self.regression_precision = 1
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

        return self.gradient_path

    def __calculate_phi_max_directions(self,  phi_max, current_heading, radius = 5):
        i, j = len(self.potential_field[0]) // 2, len(self.potential_field) // 2
        mask_x = np.arange(0, len(self.potential_field[0]))
        mask_y = np.arange(0, len(self.potential_field))

        mask = (mask_x[np.newaxis, :] - i) ** 2 + (mask_y[:, np.newaxis] - j) ** 2 < radius ** 2
        circle = np.argwhere(mask == True) - [i, j]
        phi_max_directions = []

        max_turn = np.pi / 2 + phi_max / 2 + current_heading
        min_turn = np.pi / 2 - phi_max / 2 - current_heading
        for coords in circle:
            if min_turn <= np.arctan2(coords[0], coords[1]) <= max_turn:
                phi_max_directions.append(-coords)

        return phi_max_directions
    def phi_max_gradient_descent(self, phi_max):
        self.gradient_path = []  # reset path
        origin_i, origin_j = len(self.potential_field[0]) // 2, len(self.potential_field) // 2
        i, j = len(self.potential_field[0]) // 2, len(self.potential_field) // 2

        self.gradient_path = [[i, j]]  # reset path starting at ego_vehicle
        self.gradient_heading = [0]

        while i in range(0, len(self.potential_field[0])) and j in range(0, len(self.potential_field)):
            directions = self.__calculate_phi_max_directions(phi_max, self.gradient_heading[-1])
            temp_i, temp_j = i, j
            for dr, dc in directions:
                if i+dr > len(self.potential_field[0]) or j+dc > len(self.potential_field):
                    break
                if self.potential_field[temp_i][temp_j] > self.potential_field[i + dr][j + dc]:
                    temp_i, temp_j = i + dr, j + dc

            i, j = temp_i, temp_j
            self.gradient_path.append([i, j])
            self.gradient_heading.append(np.arctan2(self.gradient_path[-2][0] - self.gradient_path[-1][0], self.gradient_path[-2][1] - self.gradient_path[-1][1]))

            if self.gradient_path[-1] == self.gradient_path[-2]:
                break  # minima found


        self.gradient_path
        return self.gradient_path
    def phi_max_regressed_descent(self, phi_max, regression_precision = 0.01, stop_at_minima = True):
        self.regression_precision = regression_precision
        self.phi_max_gradient_descent(phi_max)

        path_end_x, path_end_y = self.gradient_path[-1][0], self.gradient_path[-1][1]
        self.gradient_path = self.path_regression()
        if stop_at_minima == True:
            for i in range(len(self.gradient_path)):
                if self.gradient_path[i][0]>=path_end_x:
                    self.gradient_path = self.gradient_path[i:]

                    return self.gradient_path

        return self.gradient_path

    def path_regression(self, poly_order=3):
        coeffs = self.regression_coefficients(poly_order)
        if len(coeffs) == 1: ## poorly conditioned coeffs from regression
            return self.gradient_path

        path_distance = len(self.potential_field) // 2
        regressed_path = []
        for x in np.arange(0, path_distance, self.regression_precision):
            fx = 0
            for i, c_i in enumerate(coeffs[::-1]):
                fx += c_i * x ** i
            regressed_path.append([x,fx])

        return regressed_path #flip the path for it to start at ego
    def get_regression_precision(self):
        return self.regression_precision
    def regression_coefficients(self, poly_order=3):

        discreet_path = np.array(self.gradient_path)
        if len(discreet_path)<= poly_order+1:  # return null coeffs if polyfit is poorly conditioned
            return [0]
        coeffs = np.polyfit(discreet_path[:,0], discreet_path[:,1], poly_order)

        return coeffs
    def save_image_APF(self):

        grayscale = np.array(self.potential_field, dtype=np.uint8)
        for i in range(len(self.gradient_path)):
            px_x, px_y = round(self.gradient_path[i][0]), round(self.gradient_path[i][1])
            px_x, px_y = max(min(px_x, len(grayscale)),0), max(min(px_y, len(grayscale)),0)  # constrain to img size
            grayscale[px_x][px_y] = 255

        apf_image = Image.fromarray(grayscale, mode="L")
        apf_image.save("APF_Image.bmp")

    def show_APF(self):
        img = cv2.imread("APF_Image.bmp")
        resized = cv2.resize(img, (500, 500), interpolation=cv2.INTER_AREA)
        cv2.imshow("image",resized)

        ## TODO: WARNING - Normalizing can make everything 0 for a flat apf
        # normalized = cv2.normalize(resized, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow("image", normalized)

        cv2.waitKey(1)
