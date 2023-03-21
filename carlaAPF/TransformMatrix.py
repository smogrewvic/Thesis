import numpy as np

def rotate2D(matrix, angle, degrees = True, north_zero = True, ccw_positive = True):

    print("north reference heading", angle)
    #todo: implement angle offset
    # if north_zero == True:
    #     if -180 < angle <= 0:
    #         angle+=90
    #     elif 0 < angle < 180:
    #         angle-=90
    #     # angle-=90
    #     # angle=-angle
    # angle+=180
    print("TRUE heading", angle)
    if degrees == True: angle = np.radians(angle)


    if ccw_positive == False:
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],[np.sin(angle),np.cos(angle), 0], [0, 0, 1]])
    else:
        rotation = np.array([[np.cos(angle), np.sin(angle), 0],[-np.sin(angle),np.cos(angle), 0], [0, 0, 1]])

    matrix = np.array([matrix])  #needs to be 2D matrix for transpose
    # rotated_matrix = rotation * matrix.T
    rotated_matrix = rotation.dot(matrix.T)
    flattened_matrix = np.array([rotated_matrix[0][0], rotated_matrix[1][0], rotated_matrix[2][0]]) # make 1D again

    return flattened_matrix



def stretch2D(matrix, strech_factor):

    stretch_matrix = np.array([[strech_factor,0],[0, strech_factor]])
    matrix = np.array([matrix])  # needs to be 2D matrix for transpose

    streched_matrix = stretch_matrix * matrix.T
    flattened_matrix = np.array([streched_matrix[0][0], streched_matrix[1][0], streched_matrix[2][0]])  # make 1D again
    return flattened_matrix
