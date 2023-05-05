

class Gradient_path_planner:
    def __init__(self, potential_field):
        self.potential_field = potential_field
        self.gradient_path = []

    def holonomic_gradient_descent(self):

        self.gradient_path = [] #reset path
        i, j = self.potential_field / 2, self.potential_field / 2
        self.gradient_path = [[i,j]]  # reset path starting at ego_vehicle

        directions = [[0,1], [1,1], [1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]]
        while i in range(0, len(self.potential_field)) and j in range(0, len(self.potential_field)):
            lowest_potential = self.potential_field[i][j]

            for dr,dc in directions:
                if lowest_potential > self.potential_field[i+dr][j+dc]:
                    i, j = i+dr, j+dc
            self.gradient_path.append([i,j])
            if self.gradientpath[-1] == self.gradientpath[-2]:
                break #minima found


        return self.gradient_path

