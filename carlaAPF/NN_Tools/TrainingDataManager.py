
import os
import pickle
import json
class Training_Data_Manager:

    def __init__(self, folder_path, serialization_type):
        self.folder_path = folder_path
        self.serialization = serialization_type
        self.file_increment = 0

    def save_training_file(self, inputs, outputs):

        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)
        file_path = os.path.join(self.folder_path, f"GD{self.file_increment}.pickle")

        training_data = {'inputs':inputs, 'outputs':outputs}

        if self.serialization == 'json':
            with open(file_path, 'w') as json_file:
                json.dump(training_data, json_file, indent=4)

        elif self.serialization == 'pickle':
            with open(file_path, 'wb') as pickle_file:
                pickle.dump(training_data, pickle_file)
            print(f"Data saved to {file_path}")

        else:
            raise ValueError('Invalid serialization set. Use "json" or "pickle"')

        self.file_increment+=1


    def load_training_file(self):

        lst = os.listdir(self.folder_path)  # your directory path
        number_files = len(lst)

        # open_file = r"C:\Users\victor\Desktop\Gradient Descent Training Data\Dataset 1\test0.pickle"
        file_path = os.path.join(self.folder_path, f"GD{number_files-1}.pickle")
        with open(file_path, 'rb') as pickle_file:
            reconstructed_data = pickle.load(pickle_file)

            print(type(reconstructed_data['outputs']['regression_coefficients']))
            print(reconstructed_data['outputs'], '\n')