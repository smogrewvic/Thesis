import numpy as np
import math
import matplotlib.pyplot as plt
import skfuzzy as fuzz
import skfuzzy.control as ctrl

from SVO.FuzzyControllers.SVO_Fuzzy import SVO_Fuzzy


class Pedestrian_SVO_Fuzzy(SVO_Fuzzy):
    def fuzzy_setup(self):
        # Create fuzzy variables
        input1 = ctrl.Antecedent(np.arange(0, 10, 0.1), 'distance_to_crosswalk')
        input2 = ctrl.Antecedent(np.arange(0, 10, 0.1), 'time_waiting')
        input3 = ctrl.Antecedent(np.arange(0, 10, 0.1), 'time_looking')

        output1 = ctrl.Consequent(np.arange(-1,1, 0.1), 'svo')

        # input membership functions
        input1['close'] = fuzz.zmf(input1.universe, 1.5, 6)
        input1['far'] = fuzz.smf(input1.universe, 1.5, 6)

        input2['short'] = fuzz.zmf(input2.universe, 0.5, 5)
        input2['medium'] = fuzz.gaussmf(input2.universe, 1, 3.5)
        input2['long'] = fuzz.smf(input2.universe, 1, 7)

        input3['short'] = fuzz.zmf(input3.universe, 1, 6)
        input3['medium'] = fuzz.gaussmf(input3.universe, 1, 4)
        input3['long'] = fuzz.smf(input3.universe, 1.5, 7)

        # output memberships
        output1['altruistic'] = fuzz.gaussmf(output1.universe, 0.25, -1)
        output1['cooperative'] = fuzz.gaussmf(output1.universe, 0.25, -0.5)
        output1['individualistic'] = fuzz.gaussmf(output1.universe, 0.25, 0)
        output1['egoistic'] = fuzz.gaussmf(output1.universe, 0.25, 0.5)
        output1['sadistic'] = fuzz.gaussmf(output1.universe, 0.25, 1)


        # Create fuzzy rules
        rule1 = ctrl.Rule(input1['close'] & input2['short'] & input3['short'], output1['sadistic'])
        rule2 = ctrl.Rule(input1['close'] & input2['medium'] & input3['short'], output1['egoistic'])
        rule3 = ctrl.Rule(input1['close'] & input2['long'] & input3['short'], output1['individualistic'])
        rule4 = ctrl.Rule(input1['close'] & input2['long'] & input3['medium'], output1['cooperative'])
        rule5 = ctrl.Rule(input1['close'] & input2['long'] & input3['long'], output1['cooperative'])
        rule6 = ctrl.Rule(input1['far'] & input2['short'] & input3['short'], output1['individualistic'])
        rule7 = ctrl.Rule(input1['far'] & input2['medium'] & input3['short'], output1['cooperative'])
        rule8 = ctrl.Rule(input1['far'] & input2['medium'] & input3['medium'], output1['cooperative'])
        rule9 = ctrl.Rule(input1['far'] & input2['long'] & input3['medium'], output1['altruistic'])
        rule10 = ctrl.Rule(input1['far'] & input2['long'] & input3['long'], output1['altruistic'])


        # Create fuzzy control system
        control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10])
        fis_simulation = ctrl.ControlSystemSimulation(control_system)

        return fis_simulation

    def calculate_output(self, input_vector):

        # Set inputs and compute output
        self.fuzzy_controller.input['distance_to_crosswalk'] = input_vector['distance_to_crosswalk']
        self.fuzzy_controller.input['time_waiting'] = input_vector['time_waiting']
        self.fuzzy_controller.input['time_looking'] = input_vector['time_looking']

        try:
            self.fuzzy_controller.compute()
        except:
            print("crisp output not possible")
            return 0

        return self.fuzzy_controller.output['svo']
