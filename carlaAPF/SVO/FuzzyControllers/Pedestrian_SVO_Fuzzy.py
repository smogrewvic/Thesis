import numpy as np
import math
import matplotlib.pyplot as plt
import skfuzzy as fuzz
import skfuzzy.control as ctrl

from SVO.FuzzyControllers.SVO_Fuzzy import SVO_Fuzzy


class Pedestrian_SVO_Fuzzy(SVO_Fuzzy):
    def fuzzy_setup(self):
        # Create fuzzy variables
        input1 = ctrl.Antecedent(np.arange(0, 50, 0.1), 'distance_to_crosswalk')
        input2 = ctrl.Antecedent(np.arange(0, 10, 0.1), 'time_waiting')
        input3 = ctrl.Antecedent(np.arange(0, 10, 0.1), 'time_looking')

        output1 = ctrl.Consequent(np.arange(-1,1, 0.1), 'svo')

        # input membership functions
        input1['close'] = fuzz.zmf(input1.universe, 5, 30)
        input1['far'] = fuzz.smf(input1.universe, 10, 40)

        input2['short'] = fuzz.zmf(input2.universe, 0.5, 5)
        input2['long'] = fuzz.smf(input2.universe, 1, 7)

        input3['short'] = fuzz.zmf(input3.universe, 1, 6)
        input3['long'] = fuzz.smf(input3.universe, 2, 8)

        # output memberships
        output1['cooperative'] = fuzz.zmf(output1.universe, -1, 0.5)
        output1['individualistic'] = fuzz.gaussmf(output1.universe, 0, 0.25)
        output1['egoistic'] = fuzz.smf(output1.universe, -0.5, 1)


        # Create fuzzy rules
        rule1 = ctrl.Rule(input1['far'] & input3['long'], output1['egoistic'])
        rule2 = ctrl.Rule(input1['far'] & input3['short'], output1['cooperative'])
        rule3 = ctrl.Rule(input1['close'] & input2['long'] & input3['long'], output1['egoistic'])
        rule4 = ctrl.Rule(input1['close'] & input2['short'] & input3['short'], output1['individualistic'])
        rule5 = ctrl.Rule(input1['close'] & input2['long'] & input3['short'], output1['cooperative'])


        # Create fuzzy control system
        control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
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
