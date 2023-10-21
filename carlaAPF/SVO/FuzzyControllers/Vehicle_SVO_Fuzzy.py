import numpy as np
import math
import matplotlib.pyplot as plt
import skfuzzy as fuzz
import skfuzzy.control as ctrl

from SVO.FuzzyControllers.SVO_Fuzzy import SVO_Fuzzy


class Vehicle_SVO_Fuzzy(SVO_Fuzzy):
    def fuzzy_setup(self):
        # Create fuzzy variables
        input1 = ctrl.Antecedent(np.arange(0, 8, 0.1), 'follow_time')
        input2 = ctrl.Antecedent(np.arange(0, 20, 0.1), 'lane_changes')
        input3 = ctrl.Antecedent(np.arange(0, 1.5, 0.1), 'lane_centering')
        input4 = ctrl.Antecedent(np.arange(40, 180, 0.1), 'speed_limit_percent')
        input5 = ctrl.Antecedent(np.arange(0, 10, 0.1), 'smoothness')

        output1 = ctrl.Consequent(np.arange(-1, 1, 0.1), 'svo')

        # input membership functions
        input1['short'] = fuzz.zmf(input1.universe, 0.75, 3.5)
        input1['long'] = fuzz.smf(input1.universe, 1, 4)

        input2['low'] = fuzz.zmf(input2.universe, 1, 10)
        input2['high'] = fuzz.smf(input2.universe, 2, 10)

        input3['good'] = fuzz.zmf(input3.universe, 0.1, 0.8)
        input3['poor'] = fuzz.smf(input3.universe, 0.3, 1)

        input4['low'] = fuzz.zmf(input4.universe, 80, 130)
        input4['high'] = fuzz.smf(input4.universe, 80, 150)

        input5['good'] = fuzz.zmf(input5.universe, 1, 3)
        input5['poor'] = fuzz.smf(input5.universe, 1, 4)

        # output memberships
        output1['cooperative'] = fuzz.zmf(output1.universe, -1, 0.5)
        output1['individualistic'] = fuzz.gaussmf(output1.universe, 0, 0.25)
        output1['egoistic'] = fuzz.smf(output1.universe, -0.5, 1)

        # Create fuzzy rules
        rule1 = ctrl.Rule(input1['short'] & input2['high'] & input3['poor'] & input4['high'] & input5['poor'], output1['egoistic'])
        rule2 = ctrl.Rule(input1['long'] & input2['low'] & input3['good'] & input4['low'] & input5['good'], output1['cooperative'])
        rule3 = ctrl.Rule(input1['short'] | input2['high'] | input3['poor'] | input4['high'] | input5['poor'], output1['individualistic'])

        # Create fuzzy control system
        control_system = ctrl.ControlSystem([rule1, rule2, rule3])
        fis_simulation = ctrl.ControlSystemSimulation(control_system)

        return fis_simulation

    def calculate_output(self, input_vector):

        # Set inputs and compute output
        self.fuzzy_controller.input['follow_time'] = input_vector['follow_time']
        self.fuzzy_controller.input['lane_changes'] = input_vector['lane_changes']
        self.fuzzy_controller.input['lane_centering'] = input_vector['lane_centering']
        self.fuzzy_controller.input['speed_limit_percent'] = input_vector['speed_limit_percent']
        self.fuzzy_controller.input['smoothness'] = input_vector['smoothness']

        try:
            self.fuzzy_controller.compute()
        except:
            print("crisp output not possible")
            return 0

        return self.fuzzy_controller.output['svo']
