import numpy as np
import math
import matplotlib.pyplot as plt
import skfuzzy as fuzz
import skfuzzy.control as ctrl

from SVO_Fuzzy import SVO_Fuzzy


class Vehicle_SVO_Fuzzy(SVO_Fuzzy):
    def fuzzy_setup(self):
        # Create fuzzy variables
        input1 = ctrl.Antecedent(np.arange(0, 5, 0.1), 'following_distance')
        input2 = ctrl.Antecedent(np.arange(-100, 100, 0.1), 'merge_speed_change')
        input3 = ctrl.Antecedent(np.arange(0, 10, 0.1), 'lane_changes')
        input4 = ctrl.Antecedent(np.arange(0, 200, 0.1), 'speed_limit')
        input5 = ctrl.Antecedent(np.arange(0, 5, 0.1), 'smoothness')

        output1 = ctrl.Consequent(np.arange(0, 6, 0.1), 'svo_front')
        output2 = ctrl.Consequent(np.arange(0, 6, 0.1), 'svo_rear')
        output3 = ctrl.Consequent(np.arange(0, 3, 0.1), 'svo_lateral')

        # input membership functions
        input1['competitive'] = fuzz.zmf(input1.universe, 1, 3)
        input1['prosocial'] = fuzz.smf(input1.universe, 1.5, 4)

        input2['prosocial'] = fuzz.zmf(input2.universe, -30, 30)
        input2['competitive'] = fuzz.smf(input2.universe, -6.5, 45)

        input3['prosocial'] = fuzz.zmf(input3.universe, 0.4, 3)
        input3['competitive'] = fuzz.smf(input3.universe, 1, 5)

        input4['prosocial'] = fuzz.zmf(input4.universe, 80, 130)
        input4['competitive'] = fuzz.smf(input4.universe, 100, 160)

        input5['prosocial'] = fuzz.zmf(input5.universe, 0.1, 0.75)
        input5['competitive'] = fuzz.smf(input5.universe, 1, 5)

        # output memberships
        output1['low'] = fuzz.zmf(output1.universe, 0, 2)
        output1['high'] = fuzz.smf(output1.universe, 0.3, 2.5)

        output2['low'] = fuzz.zmf(output2.universe, 0.6, 2.5)
        output2['high'] = fuzz.smf(output2.universe, 1, 2.5)

        output3['low'] = fuzz.zmf(output3.universe, 0.25, 1.5)
        output3['high'] = fuzz.smf(output3.universe, 0.4, 1.5)

        # Create fuzzy rules
        rule1 = ctrl.Rule(input1['competitive'], output1['high'] & output2['high'])
        rule2 = ctrl.Rule(input2['competitive'], output1['high'] & output3['low'])
        rule3 = ctrl.Rule(input3['competitive'], output2['high'] & output3['high'])
        rule4 = ctrl.Rule(input4['competitive'], output1['high'] & output3['low'])
        rule5 = ctrl.Rule(input5['competitive'], output1['low'] & output2['low'] & output3['high'])  # TODO: revise
        rule6 = ctrl.Rule(input1['prosocial'], output1['low'] & output2['low'])
        rule7 = ctrl.Rule(input2['prosocial'], output2['low'])
        rule8 = ctrl.Rule(input3['prosocial'], output1['low'] & output2['low'] & output3['low'])  # TODO: revise
        rule9 = ctrl.Rule(input4['prosocial'], output1['low'] & output2['low'] & output3['low'])
        rule10 = ctrl.Rule(input5['prosocial'], output1['low'] & output2['low'] & output3['low'])

        # Create fuzzy control system
        control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10])
        fisSimulation = ctrl.ControlSystemSimulation(control_system)

        return fisSimulation

    def calculate_output(self, behavior_params):

        # Set inputs and compute output
        self.fuzzy_controller.input['following_distance'] = self.fuzzyInputs[0][0]
        self.fuzzy_controller.input['merge_speed_change'] = self.fuzzyInputs[0][1]
        self.fuzzy_controller.input['lane_changes'] = self.fuzzyInputs[0][0]
        self.fuzzy_controller.input['speed_limit'] = self.fuzzyInputs[0][0]
        self.fuzzy_controller.input['smoothness'] = self.fuzzyInputs[0][0]

        try:
            self.fuzzy_controller.compute()
        except:
            print("crisp output not possible")
            return 1, 1, 1

        svo_sigmas = {'svo_front': self.fuzzy_controller.output['svo_front'],
                      'svo_rear': self.fuzzy_controller.output['svo_rear'],
                      'svo_lateral': self.fuzzy_controller.output['svo_lateral']}

        return svo_sigmas

    def get_vehicle_svo(self, actor):

        # Set inputs and compute output
        self.fuzzy_controller.input['following_distance'] = self.fuzzyInputs[0][0]
        self.fuzzy_controller.input['merge_speed_change'] = self.fuzzyInputs[0][1]
        self.fuzzy_controller.input['lane_changes'] = self.fuzzyInputs[0][0]
        self.fuzzy_controller.input['speed_limit'] = self.fuzzyInputs[0][0]
        self.fuzzy_controller.input['smoothness'] = self.fuzzyInputs[0][0]

        try:
            self.fuzzy_controller.compute()
        except:
            print("crisp output not possible")
            return 1, 1, 1

        svo_front = self.fuzzy_controller.output['svo_front']
        svo_rear = self.fuzzy_controller.output['svo_rear']
        svo_lateral = self.fuzzy_controller.output['svo_lateral']

        return svo_front, svo_rear, svo_lateral

