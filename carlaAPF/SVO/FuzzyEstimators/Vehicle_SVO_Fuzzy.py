import numpy as np
import math
import matplotlib.pyplot as plt
import skfuzzy as fuzz
import skfuzzy.control as ctrl



class Vehicle_SVO_Fuzzy():
    def __init__(self):
        self.fuzzy_estimator = self.fuzzy_setup()

    def fuzzy_setup(self):
        # Create fuzzy variables
        input1 = ctrl.Antecedent(np.arange(0, 6.01, 0.01), 'follow_time')  # +0.01 to end value to be inclusive of final value
        input2 = ctrl.Antecedent(np.arange(0, 5.01, 0.01), 'lane_changes')
        input3 = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'lane_centering')
        input4 = ctrl.Antecedent(np.arange(0, 200.01, 0.01), 'speed_limit_percent')
        # input5 = ctrl.Antecedent(np.arange(0, 10, 0.1), 'smoothness')  # NOT USED

        output1 = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'svo')

        # input membership functions
        input1['close'] = fuzz.zmf(input1.universe, 0.5, 2.5)
        input1['medium'] = fuzz.gaussmf(input1.universe, 2, 0.5)
        input1['far'] = fuzz.smf(input1.universe, 1.5, 4)

        input2['low'] = fuzz.zmf(input2.universe, 0.25, 1.5)
        input2['medium'] = fuzz.gaussmf(input2.universe, 1.5, 0.5)
        input2['high'] = fuzz.smf(input2.universe, 1.5, 3)

        input3['good'] = fuzz.zmf(input3.universe, 0.2, 0.6)
        input3['medium'] = fuzz.gaussmf(input3.universe, 0.6, 0.15)
        input3['poor'] = fuzz.smf(input3.universe, 0.6, 1.2)

        input4['slow'] = fuzz.zmf(input4.universe, 60, 125)
        input4['medium'] = fuzz.gaussmf(input4.universe, 110, 10)
        input4['fast'] = fuzz.smf(input4.universe, 100, 160)

        # input5['good'] = fuzz.zmf(input5.universe, 0.25, 1)  # NOT USED
        # input5['medium'] = fuzz.gaussmf(input5.universe, 0.75, 1)  # NOT USED
        # input5['poor'] = fuzz.smf(input5.universe, 1, 3)  # NOT USED

        # output memberships
        output1['altruistic'] = fuzz.gaussmf(output1.universe, -1, 0.25)
        output1['cooperative'] = fuzz.gaussmf(output1.universe, -0.5, 0.25)
        output1['individualistic'] = fuzz.gaussmf(output1.universe, 0, 0.25)
        output1['egoistic'] = fuzz.gaussmf(output1.universe, 0.5, 0.25)
        output1['sadistic'] = fuzz.gaussmf(output1.universe, 1, 0.25)

        # Create fuzzy rules
        rule1 = ctrl.Rule(input1['far'] & input2['low'] & input3['good'] & input4['slow'], output1['altruistic'])
        rule2 = ctrl.Rule(input1['far'] & input2['low'] & input3['medium'] & input4['medium'], output1['cooperative'])
        rule3 = ctrl.Rule(input1['far'] & input2['medium'] & input3['good'] & input4['medium'], output1['cooperative'])
        rule4 = ctrl.Rule(input1['far'] & input2['medium'] & input3['medium'] & input4['slow'], output1['cooperative'])
        rule5 = ctrl.Rule(input1['medium'] & input2['low'] & input3['good'] & input4['medium'], output1['cooperative'])
        rule6 = ctrl.Rule(input1['medium'] & input2['low'] & input3['medium'] & input4['slow'], output1['cooperative'])
        rule7 = ctrl.Rule(input1['medium'] & input2['medium'] & input3['good'] & input4['slow'], output1['cooperative'])
        rule8 = ctrl.Rule(input1['medium'] & input2['medium'] & input3['medium'] & input4['medium'], output1['individualistic'])
        rule9 = ctrl.Rule(input1['close'] & input2['high'] & input3['medium'] & input4['medium'], output1['egoistic'])
        rule10 = ctrl.Rule(input1['close'] & input2['medium'] & input3['poor'] & input4['medium'], output1['egoistic'])
        rule11 = ctrl.Rule(input1['close'] & input2['medium'] & input3['medium'] & input4['fast'], output1['egoistic'])
        rule12 = ctrl.Rule(input1['medium'] & input2['high'] & input3['poor'] & input4['medium'], output1['egoistic'])
        rule13 = ctrl.Rule(input1['medium'] & input2['high'] & input3['medium'] & input4['fast'], output1['egoistic'])
        rule14 = ctrl.Rule(input1['medium'] & input2['medium'] & input3['poor'] & input4['fast'], output1['egoistic'])
        rule15 = ctrl.Rule(input1['close'] & input2['high'] & input3['poor'] & input4['fast'], output1['sadistic'])

        # Create fuzzy control system
        control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8,
                                             rule9, rule10, rule11, rule12, rule13, rule14, rule15])
        fis_simulation = ctrl.ControlSystemSimulation(control_system)

        return fis_simulation

    def calculate_output(self, input_vector):

        # Set inputs and compute output
        self.fuzzy_estimator.input['follow_time'] = input_vector['follow_time']
        self.fuzzy_estimator.input['lane_changes'] = input_vector['lane_changes']
        self.fuzzy_estimator.input['lane_centering'] = input_vector['lane_centering']
        self.fuzzy_estimator.input['speed_limit_percent'] = input_vector['speed_limit_percent']
        # self.fuzzy_estimator.input['smoothness'] = input_vector['smoothness']

        try:
            self.fuzzy_estimator.compute()
        except:
            print("crisp output not possible")
            return 0

        return self.fuzzy_estimator.output['svo']
