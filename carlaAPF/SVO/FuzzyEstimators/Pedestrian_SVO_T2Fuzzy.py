import numpy as np
import math
import matplotlib.pyplot as plt
import skfuzzy as fuzz
import skfuzzy.control as ctrl

from pyit2fls import IT2FS, tri_mf, const_mf, rtri_mf, ltri_mf,\
    lgauss_uncert_std_umf, lgauss_uncert_std_lmf, rgauss_uncert_std_umf, rgauss_uncert_std_lmf,\
    trapezoid_mf, gaussian_mf, IT2FS_Gaussian_UncertMean, \
    IT2FS_Gaussian_UncertStd, R_IT2FS_Gaussian_UncertStd, \
    L_IT2FS_Gaussian_UncertStd, IT2FS_plot, IT2Mamdani, min_t_norm, max_s_norm, crisp

from numpy import linspace


class Pedestrian_SVO_T2Fuzzy():
    def __init__(self):
        self.t2fuzzy_estimator = self.fuzzy_setup()

    def fuzzy_setup(self):
        # Create fuzzy variables
        distance_to_crosswalk = {}
        wait_time = {}
        look_time = {}
        svo = {}

        distance_to_crosswalk['close'] = IT2FS(linspace(0., 10, 100), rtri_mf, [6, 1.5, 1.0], rtri_mf, [6, 1.5, 0.8], check_set=True)
        distance_to_crosswalk['far'] = IT2FS(linspace(0., 10, 100), ltri_mf, [1.5, 6, 1.0], ltri_mf, [1.5, 6, 0.8], check_set=True)

        wait_time['short'] = IT2FS(linspace(0., 10, 100), rtri_mf, [5, 0.5, 1.0], rtri_mf, [5, 0.5, 0.8], check_set=True)
        wait_time['medium'] = IT2FS(linspace(0., 10, 100), gaussian_mf, [3.5, 1, 1.0], gaussian_mf, [3.5, 1, 0.8], check_set=True)
        wait_time['long'] = IT2FS(linspace(0., 10, 100), ltri_mf, [1, 7, 1.0], ltri_mf, [1, 7, 0.8], check_set=True)

        look_time['short'] = IT2FS(linspace(0., 10, 100), rtri_mf, [6, 1, 1.0], rtri_mf, [6, 1, 0.8], check_set=True)
        look_time['medium'] = IT2FS(linspace(0., 10, 100), gaussian_mf, [4, 1, 1.0], gaussian_mf, [4, 1, 0.8], check_set=True)
        look_time['long'] = IT2FS(linspace(0., 10, 100), ltri_mf, [1.5, 7, 1.0], ltri_mf, [1.5, 7, 0.8], check_set=True)

        # # output
        svo['altruistic'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [-1, 0.25, 1.0], gaussian_mf, [-1, 0.2, 1.0], check_set=True)
        svo['cooperative'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [-0.5, 0.25, 1.0], gaussian_mf, [-0.5, 0.2, 1.0], check_set=True)
        svo['individualistic'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [0, 0.25, 1.0], gaussian_mf, [0, 0.2, 1.0], check_set=True)
        svo['competitive'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [0.5, 0.25, 1.0], gaussian_mf, [0.5, 0.2, 1.0], check_set=True)
        svo['sadistic'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [1, 0.25, 1.0], gaussian_mf, [1, 0.2, 1.0], check_set=True)

        IT2FS_plot(distance_to_crosswalk['close'], distance_to_crosswalk['far'], legends=["close", "far"], filename="simp_ex_sets")
        IT2FS_plot(wait_time['short'], wait_time['medium'],wait_time['long'], legends=["short", "medium", "long"], filename="simp_ex_sets")
        IT2FS_plot(look_time['short'], look_time['medium'], look_time['long'], legends=["short", "medium", "long"], filename="simp_ex_sets")
        IT2FS_plot(svo['altruistic'], svo['cooperative'], svo['individualistic'],svo['competitive'],svo['sadistic'], legends=["altruistic", "cooperative", "individualistic", "competitive", "sadistic"], filename="simp_ex_sets")


        it2fis = IT2Mamdani(min_t_norm, max_s_norm, method="Centroid", algorithm="KM")
        it2fis.add_input_variable("distance_to_crosswalk")
        it2fis.add_input_variable("wait_time")
        it2fis.add_input_variable("wait_time")
        it2fis.add_output_variable("svo")


        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['close']), ("wait_time", wait_time['short']), ("look_time", look_time['short'])],
            [("svo", svo['sadistic'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['close']), ("wait_time", wait_time['medium']), ("look_time", look_time['short'])],
            [("svo", svo['competitive'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['close']), ("wait_time", wait_time['long']), ("look_time", look_time['short'])],
            [("svo", svo['individualistic'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['close']), ("wait_time", wait_time['long']), ("look_time", look_time['medium'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['close']), ("wait_time", wait_time['long']), ("look_time", look_time['long'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['far']), ("wait_time", wait_time['short']), ("look_time", look_time['short'])],
            [("svo", svo['individualistic'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['far']), ("wait_time", wait_time['medium']), ("look_time", look_time['short'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['far']), ("wait_time", wait_time['medium']), ("look_time", look_time['medium'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['far']), ("wait_time", wait_time['long']), ("look_time", look_time['medium'])],
            [("svo", svo['altruistic'])]
        )
        it2fis.add_rule(
            [("distance_to_crosswalk", distance_to_crosswalk['far']), ("wait_time", wait_time['long']), ("look_time", look_time['long'])],
            [("svo", svo['altruistic'])]
        )


        return it2fis
    def calculate_output(self, input_vector):
        it2out, tr = self.t2fuzzy_estimator.evaluate({"distance_to_crosswalk":input_vector['distance_to_crosswalk'],
                                                      "wait_time": input_vector['time_waiting'],
                                                      "look_time":input_vector['time_looking']
                                                      })

        return crisp(tr["svo"])
        # try:
        #     self.fuzzy_estimator.compute()
        # except:
        #     print("crisp output not possible")
        #     return 0
        #
        # return self.fuzzy_estimator.output['svo']




