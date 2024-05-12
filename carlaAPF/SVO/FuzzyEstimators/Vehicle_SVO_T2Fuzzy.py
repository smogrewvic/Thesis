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


class Vehicle_SVO_T2Fuzzy():
    def __init__(self):
        self.t2fuzzy_estimator = self.fuzzy_setup()

    def fuzzy_setup(self):
        # Create fuzzy variables
        follow_time = {}
        lane_changes = {}
        lane_centering = {}
        speed_limit_percent = {}

        svo = {}

        #closest to matlab
        follow_time['close'] = IT2FS(linspace(0., 6, 100), rtri_mf, [2.7, 0.7, 1.0], rtri_mf, [2.3, 0.3, 0.95], check_set=True)
        follow_time['medium'] = IT2FS(linspace(0., 6, 100), gaussian_mf, [2, 0.625, 1.0], gaussian_mf, [2, 0.375, 0.95], check_set=True)
        follow_time['far'] = IT2FS(linspace(0., 6, 100), ltri_mf, [1.25, 3.75, 1.0], ltri_mf, [1.75, 4.25, 0.95], check_set=True)

        # closest to matlab
        lane_changes['low'] = IT2FS(linspace(0., 5, 100), rtri_mf, [1.65, 0.35, 1.0], rtri_mf, [1.4, 0.1, 0.95], check_set=True)
        lane_changes['medium'] = IT2FS(linspace(0., 5, 100), gaussian_mf, [1.5, 0.65, 1.0], gaussian_mf, [1.5, 0.365, 0.95], check_set=True)
        lane_changes['high'] = IT2FS(linspace(0., 5, 100), ltri_mf, [1.35, 2.85, 1.0], ltri_mf, [1.65, 3.15, 0.95], check_set=True)

        # closest to matlab
        lane_centering['good'] = IT2FS(linspace(0., 2, 100), rtri_mf, [0.64, 0.24, 1.0], rtri_mf, [0.557, 0.157, 0.95], check_set=True)
        lane_centering['medium'] = IT2FS(linspace(0., 2, 100), gaussian_mf, [0.6, 0.19, 1.0], gaussian_mf, [0.6, 0.11, 0.95], check_set=True)
        lane_centering['poor'] = IT2FS(linspace(0., 2, 100), ltri_mf, [0.54, 1.14, 1.0], ltri_mf, [0.66, 1.26, 0.95], check_set=True)

        # closest to matlab
        speed_limit_percent['slow'] = IT2FS(linspace(0., 200, 100), rtri_mf, [128, 63, 1.0], rtri_mf, [122, 57, 1], check_set=True)
        speed_limit_percent['medium'] = IT2FS(linspace(0., 200, 100), gaussian_mf, [110, 11.5, 1.0], gaussian_mf, [110, 8.5, 0.95], check_set=True)
        speed_limit_percent['fast'] = IT2FS(linspace(0., 200, 100), ltri_mf, [97, 157, 1.0], ltri_mf, [103, 163, 1], check_set=True)

        # # output
        # closest to matlab
        svo['altruistic'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [-1, 0.25, 1.0], gaussian_mf, [-1, 0.15, 1.0], check_set=True)
        svo['cooperative'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [-0.5, 0.25, 1.0], gaussian_mf, [-0.5, 0.15, 1.0], check_set=True)
        svo['individualistic'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [0, 0.25, 1.0], gaussian_mf, [0, 0.15, 1.0], check_set=True)
        svo['competitive'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [0.5, 0.25, 1.0], gaussian_mf, [0.5, 0.15, 1.0], check_set=True)
        svo['sadistic'] = IT2FS(linspace(-1, 1, 100), gaussian_mf, [1, 0.25, 1.0], gaussian_mf, [1, 0.15, 1.0], check_set=True)

        # IT2FS_plot(follow_time['close'],follow_time['medium'], follow_time['far'], legends=["close", "medium", "far"], filename="simp_ex_sets")
        # IT2FS_plot(lane_changes['low'], lane_changes['medium'], lane_changes['high'], legends=["low", "medium", "high"], filename="simp_ex_sets")
        # IT2FS_plot(lane_centering['good'], lane_centering['medium'],lane_centering['poor'], legends=["good", "medium", "poor"], filename="simp_ex_sets")
        # IT2FS_plot(speed_limit_percent['slow'], speed_limit_percent['medium'], speed_limit_percent['fast'], legends=["slow", "medium", "fast"], filename="simp_ex_sets")
        # IT2FS_plot(svo['altruistic'], svo['cooperative'], svo['individualistic'],svo['competitive'],svo['sadistic'], legends=["altruistic", "cooperative", "individualistic", "competitive", "sadistic"], filename="simp_ex_sets")


        it2fis = IT2Mamdani(min_t_norm, max_s_norm, method="Centroid", algorithm="KM")
        it2fis.add_input_variable("follow_time")
        it2fis.add_input_variable("lane_changes")
        it2fis.add_input_variable("lane_centering")
        it2fis.add_input_variable("speed_limit_percent")
        it2fis.add_output_variable("svo")


        it2fis.add_rule(
            [("follow_time", follow_time['far']), ("lane_changes", lane_changes['low']), ("lane_centering", lane_centering['good']),("speed_limit_percent", speed_limit_percent['slow'])],
            [("svo", svo['altruistic'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['far']), ("lane_changes", lane_changes['low']), ("lane_centering", lane_centering['medium']),("speed_limit_percent", speed_limit_percent['medium'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['far']), ("lane_changes", lane_changes['medium']), ("lane_centering", lane_centering['good']),("speed_limit_percent", speed_limit_percent['medium'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['far']), ("lane_changes", lane_changes['medium']), ("lane_centering", lane_centering['medium']),("speed_limit_percent", speed_limit_percent['slow'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['medium']), ("lane_changes", lane_changes['low']), ("lane_centering", lane_centering['good']),("speed_limit_percent", speed_limit_percent['medium'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['medium']), ("lane_changes", lane_changes['low']), ("lane_centering", lane_centering['medium']),("speed_limit_percent", speed_limit_percent['slow'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['medium']), ("lane_changes", lane_changes['medium']), ("lane_centering", lane_centering['good']),("speed_limit_percent", speed_limit_percent['slow'])],
            [("svo", svo['cooperative'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['medium']), ("lane_changes", lane_changes['medium']), ("lane_centering", lane_centering['medium']),("speed_limit_percent", speed_limit_percent['medium'])],
            [("svo", svo['individualistic'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['close']), ("lane_changes", lane_changes['high']), ("lane_centering", lane_centering['medium']),("speed_limit_percent", speed_limit_percent['medium'])],
            [("svo", svo['competitive'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['close']), ("lane_changes", lane_changes['medium']), ("lane_centering", lane_centering['poor']),("speed_limit_percent", speed_limit_percent['medium'])],
            [("svo", svo['competitive'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['close']), ("lane_changes", lane_changes['medium']), ("lane_centering", lane_centering['medium']),("speed_limit_percent", speed_limit_percent['fast'])],
            [("svo", svo['competitive'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['medium']), ("lane_changes", lane_changes['high']), ("lane_centering", lane_centering['poor']),("speed_limit_percent", speed_limit_percent['medium'])],
            [("svo", svo['competitive'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['medium']), ("lane_changes", lane_changes['high']), ("lane_centering", lane_centering['medium']),("speed_limit_percent", speed_limit_percent['fast'])],
            [("svo", svo['competitive'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['medium']), ("lane_changes", lane_changes['medium']), ("lane_centering", lane_centering['poor']),("speed_limit_percent", speed_limit_percent['fast'])],
            [("svo", svo['competitive'])]
        )
        it2fis.add_rule(
            [("follow_time", follow_time['close']), ("lane_changes", lane_changes['high']), ("lane_centering", lane_centering['poor']),("speed_limit_percent", speed_limit_percent['fast'])],
            [("svo", svo['sadistic'])]
        )

        return it2fis
    def calculate_output(self, input_vector):
        it2out, tr = self.t2fuzzy_estimator.evaluate({"follow_time":input_vector['follow_time'],
                                                      "lane_changes": input_vector['lane_changes'],
                                                      "lane_centering": input_vector['lane_centering'],
                                                      "speed_limit_percent": input_vector['speed_limit_percent']
                                                      })

        return crisp(tr["svo"])

