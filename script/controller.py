import numpy as np
import skfuzzy as fuzz
from math import atan
from skfuzzy import control as ctrl
from tools import constraint


class FuzzyController:
    def __init__(self, speed_min, speed_max, r_min, r_mid, r_max, dr_max):
        self.r = None
        self.r_min = r_min
        self.r_max = r_max
        self.dr_max = dr_max

        x_r = np.arange(r_min, r_max + 0.1, 0.1)
        x_dr = np.arange(-dr_max, dr_max + 0.1, 0.1)
        x_speed = np.arange(speed_min, speed_max + 1, 1)

        r = ctrl.Antecedent(x_r, 'radius')
        dr = ctrl.Antecedent(x_dr, 'radius change')
        speed = ctrl.Consequent(x_speed, 'speed')
        # membership functions
        r['S'] = fuzz.trimf(x_r, [r_min, r_min, r_mid])
        r['M'] = fuzz.trimf(x_r, [r_min, r_mid, 2 * r_mid - r_min])
        r['L'] = fuzz.trapmf(x_r, [r_mid, 2 * r_mid - r_min, r_max, r_max])
        dr['D'] = fuzz.trimf(x_dr, [-dr_max, -dr_max, 0])
        dr['H'] = fuzz.trimf(x_dr, [-dr_max, 0, dr_max])
        dr['I'] = fuzz.trimf(x_dr, [0, dr_max, dr_max])
        speed['L'] = fuzz.trimf(x_speed, [speed_min, speed_min, (speed_min + speed_max) / 2])
        speed['M'] = fuzz.trimf(x_speed, [speed_min, (speed_min + speed_max) / 2, speed_max])
        speed['H'] = fuzz.trimf(x_speed, [(speed_min + speed_max) / 2, speed_max, speed_max])
        # rules
        rule1 = ctrl.Rule(
            antecedent=(
                    (r['S'] & dr['D']) |
                    (r['S'] & dr['H']) |
                    (r['M'] & dr['D'])
            ),
            consequent=speed['L'], label='L'
        )
        rule2 = ctrl.Rule(
            antecedent=(
                    (r['S'] & dr['I']) |
                    (r['M'] & dr['H']) |
                    (r['L'] & dr['D'])
            ),
            consequent=speed['M'], label='M'
        )
        rule3 = ctrl.Rule(
            antecedent=(
                    (r['M'] & dr['I']) |
                    (r['L'] & dr['H']) |
                    (r['L'] & dr['I'])
            ),
            consequent=speed['H'], label='H'
        )
        control_system = ctrl.ControlSystem([rule1, rule2, rule3])
        self.fuzzy_controller = ctrl.ControlSystemSimulation(control_system)

    def output(self, r):
        dr = 0 if self.r is None else (r - self.r)
        self.r = r
        self.fuzzy_controller.input['radius'] = constraint(r, self.r_min, self.r_max)
        self.fuzzy_controller.input['radius change'] = constraint(dr, -self.dr_max, self.dr_max)
        self.fuzzy_controller.compute()
        return self.fuzzy_controller.output['speed']


class PurePursuitController:
    def __init__(self, kp, length, steer_min, steer_max):
        self.kp = kp
        self.length = length
        self.steer_max = steer_max
        self.steer_min = steer_min

    def output(self, error, preview_distance):
        steer = 95 - atan(self.kp * 2 * self.length * error / preview_distance ** 2) * 180 / np.pi
        return constraint(steer, self.steer_min, self.steer_max)
