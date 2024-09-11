#! /usr/bin/env python

from z3 import *
import sys
root = '/home/qian/catkin_ws/src/stretch_controller/'
sys.path.insert(0, f'{root}scripts')
from tools import print_debug

class Monitor:
    def __init__(self) -> None:
        self.s = Solver()

    def change_Xp_names(self, X_p: dict) -> dict:
        new_X_p = {}
        for key, val in X_p.items():
            new_X_p[f'{key}_p'] = val
        return new_X_p

    def no_violation(self, results: list):
        """Process the result
        
        Returns:
            True if all elts in results are True
        """
        return all(results)

    def filter_violation(self, results: list):
        """Given that there exists violation in results
        Return a list of indices indicating the violated assumptions   
        """
        violated_indices = []
        for i in range(len(results)):
            if not results[i]:
                violated_indices.append(i)
        return violated_indices
        
    def sat_to_bool(self, result) -> bool:
        return str(result) == 'sat' 

    def monitor_postconditions(self, X: dict, Y: dict, X_p: dict) -> list:
        """Produce the postconditions expression for z3
        
        Args:
            inputs: list of str of inputs
            skills: list of str of skills

        Returns:
            results: list of bool indicating whether each postcondition is true or not
        """
        X_p = self.change_Xp_names(X_p)  
        x0 = Bool('x0')
        x0_p = Bool('x0_p')
        x1 = Bool('x1')
        x1_p = Bool('x1_p')
        x2 = Bool('x2')
        x2_p = Bool('x2_p')
        y0 = Bool('y0')
        y0_p = Bool('y0_p')
        y1 = Bool('y1')
        y1_p = Bool('y1_p')
        y2 = Bool('y2')
        y2_p = Bool('y2_p')
        xrequest0 = Bool('xrequest0')
        xrequest0_p = Bool('xrequest0_p')
        xrequest1 = Bool('xrequest1')
        xrequest1_p = Bool('xrequest1_p')
        xrequest2 = Bool('xrequest2')
        xrequest2_p = Bool('xrequest2_p')
        yrequest0 = Bool('yrequest0')
        yrequest0_p = Bool('yrequest0_p')
        yrequest1 = Bool('yrequest1')
        yrequest1_p = Bool('yrequest1_p')
        yrequest2 = Bool('yrequest2')
        yrequest2_p = Bool('yrequest2_p')
        x_0_y_0_terrain0 = Bool('x_0_y_0_terrain0')
        x_0_y_0_terrain0_p = Bool('x_0_y_0_terrain0_p')
        x_0_y_0_terrain1 = Bool('x_0_y_0_terrain1')
        x_0_y_0_terrain1_p = Bool('x_0_y_0_terrain1_p')
        x_0_y_1_terrain0 = Bool('x_0_y_1_terrain0')
        x_0_y_1_terrain0_p = Bool('x_0_y_1_terrain0_p')
        x_0_y_1_terrain1 = Bool('x_0_y_1_terrain1')
        x_0_y_1_terrain1_p = Bool('x_0_y_1_terrain1_p')
        x_0_y_2_terrain0 = Bool('x_0_y_2_terrain0')
        x_0_y_2_terrain0_p = Bool('x_0_y_2_terrain0_p')
        x_0_y_2_terrain1 = Bool('x_0_y_2_terrain1')
        x_0_y_2_terrain1_p = Bool('x_0_y_2_terrain1_p')
        x_1_y_0_terrain0 = Bool('x_1_y_0_terrain0')
        x_1_y_0_terrain0_p = Bool('x_1_y_0_terrain0_p')
        x_1_y_0_terrain1 = Bool('x_1_y_0_terrain1')
        x_1_y_0_terrain1_p = Bool('x_1_y_0_terrain1_p')
        x_1_y_1_terrain0 = Bool('x_1_y_1_terrain0')
        x_1_y_1_terrain0_p = Bool('x_1_y_1_terrain0_p')
        x_1_y_1_terrain1 = Bool('x_1_y_1_terrain1')
        x_1_y_1_terrain1_p = Bool('x_1_y_1_terrain1_p')
        x_1_y_2_terrain0 = Bool('x_1_y_2_terrain0')
        x_1_y_2_terrain0_p = Bool('x_1_y_2_terrain0_p')
        x_1_y_2_terrain1 = Bool('x_1_y_2_terrain1')
        x_1_y_2_terrain1_p = Bool('x_1_y_2_terrain1_p')
        x_2_y_0_terrain0 = Bool('x_2_y_0_terrain0')
        x_2_y_0_terrain0_p = Bool('x_2_y_0_terrain0_p')
        x_2_y_0_terrain1 = Bool('x_2_y_0_terrain1')
        x_2_y_0_terrain1_p = Bool('x_2_y_0_terrain1_p')
        x_2_y_1_terrain0 = Bool('x_2_y_1_terrain0')
        x_2_y_1_terrain0_p = Bool('x_2_y_1_terrain0_p')
        x_2_y_1_terrain1 = Bool('x_2_y_1_terrain1')
        x_2_y_1_terrain1_p = Bool('x_2_y_1_terrain1_p')
        x_2_y_2_terrain0 = Bool('x_2_y_2_terrain0')
        x_2_y_2_terrain0_p = Bool('x_2_y_2_terrain0_p')
        x_2_y_2_terrain1 = Bool('x_2_y_2_terrain1')
        x_2_y_2_terrain1_p = Bool('x_2_y_2_terrain1_p')
        skill_0 = Bool('skill_0')
        skill_1 = Bool('skill_1')
        skill_2 = Bool('skill_2')
        skill_3 = Bool('skill_3')
        skill_4 = Bool('skill_4')
        skill_5 = Bool('skill_5')
        skill_6 = Bool('skill_6')
        skill_7 = Bool('skill_7')
        skill_8 = Bool('skill_8')
        skill_9 = Bool('skill_9')
        skill_10 = Bool('skill_10')
        skill_11 = Bool('skill_11')
        skill_12 = Bool('skill_12')
        skill_13 = Bool('skill_13')
        skill_14 = Bool('skill_14')
        skill_15 = Bool('skill_15')
        skill_16 = Bool('skill_16')
        skill_17 = Bool('skill_17')
        skill_18 = Bool('skill_18')
        skill_19 = Bool('skill_19')
        skill_20 = Bool('skill_20')
        skill_21 = Bool('skill_21')
        skill_22 = Bool('skill_22')
        skill_23 = Bool('skill_23')
        skill_24 = Bool('skill_24')
        skill_25 = Bool('skill_25')
        skill_26 = Bool('skill_26')
        skill_27 = Bool('skill_27')
        skill_28 = Bool('skill_28')
        skill_29 = Bool('skill_29')
        skill_30 = Bool('skill_30')
        skill_31 = Bool('skill_31')
        skill_32 = Bool('skill_32')
        skill_33 = Bool('skill_33')
        skill_34 = Bool('skill_34')
        skill_35 = Bool('skill_35')
        skill_36 = Bool('skill_36')
        skill_37 = Bool('skill_37')
        skill_38 = Bool('skill_38')
        skill_39 = Bool('skill_39')
        skill_40 = Bool('skill_40')
        skill_41 = Bool('skill_41')
        skill_42 = Bool('skill_42')
        skill_43 = Bool('skill_43')
        skill_44 = Bool('skill_44')
        skill_45 = Bool('skill_45')
        skill_46 = Bool('skill_46')
        skill_47 = Bool('skill_47')
        skill_48 = Bool('skill_48')
        skill_49 = Bool('skill_49')
        skill_50 = Bool('skill_50')
        skill_51 = Bool('skill_51')
        skill_52 = Bool('skill_52')
        skill_53 = Bool('skill_53')
        skill_54 = Bool('skill_54')
        skill_55 = Bool('skill_55')
        skill_56 = Bool('skill_56')
        skill_57 = Bool('skill_57')
        skill_58 = Bool('skill_58')
        skill_59 = Bool('skill_59')
        skill_60 = Bool('skill_60')
        skill_61 = Bool('skill_61')
        skill_62 = Bool('skill_62')
        skill_63 = Bool('skill_63')
        skill_64 = Bool('skill_64')
        skill_65 = Bool('skill_65')
        skill_66 = Bool('skill_66')
        skill_67 = Bool('skill_67')
        skill_68 = Bool('skill_68')
        skill_69 = Bool('skill_69')
        skill_70 = Bool('skill_70')
        skill_71 = Bool('skill_71')
        skill_72 = Bool('skill_72')
        skill_73 = Bool('skill_73')
        skill_74 = Bool('skill_74')
        skill_75 = Bool('skill_75')
        skill_76 = Bool('skill_76')
        skill_77 = Bool('skill_77')
        skill_78 = Bool('skill_78')
        skill_79 = Bool('skill_79')
        skill_80 = Bool('skill_80')
        skill_81 = Bool('skill_81')
        skill_82 = Bool('skill_82')
        skill_83 = Bool('skill_83')
        self.formulas = []
        self.formulas.append(Or(And(x_0_y_0_terrain0, Not(x_0_y_0_terrain1), x_0_y_1_terrain0, Not(x_0_y_1_terrain1), x_0_y_2_terrain0, Not(x_0_y_2_terrain1), x_1_y_0_terrain0, Not(x_1_y_0_terrain1), x_1_y_1_terrain0, Not(x_1_y_1_terrain1), x_1_y_2_terrain0, Not(x_1_y_2_terrain1), x_2_y_0_terrain0, Not(x_2_y_0_terrain1), x_2_y_1_terrain0, Not(x_2_y_1_terrain1), x_2_y_2_terrain0, Not(x_2_y_2_terrain1)), And(Not(x_0_y_0_terrain0), x_0_y_0_terrain1, Not(x_0_y_1_terrain0), x_0_y_1_terrain1, Not(x_0_y_2_terrain0), x_0_y_2_terrain1, x_1_y_0_terrain0, Not(x_1_y_0_terrain1), x_1_y_1_terrain0, Not(x_1_y_1_terrain1), x_1_y_2_terrain0, Not(x_1_y_2_terrain1), Not(x_2_y_0_terrain0), x_2_y_0_terrain1, Not(x_2_y_1_terrain0), x_2_y_1_terrain1, Not(x_2_y_2_terrain0), x_2_y_2_terrain1), And(x_0_y_0_terrain0, Not(x_0_y_0_terrain1), x_0_y_1_terrain0, Not(x_0_y_1_terrain1), x_0_y_2_terrain0, Not(x_0_y_2_terrain1), Not(x_1_y_0_terrain0), x_1_y_0_terrain1, Not(x_1_y_1_terrain0), x_1_y_1_terrain1, Not(x_1_y_2_terrain0), x_1_y_2_terrain1, x_2_y_0_terrain0, Not(x_2_y_0_terrain1), x_2_y_1_terrain0, Not(x_2_y_1_terrain1), x_2_y_2_terrain0, Not(x_2_y_2_terrain1)), And(Not(x_0_y_0_terrain0), x_0_y_0_terrain1, Not(x_0_y_1_terrain0), x_0_y_1_terrain1, Not(x_0_y_2_terrain0), x_0_y_2_terrain1, Not(x_1_y_0_terrain0), x_1_y_0_terrain1, Not(x_1_y_1_terrain0), x_1_y_1_terrain1, Not(x_1_y_2_terrain0), x_1_y_2_terrain1, Not(x_2_y_0_terrain0), x_2_y_0_terrain1, Not(x_2_y_1_terrain0), x_2_y_1_terrain1, Not(x_2_y_2_terrain0), x_2_y_2_terrain1)))
        results = []
        for formula in self.formulas:
            self.s.push()
            self.s.add(formula)
            for env in [X, Y, X_p]:
                for name, value in env.items():
                    exec(f"self.s.add({name} == {str(value)})")
            print_debug(f"Solver for post {formula}")
            print_debug(self.s)
            results.append(self.sat_to_bool(self.s.check()))
            self.s.pop()
        return results
    def monitor_curr_input_state(self, input_state: dict) -> list:
        x0 = Bool('x0')
        x1 = Bool('x1')
        x2 = Bool('x2')
        y0 = Bool('y0')
        y1 = Bool('y1')
        y2 = Bool('y2')
        xrequest0 = Bool('xrequest0')
        xrequest1 = Bool('xrequest1')
        xrequest2 = Bool('xrequest2')
        yrequest0 = Bool('yrequest0')
        yrequest1 = Bool('yrequest1')
        yrequest2 = Bool('yrequest2')
        x_0_y_0_terrain0 = Bool('x_0_y_0_terrain0')
        x_0_y_0_terrain1 = Bool('x_0_y_0_terrain1')
        x_0_y_1_terrain0 = Bool('x_0_y_1_terrain0')
        x_0_y_1_terrain1 = Bool('x_0_y_1_terrain1')
        x_0_y_2_terrain0 = Bool('x_0_y_2_terrain0')
        x_0_y_2_terrain1 = Bool('x_0_y_2_terrain1')
        x_1_y_0_terrain0 = Bool('x_1_y_0_terrain0')
        x_1_y_0_terrain1 = Bool('x_1_y_0_terrain1')
        x_1_y_1_terrain0 = Bool('x_1_y_1_terrain0')
        x_1_y_1_terrain1 = Bool('x_1_y_1_terrain1')
        x_1_y_2_terrain0 = Bool('x_1_y_2_terrain0')
        x_1_y_2_terrain1 = Bool('x_1_y_2_terrain1')
        x_2_y_0_terrain0 = Bool('x_2_y_0_terrain0')
        x_2_y_0_terrain1 = Bool('x_2_y_0_terrain1')
        x_2_y_1_terrain0 = Bool('x_2_y_1_terrain0')
        x_2_y_1_terrain1 = Bool('x_2_y_1_terrain1')
        x_2_y_2_terrain0 = Bool('x_2_y_2_terrain0')
        x_2_y_2_terrain1 = Bool('x_2_y_2_terrain1')
        self.formulas = []
        self.formulas.append(Or
                             (And(x_0_y_0_terrain0, Not(x_0_y_0_terrain1), x_0_y_1_terrain0, Not(x_0_y_1_terrain1), x_0_y_2_terrain0, Not(x_0_y_2_terrain1), x_1_y_0_terrain0, Not(x_1_y_0_terrain1), x_1_y_1_terrain0, Not(x_1_y_1_terrain1), x_1_y_2_terrain0, Not(x_1_y_2_terrain1), x_2_y_0_terrain0, Not(x_2_y_0_terrain1), x_2_y_1_terrain0, Not(x_2_y_1_terrain1), x_2_y_2_terrain0, Not(x_2_y_2_terrain1)), 
                              And(Not(x_0_y_0_terrain0), x_0_y_0_terrain1, Not(x_0_y_1_terrain0), x_0_y_1_terrain1, Not(x_0_y_2_terrain0), x_0_y_2_terrain1, x_1_y_0_terrain0, Not(x_1_y_0_terrain1), x_1_y_1_terrain0, Not(x_1_y_1_terrain1), x_1_y_2_terrain0, Not(x_1_y_2_terrain1), Not(x_2_y_0_terrain0), x_2_y_0_terrain1, Not(x_2_y_1_terrain0), x_2_y_1_terrain1, Not(x_2_y_2_terrain0), x_2_y_2_terrain1), 
                              And(x_0_y_0_terrain0, Not(x_0_y_0_terrain1), x_0_y_1_terrain0, Not(x_0_y_1_terrain1), x_0_y_2_terrain0, Not(x_0_y_2_terrain1), Not(x_1_y_0_terrain0), x_1_y_0_terrain1, Not(x_1_y_1_terrain0), x_1_y_1_terrain1, Not(x_1_y_2_terrain0), x_1_y_2_terrain1, x_2_y_0_terrain0, Not(x_2_y_0_terrain1), x_2_y_1_terrain0, Not(x_2_y_1_terrain1), x_2_y_2_terrain0, Not(x_2_y_2_terrain1)), 
                              And(Not(x_0_y_0_terrain0), x_0_y_0_terrain1, Not(x_0_y_1_terrain0), x_0_y_1_terrain1, Not(x_0_y_2_terrain0), x_0_y_2_terrain1, Not(x_1_y_0_terrain0), x_1_y_0_terrain1, Not(x_1_y_1_terrain0), x_1_y_1_terrain1, Not(x_1_y_2_terrain0), x_1_y_2_terrain1, Not(x_2_y_0_terrain0), x_2_y_0_terrain1, Not(x_2_y_1_terrain0), x_2_y_1_terrain1, Not(x_2_y_2_terrain0), x_2_y_2_terrain1)))
        results = []
        for formula in self.formulas:
            self.s.push()
            self.s.add(formula)
            for name, value in input_state.items():
                exec(f"self.s.add({name} == {str(value)})")
            print_debug(f"Solver for post {formula}")
            print_debug(self.s)
            results.append(self.sat_to_bool(self.s.check()))
            self.s.pop()
        return results

if __name__ == '__main__':
    print("Testing monitor")
    monitor = Monitor()
    terrain_state = {
        "x_0_y_0_terrain0": True,
        "x_0_y_0_terrain1": False,
        "x_0_y_1_terrain0": True,
        "x_0_y_1_terrain1": False,
        "x_0_y_2_terrain0": True,
        "x_0_y_2_terrain1": False,
        "x_1_y_0_terrain0": True,
        "x_1_y_0_terrain1": False,
        "x_1_y_1_terrain0": True,
        "x_1_y_1_terrain1": False,
        "x_1_y_2_terrain0": True,
        "x_1_y_2_terrain1": False,
        "x_2_y_0_terrain0": True,
        "x_2_y_0_terrain1": False,
        "x_2_y_1_terrain0": True,
        "x_2_y_1_terrain1": False,
        "x_2_y_2_terrain0": True,
        "x_2_y_2_terrain1": False
    }
    results = monitor.monitor_curr_input_state(terrain_state)
    print(results)
    print(monitor.no_violation(results))