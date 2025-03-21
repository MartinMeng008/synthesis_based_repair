#! /usr/bin/env python
import sys
import argparse
import copy
import time
from collections import defaultdict, deque
from tools import (
    clear_file,
    load_skills_from_json,
    check_slugsin_realizability,
    find_symbols_by_objects_type_category,
    json_load_wrapper,
    create_symbols_from_objects_and_locations,
    find_true_symbols,
    is_manipulation_object,
    is_mobile_object,
    find_controllable_symbols,
    find_uncontrollable_symbols,
    find_controllable_mobile_symbols,
    find_controllable_manipulation_symbols,
    load_list_values_from_json,
    state2dict_bool,
    check_slugsin_realizability_with_readable_strategy,
    list2dict_bool_true,
    get_skill_type,
    generate_counterstrategy,
    run_slugs,
    )
from grounding import Groundings
# sys.path.insert(0, '/home/qian/workspace/slugs/tools/StructuredSlugsParser')
sys.path.insert(0, '/home/jnl77/multi_ws/src/stretch_controller/slugs/tools/StructuredSlugsParser')
from compiler import (
    get_asts, 
    asts_to_slugsin, 
    get_asts_from_structuredslugsplus, 
    isValidRecursiveSlugsProperty, 
    parseLTL,
    )
from Parser import Parser
import os
from z3 import *


# synthesis_based_repair_dir = '/home/qian/workspace/synthesis_based_repair/synthesis_based_repair'
# sys.path.insert(0, synthesis_based_repair_dir)
repair_dir = '../synthesis_based_repair'
sys.path.insert(0, repair_dir)
from skills import Skill

DEBUG = False
STATIC_WORLD = False

class Compiler:
    def __init__(self, input_file: str, skills_data: dict, symbols_data: dict, objects_data: dict, controllabe_variables: list, uncontrollable_variables: list = [], controllable_mobile_inputs: list = None, controllable_manipulation_inputs: list = None) -> None:
        """
        Args:
            input_file: structuredslugsplus file
            skills_data: skills json file: 
            uncontrollable_variables: a list of reactive variables names
        """
        self.uncontrollable_variables = uncontrollable_variables
        self.controllable_variables = controllabe_variables
        if controllable_mobile_inputs is not None:
            self.controllable_mobile_symbols = controllable_mobile_inputs
        else:
            self.controllable_mobile_symbols = find_controllable_mobile_symbols(symbols_data, objects_data)
        if controllable_manipulation_inputs is not None:
            self.controllable_manipulation_symbols = controllable_manipulation_inputs
        else:
            self.controllable_manipulation_symbols = find_controllable_manipulation_symbols(symbols_data, objects_data)
        self.skills_data = skills_data
        self.symbols_data = symbols_data
        self.objects_data = objects_data
        self.set_data()
        self._set_keys()
        self.set_asts(input_file)
        self.set_mappings_int_to_bool_vars()
    
    def set_data(self) -> None:
        """Set the data for added new skills"""
        self.data = {"new_skills": 0, "env_trans": 0, "sys_trans": 0,}
        self.not_allowed_repair_curr_inp_ls = []
        self.violated_env_trans_hard_indices = set()
        self.MANIPULATION_ONLY = all(is_manipulation_object(obj, self.objects_data) for obj in self.objects_data.keys())
        self.MOBILE_ONLY = all(is_mobile_object(obj, self.objects_data) for obj in self.objects_data.keys())
        if DEBUG:
            print("MANIPULATION_ONLY: ", self.MANIPULATION_ONLY)
            print("MOBILE_ONLY: ", self.MOBILE_ONLY)

    def set_asts(self, input_file: str) -> None:
        """Set the ASTs from the input_file"""
        self.input_file = input_file
        file_extension = os.path.splitext(input_file)[-1]
        if file_extension == ".structuredslugsplus":
            self._get_vars_and_asts_from_structuredslugsplus()
        elif file_extension == ".structuredslugs":
            self._get_vars_and_asts_from_structuredslugs()
        else:
            raise Exception("Input file for monitor_compiler must be either structuredslugsplus or structuredslugs")
    
    def _set_keys(self) -> None:
        self.structuredslugs_property_types = ["[ENV_TRANS]","[ENV_INIT]","[SYS_TRANS]","[SYS_INIT]","[ENV_LIVENESS]","[SYS_LIVENESS]"]
        self.structuredslugsplus_properties = ["[ENV_TRANS]", "[ENV_INIT]", "[INPUT]", "[OUTPUT]", "[SYS_TRANS]", "[SYS_INIT]", "[ENV_LIVENESS]", "[SYS_LIVENESS]", "[OBSERVABLE_INPUT]", "[UNOBSERVABLE_INPUT]", "[CONTROLLABLE_INPUT]", "[ENV_TRANS_HARD]", "[SYS_TRANS_HARD]", "[CHANGE_CONS]", "[NOT_ALLOWED_REPAIR]"]
        self.variable_types = ["[INPUT]","[OUTPUT]","[OBSERVABLE_INPUT]","[UNOBSERVABLE_INPUT]","[CONTROLLABLE_INPUT]"]
        self.structuredslugsplus_property_types = ["[ENV_INIT]", "[SYS_INIT]", "[ENV_TRANS]", "[ENV_TRANS_HARD]", "[SYS_TRANS]", "[SYS_TRANS_HARD]", "[ENV_LIVENESS]", "[SYS_LIVENESS]", "[CHANGE_CONS]", "[NOT_ALLOWED_REPAIR]"]
        self.terminals = {"formula": "Formula", "biimplication": "Biimplication", "implication": "Implication", "conjunction": "Conjunction", "disjunction": "Disjunction", "unary": "UnaryFormula", "not": "NotOperator", "next": "NextOperator", "assignment": "Assignment", "true": "TRUE", "false": "FALSE", "calculation": "CalculationSubformula", "numID": "numID", "comparison": "NumberComparisonOperator", "equal": "EqualOperator", "number": "numeral"}
        self.properties = {"env_trans": "[ENV_TRANS]", "env_init": "[ENV_INIT]", "input": "[INPUT]", "output": "[OUTPUT]", "sys_trans": "[SYS_TRANS]", "sys_init": "[SYS_INIT]", "env_liveness": "[ENV_LIVENESS]", "sys_liveness": "[SYS_LIVENESS]", "observable_input": "[OBSERVABLE_INPUT]", "unobservable_input": "[UNOBSERVABLE_INPUT]", "controllable_input": "[CONTROLLABLE_INPUT]", "env_trans_hard": "[ENV_TRANS_HARD]", "sys_trans_hard": "[SYS_TRANS_HARD]", "change_cons": "[CHANGE_CONS]", "not_allowed_repair": "[NOT_ALLOWED_REPAIR]"}

    def set_mappings_int_to_bool_vars(self) -> None:
        """Set mappings from integer variables to boolean variables"""
        self.vars_to_int_vars = {}
        self.int_vars_to_vars = {}
        self.int_to_bool_vars = {}
        self.int_vars_to_limits = {}
        self.int_to_bool_already_transformed = False

    def _get_vars_and_asts_from_structuredslugsplus(self) -> None:
        """Set up vars and asts initially"""
        vars, asts = get_asts_from_structuredslugsplus(self.input_file)
        self.vars = vars
        self.asts = asts
        # self.env_trans_asts = self.asts[self.properties["env_trans"]] + self.asts[self.properties["env_trans_hard"]]
        self._set_pointers_to_not_any_skills()
        self._set_pointer_to_manipulation_or_mobile_only()
        return None

    def _get_vars_and_asts_from_structuredslugs(self) -> None:
        """Get the ASTs of environment safety assumptions"""
        vars, asts = get_asts(self.input_file)
        self.vars = vars
        self.asts = asts
        # self.env_trans_asts = asts[self.properties["env_trans"]]
        return None
    
    def _set_pointers_to_not_any_skills(self) -> None:
        """Set pointers to not any skills"""
        env_trans_hard = self.asts[self.properties["env_trans_hard"]]
        if not self.MANIPULATION_ONLY and not self.MOBILE_ONLY and len(env_trans_hard) >= 4:
            print("Manipulation only: ", self.MANIPULATION_ONLY)
            inact_wo_mobile_skills = env_trans_hard[-4]
            self.not_any_mobile_skills_conjunction = inact_wo_mobile_skills[1][1]
            # if self.not_any_mobile_skills_conjunction[0] != "Conjunction":
            #     self.not_any_mobile_skills_conjunction = inact_wo_mobile_skills
            if True:
                print("not any mobile skill conjunction: ", self.not_any_mobile_skills_conjunction)
        if not self.MOBILE_ONLY and not self.MANIPULATION_ONLY and len(env_trans_hard) >= 3:
            inact_wo_manipulation_skills = env_trans_hard[-3]
            self.not_any_manipulation_skills_conjunction = inact_wo_manipulation_skills[1][1]
            # if self.not_any_manipulation_skills_conjunction[0] != "Conjunction":
            #     self.not_any_manipulation_skills_conjunction = inact_wo_manipulation_skills
            if True:
                print("not any manipulation skill conjunction: ", self.not_any_manipulation_skills_conjunction)
        if len(env_trans_hard) >= 2:
            inact_wo_skills = env_trans_hard[-2]
            self.not_any_skills_conjunction = inact_wo_skills[1][1]
            # if self.not_any_skills_conjunction[0] != "Conjunction":
            #     self.not_any_skills_conjunction = inact_wo_skills
            if True:
                print("not any skill conjunction: ", self.not_any_skills_conjunction)
        # self.skill_mutual_exclusion = env_trans_hard[-1][1]
        # print(self.skill_mutual_exclusion)

    def _set_pointer_to_manipulation_or_mobile_only(self) -> None:
        """Set pointer to manipulation or mobile only"""
        if not self.MANIPULATION_ONLY and not self.MOBILE_ONLY:
            not_allowed_repair = self.asts[self.properties["not_allowed_repair"]]
            if len(not_allowed_repair) > 0 and len(not_allowed_repair[0]) > 1:
                self.manipulation_or_mobile_only_conjunction = not_allowed_repair[0][1]
                self.asts[self.properties["change_cons"]][0][1] = self.manipulation_or_mobile_only_conjunction
            if DEBUG:
                print("Manipulation or mobile only: ", self.manipulation_or_mobile_only_conjunction)

    def get_vars(self): 
        return self.vars
    
    def get_inputs(self):
        return self.vars[self.properties["input"]]

    def get_skills(self):
        """Get the existing skills from the input_file"""
        return self.vars[self.properties["output"]]
    
    def get_inputs_and_outputs(self):
        return self.get_inputs() + self.get_skills()
    
    def get_inputs_and_outputs_prime(self):
        return [f"{var}'" for var in self.get_inputs_and_outputs()]

    def get_asts(self):
        return self.asts

    def get_env_trans_asts(self):
        return self.asts[self.properties["env_trans"]] + self.asts[self.properties["env_trans_hard"]]
    
    def get_env_trans_mutable_asts(self):
        return self.asts[self.properties["env_trans"]]
    
    def get_sys_trans_hard_asts(self):
        return self.asts[self.properties["sys_trans_hard"]]
    
    def get_sys_live_asts(self):
        return self.asts[self.properties["sys_liveness"]]
    
    def check_realizability(self, plot: bool = False, counterstrategy: bool = False, is_realizable_only: bool = False) -> bool:
        """Check the realizability of the spec represented by the ASTs"""
        tmp_slugsin = "build/tmp.slugsin"
        self.generate_slugsin(tmp_slugsin)
        return check_slugsin_realizability(filename_slugsin=tmp_slugsin, plot=plot, counterstrategy=counterstrategy, is_realizable_only=is_realizable_only)
    
    def check_realizability_with_readable_strategy(self, plot: bool = False) -> bool:
        """Check the realizability of the spec represented by the ASTs"""
        tmp_slugsin = "build/tmp.slugsin"
        self.generate_slugsin(tmp_slugsin)
        return check_slugsin_realizability_with_readable_strategy(filename_slugsin=tmp_slugsin, plot=plot)[0]
    
    def synthesize_readable_strategy(self) -> str:
        """Synthesize a readable strategy"""
        tmp_slugsin = "build/tmp.slugsin"
        self.generate_slugsin(tmp_slugsin)
        return check_slugsin_realizability_with_readable_strategy(tmp_slugsin)[1]
    
    def synthesize_strategy(self) -> str:
        """Synthesize a strategy"""
        tmp_slugsin = "build/tmp.slugsin"
        self.generate_slugsin(tmp_slugsin)
        result = run_slugs(tmp_slugsin, plot=False, counterstrategy=False, is_realizable_only=False)
        return result.stdout
    
    def synthesize_counterstrategy(self) -> str:
        """Synthesize a counterstrategy"""
        tmp_slugsin = "build/tmp.slugsin"
        self.generate_slugsin(tmp_slugsin)
        return check_slugsin_realizability_with_readable_strategy(tmp_slugsin,plot=True, counterstrategy=True)

# =============== Generate Monitors =============== #
    def generate_heading(self) -> None:
        fid = open(self.monitor_file, 'a')
        fid.write('''\
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
                  
    def no_violation_mutable(self, results: list, env_mutable_assumptions_len: int) -> bool:
        """ Process the result
        
        Returns:
            True if all env_mutable assumptions are not violated
        """
        return all(results[:env_mutable_assumptions_len])
    
    def no_violation_hard(self, results: list, env_mutable_assumptions_len: int) -> bool:
        """ Process the result
        
        Returns:
            True if all env_mutable assumptions are not violated
        """
        return all(results[env_mutable_assumptions_len:])

    def filter_violation(self, results: list):
        """Given that there exists violation in results
        Return a list of indices indicating the violated assumptions   
        """
        violated_indices = []
        for i in range(len(results)):
            if not results[i]:
                violated_indices.append(i)
        return violated_indices
                  
    def filter_violation_hard(self, results: list, env_mutable_assumptions_len: int):
        """Given that there exists violation in results
        Return a list of indices indicating the violated hard assumptions   
        """
        violated_indices = []
        for i in range(env_mutable_assumptions_len, len(results)):
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
''')
        fid.close()

    def generate_vars(self) -> None:
        fid = open(self.monitor_file, 'a')

        # Generate X and X'
        for var in self.vars[self.properties["input"]]:
            fid.write('''\
        %s = Bool('%s')
        %s_p = Bool('%s_p')
''' % (var, var, var, var))

        # Generate Y
        for skill in self.vars[self.properties["output"]]:
            fid.write('''\
        %s = Bool('%s')
''' % (skill, skill))

        fid.close()

    def generate_simple_formula(self, ast: list, is_next: bool) -> str:
        if ast[0] == self.terminals["biimplication"]:
            assert len(ast) == 3
            left = self.generate_simple_formula(ast[1], is_next)
            right = self.generate_simple_formula(ast[2], is_next)
            return f"{left} == {right}"
        if ast[0] == self.terminals["implication"]:
            assert len(ast) == 3
            left = self.generate_simple_formula(ast[1], is_next)
            right = self.generate_simple_formula(ast[2], is_next)
            return f"Implies({left}, {right})"
        if ast[0] == self.terminals["conjunction"]:
            elt_list = [self.generate_simple_formula(ast[1], is_next)]
            for elt in ast[2:]:
                elt_list.append(self.generate_simple_formula(elt, is_next))
            return "And(%s)" % ", ".join(elt_list)
        if ast[0] == self.terminals["disjunction"]:
            elt_list = [self.generate_simple_formula(ast[1], is_next)]
            for elt in ast[2:]:
                elt_list.append(self.generate_simple_formula(elt, is_next))
            return "Or(%s)" % ", ".join(elt_list)
        if ast[0] == self.terminals["unary"]:
            if ast[1][0] == self.terminals["not"]:
                return "Not(%s)" % self.generate_simple_formula(ast[2], is_next)
            if ast[1][0] == self.terminals["next"]:
                if is_next:
                    raise Exception("Can't do double next")
                return self.generate_simple_formula(ast[2], True)
        if ast[0] == self.terminals["assignment"]:
            var = ast[1]
            if is_next:
                if "'" in var:
                    raise Exception("Cannot do double next")
                var += "_p"
            if "'" in var:
                var = var[:-1] + "_p"
            return var
        if ast[0] == self.terminals["true"]:
            return "True"
        if ast[0] == self.terminals["false"]:
            return "False"
        print("Cannot parse ast:", file=sys.stderr)
        print(ast, file=sys.stderr)
        raise Exception("AST is not well-formed")
    
    def generate_simple_z3_formula(self, ast: list, is_next: bool) -> ExprRef:
        """Generate z3 formula"""
        if ast[0] == self.terminals["biimplication"]:
            assert len(ast) == 3
            left = self.generate_simple_z3_formula(ast[1], is_next)
            right = self.generate_simple_z3_formula(ast[2], is_next)
            return left == right
        if ast[0] == self.terminals["implication"]:
            assert len(ast) == 3
            left = self.generate_simple_z3_formula(ast[1], is_next)
            right = self.generate_simple_z3_formula(ast[2], is_next)
            return Implies(left, right)
        if ast[0] == self.terminals["conjunction"]:
            elt_list = [self.generate_simple_z3_formula(ast[1], is_next)]
            for elt in ast[2:]:
                elt_list.append(self.generate_simple_z3_formula(elt, is_next))
            return And(elt_list)
        if ast[0] == self.terminals["disjunction"]:
            elt_list = [self.generate_simple_z3_formula(ast[1], is_next)]
            for elt in ast[2:]:
                elt_list.append(self.generate_simple_z3_formula(elt, is_next))
            return Or(elt_list)
        if ast[0] == self.terminals["unary"]:
            if ast[1][0] == self.terminals["not"]:
                return Not(self.generate_simple_z3_formula(ast[2], is_next))
            if ast[1][0] == self.terminals["next"]:
                if is_next:
                    raise Exception("Can't do double next")
                return self.generate_simple_z3_formula(ast[2], True)
        if ast[0] == self.terminals["assignment"]:
            var = ast[1]
            if is_next:
                if "'" in var:
                    raise Exception("Cannot do double next")
                var += "'"
            return self.z3_vars[var]
        if ast[0] == self.terminals["true"]:
            return True
        if ast[0] == self.terminals["false"]:
            return False
        print("Cannot parse ast:", file=sys.stderr)
        print(ast, file=sys.stderr)
        raise Exception("AST is not well-formed")
    
    def generate_formula(self, ast: list) -> str:
        assert len(ast) == 2
        return self.generate_simple_formula(ast[1], False)
    
    def generate_z3_formula(self, ast: list) -> str:
        assert len(ast) == 2
        return self.generate_simple_z3_formula(ast[1], False)

    def generate_formulas(self) -> None:
        fid = open(self.monitor_file, 'a')
        fid.write('''\
        self.formulas = []
''')
        for ast in self.get_env_trans_asts():
            fid.write('''\
        self.formulas.append(%s)
''' % self.generate_formula(ast))
        # print(len(self.asts))
        fid.close()

    def generate_postprocessing(self) -> None:
        fid = open(self.monitor_file, 'a')
        fid.write('''\
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
''')
        fid.close()

    def generate_monitor_curr_input_state(self) -> None:
        fid = open(self.monitor_file, 'a')
        fid.write('''\
    def monitor_curr_input_state(self, input_state: dict) -> list:
''')
        # Generate X
        for var in self.vars[self.properties["input"]]:
            fid.write('''\
        %s = Bool('%s')
''' % (var, var))
            
        # Generate the SMT calls
        fid.write('''\
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
''')
                  
        fid.close()



    def generate_z3_monitor(self, filename: str) -> None:
        """Generate SAT formulas"""
        # Set up output file
        self.monitor_file = filename
        clear_file(self.monitor_file)

        # Generate the heading, same for all monitors
        self.generate_heading()

        # Generate the vars and skills
        self.generate_vars()

        # Generate the formulas
        self.generate_formulas()

        # Generate monitor for each formula
        self.generate_postprocessing()  

        # Generate monitoring the current input state
        self.generate_monitor_curr_input_state()

# ================================================================ #
####
# =============== Generate Structuredslugsplus, Structuredslugs, Slugsin files =============== #
    def generate_spec_vars(self) -> None:
        """Write variables to structuredslugsplus file"""
        fid = open(self.file, 'a')
        for variable_type in self.variable_types:
            if variable_type in self.vars and len(self.vars[variable_type]) > 0:
                fid.write(f'{variable_type}\n')
                for var in self.vars[variable_type]:
                    fid.write(f'{var}\n')
                fid.write('\n')
        fid.close()

    def generate_structuredslugsplus_formula(self, ast: list, is_prime: bool) -> list:
        """Return a list of tokens in structuredslugsplus corresponding to the ast"""
        # print(ast)
        if ast[0] == self.terminals['formula']:
            assert len(ast) == 2
            return self.generate_structuredslugsplus_formula(ast[1], is_prime)
        if ast[0] == self.terminals['biimplication']:
            assert len(ast) == 3
            b1 = self.generate_structuredslugsplus_formula(ast[1], is_prime)
            b2 = self.generate_structuredslugsplus_formula(ast[2], is_prime)
            return ['('] + b1 + [ '<->'] + b2 + [')']
        if ast[0] == self.terminals['implication']:
            assert len(ast) == 3, f"Error in {ast}"
            b1 = self.generate_structuredslugsplus_formula(ast[1], is_prime)
            b2 = self.generate_structuredslugsplus_formula(ast[2], is_prime)
            return ['('] + b1 + ['->'] + b2 + [')']
        if ast[0] == self.terminals['conjunction']:
            result = self.generate_structuredslugsplus_formula(ast[1], is_prime)
            for b in ast[2:]:
                result.append('&')
                result.extend(self.generate_structuredslugsplus_formula(b, is_prime))
            return ['('] + result + [')']
        if ast[0] == self.terminals['disjunction']:
            result = self.generate_structuredslugsplus_formula(ast[1], is_prime)
            for b in ast[2:]:
                result.append('|')
                result.extend(self.generate_structuredslugsplus_formula(b, is_prime))
            return ['('] + result + [')']
        if ast[0] == self.terminals['unary']:
            if ast[1][0] == self.terminals['not']:
                return ['!'] + self.generate_structuredslugsplus_formula(ast[2], is_prime)
            elif ast[1][0] == self.terminals['next']:
                if is_prime:
                    raise Exception("Nested nexts are not supported in GR(1)")
                return self.generate_structuredslugsplus_formula(ast[2], True)
        if ast[0] == self.terminals['assignment']:
            var = ast[1]
            if is_prime:
                if "'" in var:
                    raise Exception("Do not support double prime variable")
                var = var + "'"
            return [var]
        if ast[0] == self.terminals['true']:
            return ["TRUE"]
        if ast[0] == self.terminals['false']:
            return ["FALSE"]
        print("Error ast: ")
        print(ast)
        raise Exception("Monitor compiler parsing error when generating structuredslugsplus file")                


    def generate_structuredslugsplus_formulas(self) -> None:
        """Write formulas to structuredslugsplus file"""
        self.add_not_allowed_repair_curr_inp_to_not_allowed_repair()
        fid = open(self.file, 'a')
        for property_type in self.structuredslugsplus_property_types:
            fid.write(f'{property_type}\n')
            # print(property_type)
            for ast in self.asts[property_type]:
                tokens = self.generate_structuredslugsplus_formula(ast, False)
                curr_line = " ".join(tokens)
                fid.write(f'{curr_line}\n')
            fid.write('\n')
        fid.close()
        self.remove_not_allowed_repair_curr_inp_to_not_allowed_repair()

    def add_not_allowed_repair_curr_inp_to_not_allowed_repair(self) -> None:
        self.asts["[NOT_ALLOWED_REPAIR]"].extend(self.not_allowed_repair_curr_inp_ls)
        return None
    
    def remove_not_allowed_repair_curr_inp_to_not_allowed_repair(self) -> None:
        for _ in range(len(self.not_allowed_repair_curr_inp_ls)):
            self.asts["[NOT_ALLOWED_REPAIR]"].pop()
        return None


    def generate_structuredslugsplus(self, filename: str) -> None:
        """From ASTs, generate structuredslugsplus file to filename"""
        # Set up output file
        self.file = filename
        clear_file(self.file)

        # Generate vars
        self.generate_spec_vars()

        # Generate formulas
        self.generate_structuredslugsplus_formulas()

    def generate_structuredslugs_formulas(self) -> None:
        """Write formulas to structuredslugs file"""
        fid = open(self.file, 'a')
        for property_type in self.structuredslugs_property_types:
            fid.write(f'{property_type}\n')
            for ast in self.asts[property_type]:
                tokens = self.generate_structuredslugsplus_formula(ast, False)
                curr_line = " ".join(tokens)
                fid.write(f'{curr_line}\n')
            if property_type == self.properties["env_trans"]:
                for ast in self.asts[self.properties["env_trans_hard"]]:
                    tokens = self.generate_structuredslugsplus_formula(ast, False)
                    curr_line = " ".join(tokens)
                    fid.write(f'{curr_line}\n')
            elif property_type == self.properties["sys_trans"]:
                for ast in self.asts[self.properties["sys_trans_hard"]]:
                    tokens = self.generate_structuredslugsplus_formula(ast, False)
                    curr_line = " ".join(tokens)
                    fid.write(f'{curr_line}\n')
            fid.write('\n')
        fid.close()

    def generate_structuredslugs(self, filename: str) -> None:
        """From ASTs, generate structuredslugsplus file to filename"""
        # Set up output file
        self.file = filename
        clear_file(self.file)

        # Generate vars
        self.generate_spec_vars()

        # Generate formulas
        self.generate_structuredslugs_formulas()

    def generate_slugsin(self, output_file: str) -> None:
        asts_to_slugsin(self.vars, self.asts, output_file)
# ================================================================ #

# =============== Transform ASTs with integer inputs to ASTs with boolean inputs only =============== #
    def transform_asts_int2bool(self) -> None:
        """Transform ASTs with integer inputs to ASTs with boolean inputs only"""
        if self.int_to_bool_already_transformed:
            raise Exception("The ASTs have already been transformed from integer inputs to boolean inputs only")
        if self.int2bool_transform_needed():
            self._change_integer_inputs_to_boolean_inputs()
            self.int_to_bool_already_transformed = True
        return None

    def int2bool_transform_needed(self) -> bool:
        """Check if the transformation is needed by checking if there is any integer input"""
        for var in self.vars[self.properties["input"]]:
            if ":" in var:
                return True
        return False
    
    def _change_integer_inputs_to_boolean_inputs(self) -> None:
        """Change the integer inputs to boolean inputs"""
        # 1. Change the vars
        if True:
            print("old inputs: ", self.vars[self.properties["input"]])
        for var in self.vars[self.properties["input"]]:
            if ":" in var:
                self._change_integer_input_to_boolean_input(var)
        
        # 2. Modify self.vars[self.properties["input"]] in place
        for int_var, bool_vars in self.int_to_bool_vars.items():
            self.vars[self.properties["input"]].remove(self.int_vars_to_vars[int_var])
            self.vars[self.properties["input"]] += bool_vars
        
        if True:
            print("new inputs: ", self.vars[self.properties["input"]])
            # sys.exit(0)

        # 3. Change the ASTs
        for property_type in self.structuredslugsplus_property_types:
            for ast in self.asts[property_type]:
                self._change_integer_inputs_to_boolean_inputs_in_ast(ast)

        if True:
            self.print_asts()
            print("new asts: ", self.asts)
            # sys.exit(0)

        # 4. Add input mutual exclusion for each integer variable
        for _, bool_vars in self.int_to_bool_vars.items():
            self._add_inputs_mutual_exclusion_and_must_exist(bool_vars, is_primed = True)
            self._add_inputs_mutual_exclusion_and_must_exist(bool_vars, is_primed = False)

        if True:
            self.print_asts()
            print("new asts with added mx: ", self.asts)

    def _change_integer_input_to_boolean_input(self, var: str) -> None:
        """Change the integer input to boolean input"""
        parts = var.split(":")
        assert len(parts) == 2, f"Error in {var}, too many ':'"
        int_var = parts[0]
        self.vars_to_int_vars[var] = int_var
        self.int_vars_to_vars[int_var] = var
        int_var_range = parts[1]
        parts2 = int_var_range.split("...")
        assert len(parts2) == 2, f"Error in {int_var_range}, too many '...'"
        try:
            start = int(parts2[0])
            end = int(parts2[1])
        except:
            raise Exception(f"Error in {int_var_range}, not integer")
        assert start <= end, f"Error in {int_var_range}, start > end"
        self.int_vars_to_limits[int_var] = (start, end)

        self.int_to_bool_vars[int_var] = []
        for i in range(start, end + 1):
            bool_var = f"{int_var}{i}"
            self.int_to_bool_vars[int_var].append(bool_var)
            # self.vars[self.properties["input"]].append(bool_var)
        # self.vars[self.properties["input"]].remove(var)
        return None
    
    def _change_integer_inputs_to_boolean_inputs_in_ast(self, ast: list) -> None:
        """Change the integer inputs to boolean inputs in the ast"""
        if ast[0] == self.terminals['formula']:
            assert len(ast) == 2, f"Error in {ast}, formula should have 2 elts"
            self._change_integer_inputs_to_boolean_inputs_in_ast(ast[1])
            return None
        if ast[0] == self.terminals['biimplication']:
            assert len(ast) == 3, f"Error in {ast}, biimplication should have 3 elts"
            self._change_integer_inputs_to_boolean_inputs_in_ast(ast[1])
            self._change_integer_inputs_to_boolean_inputs_in_ast(ast[2])
            return None
        if ast[0] == self.terminals['implication']:
            assert len(ast) == 3, f"Error in {ast}, implication should have 3 elts"
            self._change_integer_inputs_to_boolean_inputs_in_ast(ast[1])
            self._change_integer_inputs_to_boolean_inputs_in_ast(ast[2])
            return None
        if ast[0] == self.terminals['conjunction'] or ast[0] == self.terminals['disjunction']:
            assert len(ast) >= 3, f"Error in {ast}, conjunction or disjunction should have at least 3 elts"
            for b in ast[1:]:
                self._change_integer_inputs_to_boolean_inputs_in_ast(b)
            return None 
        if ast[0] == self.terminals['unary']:
            assert len(ast) == 3, f"Error in {ast}, unary should have 3 elts"
            self._change_integer_inputs_to_boolean_inputs_in_ast(ast[2])
            return None
        if ast[0] == self.terminals['assignment'] or ast[0] == self.terminals['true'] or ast[0] == self.terminals['false']:
            # Do nothing
            return None
        if ast[0] == self.terminals['calculation']:
            assert len(ast) == 4, f"Error in {ast}, calculation should have 3 elts"
            assert ast[2][0] == self.terminals["comparison"], f"Error in {ast}, calculation formula should have comparison as the second elt"
            self._change_integer_inputs_to_boolean_inputs_in_ast(ast[2])
            
            left = ast[1]
            right = ast[3]
            new_conjunction_list = []

            # Case 1: both left and right are numID
            if left[0] == self.terminals["numID"] and right[0] == self.terminals["numID"]:
                left_int_var = left[1]
                right_int_var = right[1]
                left_bool_vars = self._get_bool_vars_from_int_var(left_int_var)
                right_bool_vars = self._get_bool_vars_from_int_var(right_int_var)
                assert len(left_bool_vars) == len(right_bool_vars), f"Error in {ast}, left {left_bool_vars} and right {right_bool_vars} should have the same number of boolean variables"
                for i in range(len(left_bool_vars)):
                    new_conjunction_list.append(self.add_biimplication_wrapper(self.name2assignment(left_bool_vars[i]), self.name2assignment(right_bool_vars[i])))

            # Case 2: one is numID, the other is number 
            elif left[0] == self.terminals["numID"] or right[0] == self.terminals["numID"]:
                if left[0] != self.terminals["numID"]:
                    left, right = right, left
                assert right[0] == self.terminals["number"], f"Error in {ast}, {right} should be number"
                left_int_var = left[1]
                try:
                    right_number = int(right[1])
                except:
                    raise Exception(f"Error in {ast}, {right} should be number")
                left_bool_vars = self._get_bool_vars_from_int_var(left_int_var)
                limit = self._get_int_var_limit(left_int_var)
                if DEBUG:
                    print("right_number: ", right_number)
                    print("limit: ", limit)
                for i in range(limit[0], limit[1]+1):
                    if DEBUG: breakpoint()
                    if i == right_number:
                        new_conjunction_list.append(self.name2assignment(left_bool_vars[i - limit[0]]))
                    else:
                        new_conjunction_list.append(self.add_not_wrapper(self.name2assignment(left_bool_vars[i - limit[0]])))
            
            # Case 3: both are numbers
            else:
                raise Exception(f"Error in {ast}, both {left} and {right} are numbers, not supported yet")
            
            ast[0] = self.terminals['conjunction']
            ast[1:] = new_conjunction_list
            return None
            
        if ast[0] == self.terminals['comparison']:
            assert len(ast) == 2, f"Error in {ast}, comparison should have 2 elts"
            assert ast[1][0] == self.terminals["equal"], f"Error in {ast}, comparison should have equal as the second elt, other comparision operators are not supported yet"

    def _get_bool_vars_from_int_var(self, int_var: str) -> list:
        """Return a list of boolean variables corresponding to the integer variable"""
        is_primed = self.is_prime(int_var)
        if is_primed: 
            int_var = int_var[:-1]
        if int_var not in self.int_to_bool_vars:
            raise Exception(f"Error in {int_var}, not an integer variable")
        return self.int_to_bool_vars[int_var] if not is_primed else [var + "'" for var in self.int_to_bool_vars[int_var]]
    
    def _get_int_var_limit(self, int_var: str) -> tuple:
        """Return the limit of the integer variable"""
        is_primed = self.is_prime(int_var)
        if is_primed:
            int_var = int_var[:-1]
        if int_var not in self.int_vars_to_limits:
            raise Exception(f"Error in {int_var}, not an integer variable")
        return self.int_vars_to_limits[int_var]

    def is_prime(self, var: str) -> bool:
        """Return whether the variable is primed"""
        return var[-1] == "'"
    
    def _add_inputs_mutual_exclusion_and_must_exist(self, vars: list, is_primed = False) -> None:
        """Add boolean input variables mutual exclusion to env_trans_hard"""
        if is_primed:
            vars = [var + "'" for var in vars]

        # Add must exist
        new_formula = self.add_formula_wrapper(self.add_disjunction_wrapper([self.name2assignment(var) for var in vars]))
        self.asts[self.properties["env_trans_hard"]].insert(0, new_formula)

        # Add mutual exclusion
        for i in range(len(vars)):
            for j in range(i+1, len(vars)):
                new_formula = self.add_formula_wrapper(
                    self.add_not_wrapper(
                        self.add_conjunction_wrapper(
                            [self.name2assignment(vars[i]), self.name2assignment(vars[j])])))
                self.asts[self.properties["env_trans_hard"]].insert(0, new_formula)



# ================================================================ #

# =============== Relax Env_Trans assumptions =============== #
    def contains_controllable_input(self, ast: list) -> bool:
        """Check whether the ast contains controllable inputs
        """
        if isinstance(ast, str):
            return ast in self.controllable_variables
        for sub_ast in ast:
            if self.contains_controllable_input(sub_ast):
                return True
        return False

    def env_to_formula(self, envs: list, true_only: bool = False) -> list:
        """Return a conjunction of env: list of dict[str, bool] in AST"""
        formula = ['Conjunction']
        if len(envs) >= 2:
            envs[-1] = self.change_Xp_names(envs[-1])
        for env in envs:
            self.dict_to_ast(conjunction=formula, env=env, true_only=true_only)
        return formula
    
    def remove_uncontrollable_inputs(self, X: dict, X_p: dict) -> None:
        """Remove the reactive inputs from X and X_p in place"""
        for uncontrollable_variable in self.uncontrollable_variables:
            if uncontrollable_variable in X.keys():
                X.pop(uncontrollable_variable)
                X_p.pop(uncontrollable_variable)
        return None
    
    def remove_controllable_inputs(self, X: dict) -> dict:
        """Remove the controllable inputs from X, only left uncontrollable inputs"""
        new_X = {}
        for uncontrollable_input in self.uncontrollable_variables:
            if uncontrollable_input in X.keys():
                new_X[uncontrollable_input] = X[uncontrollable_input]
        return new_X

    def relax_assumption(self, indices_xyx: list, indices_x: list, X: dict, Y: dict, X_p: dict, true_only: bool = False) -> None:
        """Relax the violated environment assumption indicated by index to include new environment behavior (the conjunction of X, Y, X') """
        # assert len(index) == 1, "More than one assumption is violated"
        # violation_index = index[0]
        # self.violated_env_trans_hard_indices.update(index)
        X_original, Y_original, X_p_original = X, Y, X_p
        for violated_index in indices_xyx[::-1]: # <- Relax all violated assumptions from the last to first, so that env_trans_hard is relaxed first w/o removing the uncontrollable variables
            X, Y, X_p = copy.deepcopy(X_original), copy.deepcopy(Y_original), copy.deepcopy(X_p_original)
            violated_assumption = self.get_env_trans_asts()[violated_index]
            if not self.is_env_trans_hard(violated_index):
                self.remove_uncontrollable_inputs(X, X_p) # <- Remove reactive variables here
            else:
                self.violated_env_trans_hard_indices.add(violated_index - len(self.asts[self.properties["env_trans"]]))
            
            ## ==== Consider the special cases ==== ##
            # if self.contains_skill(violated_assumption):
            #     formula_to_be_added = self.env_to_formula([X, Y, X_p])
            # elif self.contains_controllable_input(violated_assumption):
            #     formula_to_be_added = self.env_to_formula([X, X_p])
            # else:
            #     formula_to_be_added = self.env_to_formula([self.remove_controllable_inputs(X), self.remove_controllable_inputs(X_p)])
            ## ==== Not consider the special cases ==== ##
            formula_to_be_added = self.env_to_formula([X, Y, X_p], true_only=true_only)

            old_formula = violated_assumption[1]
            if DEBUG:
                print("old formula: ", old_formula)
            if old_formula[0][0] == self.terminals['disjunction']:
                old_formula[0][1].append(formula_to_be_added)
            else:
                if DEBUG:
                    print("old_formula[1][0]", old_formula[1][0])
                new_formula = [self.terminals['disjunction'], old_formula, formula_to_be_added]
                violated_assumption[1] = new_formula
            if DEBUG:
                print("Relaxed environment assumption:", violated_index)
                print(violated_assumption)
        
        for violated_index in indices_x: # <- Relax the assumption to include the current input state X_p
            X_p = copy.deepcopy(X_p_original)
            violated_assumption = self.get_env_trans_asts()[violated_index]
            if self.is_env_trans_hard(violated_index):
                self.violated_env_trans_hard_indices.add(violated_index - len(self.asts[self.properties["env_trans"]]))
            if self.contains_controllable_input(violated_assumption):
                formula_to_be_added = self.env_to_formula([X_p], true_only=true_only)
            else:
                formula_to_be_added = self.env_to_formula([self.remove_controllable_inputs(X_p)], true_only=true_only)
            old_formula = violated_assumption[1]
            if old_formula[0][0] == self.terminals['disjunction']:
                old_formula[0][1].append(formula_to_be_added)
            else:
                new_formula = [self.terminals['disjunction'], old_formula, formula_to_be_added]
                violated_assumption[1] = new_formula
            if DEBUG:
                print("Relaxed environment assumption:", violated_index)
                self.print_ast(violated_assumption)

    def relax_assumption_given_violation_formula(self, indices_xyx: list, violation_formula: str) -> None:
        """Relax the violated environment assumption indicated by index to include new environment behavior (the violation formula) """
        formula_to_be_added = parseLTL(violation_formula, "")[1]
        for violated_index in indices_xyx[::-1]: # <- Relax all violated assumptions from the last to first, so that env_trans_hard is relaxed first w/o removing the uncontrollable variables
            violated_assumption = self.get_env_trans_asts()[violated_index]
            old_formula = violated_assumption[1]
            if DEBUG:
                print("old formula: ", old_formula)
            if old_formula[0][0] == self.terminals['disjunction']:
                old_formula[0][1].append(formula_to_be_added)
            else:
                if DEBUG:
                    print("old_formula[1][0]", old_formula[1][0])
                new_formula = [self.terminals['disjunction'], old_formula, formula_to_be_added]
                violated_assumption[1] = new_formula
            # old_formula = violated_assumption[1]
            # old_formula.append(ast_tree)
            if DEBUG:
                print("ast_tree: ", formula_to_be_added)
                print("Relaxed environment assumption:", violated_index)
                print(" ==== old formula ====")
                print(old_formula)
                print(" ==== new_formula====")
                print(new_formula)
                # print(" ==== old formula [1] ====")
                # print(old_formula[1])
                # print(" ==== old formula [2] ====")
                # print(old_formula[2])
                # print(" ==== old formula [3] ====")
                # print(old_formula[3])
                print("Exit due to debugging")
                sys.exit(0)
        return None
        

    def is_env_trans_hard(self, index: int) -> bool:
        """Returns whether the violated assumption is an env_trans_hard"""
        return index >= len(self.asts[self.properties["env_trans"]])

    def change_env_init(self, X: dict) -> None:
        """Change the env_init of ASTs"""
        formulas = []
        self.dict_to_ast_formulas(formulas, X)
        self.asts[self.properties["env_init"]] = formulas

    def change_env_init_llm(self, violated_observation: dict) -> None:
        """Change the env_init of ASTs for llm"""
        if "violations" not in violated_observation.keys():
            violated_observation_new = {}
            violated_observation_new["violations"] = [violated_observation]
            violated_observation = violated_observation_new
        last_violation = violated_observation["violations"][-1]
        X = state2dict_bool(inputs=self.get_inputs(), inp_state=last_violation["current_input_state"])
        formulas = []
        self.dict_to_ast_formulas(formulas, X)
        self.asts[self.properties["env_init"]] = formulas

    def reorder_liveness_goals(self, rank: int) -> None:
        """Reorder asts['[SYS_LIVENESS]'] s.t. the one with the given rank and those after it are at the front"""
        self.asts[self.properties["sys_liveness"]] = self.asts[self.properties["sys_liveness"]][rank:] + self.asts[self.properties["sys_liveness"]][:rank]

    def change_not_allowed_repair_curr_inp(self, X: dict, is_mobile_repair: bool, objects_data: dict, inputs_data: dict) -> None:
        """
        Record the mobile/manipulation X as not allowed repair constraint
        """
        self.not_allowed_repair_curr_inp_ls = []
        keyword = "manipulation" if is_mobile_repair else "mobile"
        for inp, val in X.items():
            if val and inputs_data[inp] and \
            objects_data[inputs_data[inp]['object']]['type'] == keyword:
                self.not_allowed_repair_curr_inp_ls.append(self.add_formula_wrapper(self.name2assignment(inp)))
        print("========")
        print(f"not allowed repair curr inp: {self.not_allowed_repair_curr_inp_ls}")
        print("========")
        return None



# ================================================================ #

# =============== Add and remove backup skills before and after repair module =============== #

    def add_backup_skills_to_formula(self, ast: list, is_prime: bool = False) -> None:
        if ast[0] == self.terminals['formula']:
            assert len(ast) == 2
            self.add_backup_skills_to_formula(ast[1], is_prime)
            return None
        if ast[0] == self.terminals['biimplication'] or ast[0] == self.terminals['implication']:
            assert len(ast) == 3
            self.add_backup_skills_to_formula(ast[1], is_prime)
            self.add_backup_skills_to_formula(ast[2], is_prime)
            return None
        if ast[0] == self.terminals['conjunction'] or ast[0] == self.terminals['disjunction']:
            for b in ast[1:]:
                self.add_backup_skills_to_formula(b, is_prime)
            return None
        if ast[0] == self.terminals['unary']:
            if ast[1][0] == self.terminals['not']:
                self.add_backup_skills_to_formula(ast[2], is_prime)
                return None
            elif ast[1][0] == self.terminals['next']:
                if is_prime:
                    raise Exception("Nested nexts are not supported in GR(1)")
                self.add_backup_skills_to_formula(ast[2], True)
                return None
        if ast[0] == self.terminals['assignment']:
            var = ast[1]
            if is_prime and "'" in var:
                raise Exception("Do not support double prime variable")
            if 'skill' in var:
                if "'" in var:
                    var = var[:-1] + 'backup' + "'"
                else:
                    var = var + 'backup'
                ast[1] = var
            return None
        if ast[0] == self.terminals['true'] or ast[0] == self.terminals['false']:
            return None
        print("Error")
        raise Exception("Monitor compiler parsing error when adding backup skills to the spec") 


    def add_backup_skills(self) -> None:
        """Add backup skills to spec before repair module"""
        # output, sys_init, not_allowed_repair
        backup_skills = [f'{skill}backup' for skill in self.vars[self.properties["output"]]]
        self.vars[self.properties["output"]] = self.vars[self.properties["output"]] + backup_skills
        for backup_skill in backup_skills:
            self.skills_data[backup_skill] = self.skills_data[backup_skill[:-6]]
        self.asts[self.properties["sys_init"]] = self.asts[self.properties["sys_init"]] + [self.add_formula_wrapper(self.add_not_wrapper(self.name2assignment(backup_skill))) for backup_skill in backup_skills]
        self.asts[self.properties["not_allowed_repair"]] = self.asts[self.properties["not_allowed_repair"]] + [self.add_formula_wrapper(self.add_not_wrapper(self.name2assignment(backup_skill))) for backup_skill in backup_skills]
        

        # Bookkeeping
        # self.data_backup_skills = dict()
        # self.data_backup_skills["num_backup_skills"] = len(backup_skills)

        # env_trans, sys_trans, env_liveness, sys_liveness
        for property_type in [self.properties['env_trans'], self.properties['sys_trans'], self.properties['env_liveness'], self.properties['sys_liveness']]:
            new_formulas = []
            for formula in self.asts[property_type]:
                if self.contains_skill(formula):
                    formula_prime = copy.deepcopy(formula)
                    self.add_backup_skills_to_formula(formula_prime)
                    new_formulas.append(formula_prime)
            self.asts[property_type] = self.asts[property_type] + new_formulas
            # self.data_backup_skills[property_type] = len(new_formulas)
        
        # env_trans_hard, sys_trans_hard
        self.generate_env_trans_hard()
        self.generate_sys_trans_hard()

    def remove_backup_skills(self) -> None:
        """Remove backup skills from spec after repair module"""
        # print(self.vars[self.properties["output"]])
        self.vars[self.properties["output"]] = self.filter_list(self.vars[self.properties["output"]], lambda x: not self.contains_backup(x))
        # print(self.vars[self.properties["output"]])
        for property_type in [self.properties["sys_init"], self.properties["not_allowed_repair"], self.properties['env_trans'], self.properties['sys_trans'], self.properties['env_liveness'], self.properties['sys_liveness']]:
            self.asts[property_type] = self.filter_list(self.asts[property_type], lambda x: not self.contains_skill_and_backup(x))
            [ast for ast in self.asts[property_type] if not self.contains_skill_and_backup(ast)]
        for skill_name in list(self.skills_data.keys()):
            if 'backup' in skill_name:
                self.skills_data.pop(skill_name)
        self.generate_env_trans_hard()
        self.generate_sys_trans_hard()
        # self.data_backup_skills = dict()

    def contains_backup(self, formula: list) -> bool:
        """Given a formula AST, return True if it contains the string "backup" in any of its sub-formulas"""
        return self.contains_keyword(formula, 'backup')
    
    def contains_skill_and_backup(self, formula: list) -> bool:
        """Given a formula AST, return True if it contains the string "skill" and "backup" in any of its sub-formulas"""
        return self.contains_keywords(formula, ['skill', 'backup'])
        
    def change_manipulation_or_mobile_only(self, opts: dict) -> None:
        """Change manipulation only or mobile only in not_allowed_repair by the opts"""
        if not self.MANIPULATION_ONLY and hasattr(self, 'manipulation_or_mobile_only_conjunction'):
            if "mobile_repair" not in opts.keys():
                raise Exception("mobile_repair not in opts")
            self.manipulation_or_mobile_only_conjunction[1:] = []
            if True:
                print("mobile_repair: ", opts["mobile_repair"])
            if opts["mobile_repair"]:
                self.manipulation_or_mobile_only_conjunction.extend(self._create_variables_stay_list(self.controllable_manipulation_symbols))
            else:
                self.manipulation_or_mobile_only_conjunction.extend(self._create_variables_stay_list(self.controllable_mobile_symbols))
            if True:
                print("mobile_repair: ", opts["mobile_repair"])
                print("New manipulation_or_mobile_only_conjunction: ", self.manipulation_or_mobile_only_conjunction)

    def _create_variables_stay_list(self, variables: list) -> list:
        """Given a list of variables [var, ...], 
        Return a list of [var <-> var', ...]"""
        result = []
        for var in variables:
            result.append(self.add_biimplication_wrapper(self.name2assignment(var), self.name2assignment(var, is_prime=True)))
        return result

# ================================================================ #


# =============== Add and remove suggested skills from repair module =============== #
    def create_postcondition(self, name: str, skill: Skill) -> list:
        """Create postconditions in AST from the given skill
        Deprived"""

        post_dict_list = skill.get_final_posts()
        posts = self.get_env_trans_posts(post_dict_list)
        return [self.terminals['formula'], [self.terminals['implication'], [self.terminals['assignment'], name], posts]]

    def create_precondition_initial_pres(self, name: str, skill: Skill) -> list:
        """Create a list of [initial_pre1', initial_pre2']"""
        result = []
        for pre_dict in skill.get_initial_pres():
            conjunction = self.create_empty_conjunction()
            self.dict_to_ast(conjunction, pre_dict, is_prime=True)
            result.append(conjunction)
        return result
    
    def create_precondition_initial_pres_llm(self, name: str, intermediate_transitions: list) -> list:
        """Create a list of [initial_pre1', initial_pre2']"""
        result = []
        initial_pres = self._find_initial_pres_from_intermediate_transitions(intermediate_transitions)
        for pre_dict in initial_pres:
            conjunction = self.create_empty_conjunction()
            self.dict_to_ast(conjunction, pre_dict, is_prime=True)
            result.append(conjunction)
        return result
    
    def _find_initial_pres_from_intermediate_transitions(self, intermediate_transitions: list) -> list:
        """Return the initial preconditions from the intermediate transitions"""
        all_posts = [post for _, post in intermediate_transitions]
        all_pres = [pre for pre, _ in intermediate_transitions]
        initial_pres = [pre for pre in all_pres if pre not in all_posts]
        return initial_pres

    def create_precondition(self, name: str, skill: Skill) -> list:
        """Create precondition of skill in AST"""
        conjunctions = []
        for pre_dict in skill.get_initial_pres():
            pre_dict = self.change_Xp_names(pre_dict)
            conjunction = [self.terminals['conjunction']]
            self.dict_to_ast(conjunction, pre_dict)
            conjunctions.append(conjunction)
        if len(conjunctions) == 0:
            # print("The suggested skill has no initial precondition", file=sys.stderr)
            raise Exception("Suggested skill has no initial precondition")
        if len(conjunctions) > 1:
            disjunction = [self.terminals['disjunction']]
            disjunction.extend(conjunctions)
        else:
            disjunction = conjunctions[0]
        pre = [self.terminals['unary'], [self.terminals['not'], ('!',)], disjunction]
        return [self.terminals['formula'], [self.terminals['implication'], pre, [self.terminals['unary'], [self.terminals['not'], ('!',)], [self.terminals['assignment'], f"{name}'"]]]]

    def get_env_trans_pre(self, name: str, pre_dict: dict) -> list:
        """Return the conjunction skill & pre"""
        conjunction = [self.terminals['conjunction']]
        conjunction.append(self.name2assignment(name))
        self.dict_to_ast(conjunction, pre_dict)
        return conjunction
    
    def get_env_trans_posts(self, post_dict_list) -> list:
        """Returns the [post] part of skill & pre -> post in env trans
        Given a list of dicts, return the disjunction of conjunction of dicts"""
        conjunctions = []
        for post_dict in post_dict_list:
            post_dict = self.change_Xp_names(post_dict)
            conjunction = self.create_empty_conjunction()
            self.dict_to_ast(conjunction, post_dict)
            conjunctions.append(conjunction)
        if len(conjunctions) == 0:
            raise Exception("Suggested skill has no final postcondition")
        if len(conjunctions) > 1:
            posts = [self.terminals['disjunction']]
            posts.extend(conjunctions)
        else:
            posts = conjunctions[0]
        return posts

    def pre_posts_to_env_trans_ast(self, name: str, pre_dict: dict, post_dict_list: list) -> list:
        """Given a pair of precondition and a list of postconditions, return the corresponding formula AST"""
        pre = self.get_env_trans_pre(name, pre_dict)
        posts = self.get_env_trans_posts(post_dict_list)
        return [self.terminals['formula'], [self.terminals['implication'], pre, posts]]  


    def create_postconditions(self, name: str, skill: Skill) -> list:
        """Create the postconditions of a skill in AST env_trans"""
        formulas = []
        for pre_dict, post_dict_list in skill.get_intermediate_states():
            formulas.append(self.pre_posts_to_env_trans_ast(name, pre_dict, post_dict_list))
        return formulas
    
    def create_postconditions_llm(self, name: str, intermediate_transitions: list) -> list:
        """Create the postconditions of a skill in AST env"""
        formulas = []
        for pre_dict, post_dict_list in intermediate_transitions:
            formulas.append(self.pre_posts_to_env_trans_ast(name, pre_dict, post_dict_list))
        return formulas

    
    def create_pre_skill_post_conjunction(self, name: str, pre_dict: dict, post_dict: dict) -> list:
        """Create pre & skill & post' conjunction"""
        allowable_skill_conjunction = self.create_empty_conjunction() 
        self.dict_to_ast(allowable_skill_conjunction, pre_dict)
        allowable_skill_conjunction.append(self.name2assignment(name))
        self.dict_to_ast(allowable_skill_conjunction, post_dict, is_prime=True)
        return allowable_skill_conjunction
    
    def create_pre_skill_post_continue_formula(self, name: str, allowable_skill_conjunction: list) -> list:
        """Create pre & skill & post' -> skill' formula"""
        post = self.name2assignment(name, is_prime=True)
        implication = self.add_implication_wrapper(allowable_skill_conjunction, post)
        return self.add_formula_wrapper(implication)
    
    def pre_continue_sys_trans_ast(self, name: str, pre_dict: dict, post_dict_list: list, allowable_skill_disjunction: list, skill_continue_formulas: list, all_pres: list) -> None:
        """Create the precondition and change continue AST formula for each pre&post pair
        Args:
            name:                           str:                skill name
            pre_dict:                       dict[str, bool]:    precondition state
            post_dict_list:                 list[dict]:         a list of postcondition states  
            allowable_skills_disjunction:   list:               a list of disjunction corresponding to each intermediate state
            skill_continue_formulas:        list:               a list of formulas representing (pre & skill & post') -> skill'
            all_pres:                       list:               all possible preconditions of the skill
        Returns:
            None, modify allowable_skills_disjunction and skill_continue_formulas in-place
        """
        for post_dict in post_dict_list:
            if post_dict not in all_pres or pre_dict == post_dict:
                continue # Don't need the final post
            allowable_skill_conjunction = self.create_pre_skill_post_conjunction(name, pre_dict, post_dict)
            allowable_skill_disjunction.append(allowable_skill_conjunction)
            skill_continue_formulas.append(self.create_pre_skill_post_continue_formula(name, copy.deepcopy(allowable_skill_conjunction)))
        return None

    def create_preconditions(self, name: str, skill: Skill) -> list:
        """Create the preconditions of a skill in AST sys_trans
        Components:
            1. !((pre & skill & post') | ... | init_pre') -> !skill'
            2. change continues: (pre & skill & post') -> skill'
        """
        allowable_skill_disjunction = self.create_empty_disjunction()
        skill_continue_formulas = []
        all_pres = [pre for pre, _ in skill.get_intermediate_states()]
        for pre_dict, post_dict_list in skill.get_intermediate_states():
            self.pre_continue_sys_trans_ast(name, pre_dict, post_dict_list, allowable_skill_disjunction, skill_continue_formulas, all_pres)
        allowable_skill_disjunction.extend(self.create_precondition_initial_pres(name, skill))
        not_pre = self.add_not_wrapper(allowable_skill_disjunction)
        not_post = self.add_not_wrapper(self.name2assignment(name, is_prime=True))
        implication = self.add_implication_wrapper(not_pre, not_post)
        allowable_skill_formula = self.add_formula_wrapper(implication)
        return [allowable_skill_formula] + skill_continue_formulas
    
    def create_preconditions_llm(self, name: str, intermediate_transitions: list) -> list:
        """Create the preconditions of a skill in AST sys_trans"""
        allowable_skill_disjunction = self.create_empty_disjunction()
        skill_continue_formulas = []
        all_pres = [pre for pre, _ in intermediate_transitions]
        for pre_dict, post_dict_list in intermediate_transitions:
            self.pre_continue_sys_trans_ast(name, pre_dict, post_dict_list, allowable_skill_disjunction, skill_continue_formulas, all_pres)
        allowable_skill_disjunction.extend(self.create_precondition_initial_pres_llm(name, intermediate_transitions))
        not_pre = self.add_not_wrapper(allowable_skill_disjunction)
        not_post = self.add_not_wrapper(self.name2assignment(name, is_prime=True))
        implication = self.add_implication_wrapper(not_pre, not_post)
        allowable_skill_formula = self.add_formula_wrapper(implication)
        return [allowable_skill_formula] + skill_continue_formulas

    def remove_last_added_skills(self) -> None:
        """Remove the last added skills
        Procedure:
        """
        for _ in range(self.data['new_skills']):
            self.vars['[OUTPUT]'].pop()
            self.asts['[SYS_INIT]'].pop()
            self.skills_data.popitem()
        for _ in range(self.data['env_trans']):
            self.asts['[ENV_TRANS]'].pop()
        for _ in range(self.data['sys_trans']):
            self.asts['[SYS_TRANS]'].pop()
        self.reset_dict_num(self.data)

    def contains_skill(self, formula: list) -> bool:
        """Given a formula AST, return True if it contains the string "skill" in any of its sub-formulas"""
        return self.contains_keyword(formula, 'skill')

    def delete_skills(self, formulas: list, violated_indices: set = None) -> list:
        """Given a list of AST formulas, delte those formulas that contains the string "skill" in any of its sub-formulas"""
        if violated_indices:
            return [formula for i, formula in enumerate(formulas) if not self.contains_skill(formula) or i in violated_indices]
        return [formula for formula in formulas if not self.contains_skill(formula)]
        # return self.filter_list(formulas, lambda formula: not self.contains_skill(formula))
    
    def create_inactivity_without_skills_formulas(self) -> list:
        """Create inactivity without skill formula (!skills -> (x <-> x')
        """
        formulas = []
        for type in ["mobile", "manipulation"]:
            skills = [skill for skill, skill_data in self.skills_data.items() if skill_data['type'] == type]
            symbols = find_symbols_by_objects_type_category(type, self.symbols_data, self.objects_data)
            if skills:
                formulas.append(self.create_inactivity_without_skills_formula_given_skills_symbols(skills, symbols))
        formulas.append(self.create_inactivity_without_skills_formula_given_skills_symbols(skills=list(self.skills_data.keys()), 
                                                                                           symbols=self.controllable_variables))
        return formulas
    
    def modify_inactivity_without_skills_formulas(self) -> None:
        mobile_skills = [skill for skill, skill_data in self.skills_data.items() if skill_data['type'] == "mobile"]
        if not self.MANIPULATION_ONLY and mobile_skills and hasattr(self, 'not_any_mobile_skills_conjunction'):
            self.modify_inactivity_without_skills_formulas_given_skills(self.not_any_mobile_skills_conjunction, mobile_skills)

        manipulation_skills = [skill for skill, skill_data in self.skills_data.items() if skill_data['type'] == "manipulation"]
        if manipulation_skills and hasattr(self, 'not_any_manipulation_skills_conjunction'):
            self.modify_inactivity_without_skills_formulas_given_skills(self.not_any_manipulation_skills_conjunction, manipulation_skills)

        all_skills = list(self.skills_data.keys())
        if all_skills:
            self.modify_inactivity_without_skills_formulas_given_skills(self.not_any_skills_conjunction, all_skills)
    
    def modify_inactivity_without_skills_formulas_given_skills(self, conjunction: list, skills: list) -> None:
        """Modify the inactivity without skills formulas given translate_skill_llm
        """
        if conjunction is None:
            print("conjunction is None")
            return None
        new_skill_conjunction = []
        for skill in skills:
            new_skill_conjunction.append(self.add_not_wrapper(self.name2assignment(skill)))
        if len(skills) > 1:
            conjunction[0] = self.terminals['conjunction']
            conjunction[1:] = []
            conjunction.extend(new_skill_conjunction)
        elif len(skills) == 1:
            conjunction[:] = new_skill_conjunction[0]
            if DEBUG:
                print("==== Modified inactivity without skills Conjunction ====")
                print(conjunction)
        else:
            raise Exception(f"Too few skills in {skills}")
        return None


    def create_inactivity_without_skills_formula_given_skills_symbols(self, skills, symbols) -> list:
        """Create inactivity without skill formula (!skills -> (x <-> x') given skills and symbols"""
        # skills = self.vars['[OUTPUT]']
        not_skills_conjunction = self.create_empty_conjunction()
        for skill in skills:
            not_skills_conjunction.append(self.add_not_wrapper(self.name2assignment(skill)))
        inactivity_conjunction = self.create_empty_conjunction()
        for var in symbols:
            # if var not in self.uncontrollable_variables:
            inactivity_conjunction.append(self.add_biimplication_wrapper(self.name2assignment(var), self.name2assignment(var, is_prime=True)))
        implication = self.add_implication_wrapper(not_skills_conjunction, inactivity_conjunction)
        return self.add_formula_wrapper(implication)
    
    
    def create_mutual_exclusion_skills_formula(self, is_prime: bool = False) -> list:
        """Create mutual exclusion skills formula (skill0 & !skill1) | (!skill0 & skill1) | (!skill0 & !skill1)"""
        skills = self.vars['[OUTPUT]']
        if is_prime:
            skills = self.add_prime_to_list(skills)
        mutual_exclusion_disjunction = self.create_empty_disjunction()
        for i in range(len(skills)):
            mutual_exclusion_conjunction = self.create_empty_conjunction()
            mutual_exclusion_conjunction.append(self.name2assignment(skills[i]))
            for j in range(len(skills)):
                if i == j:
                    continue
                mutual_exclusion_conjunction.append(self.add_not_wrapper(self.name2assignment(skills[j])))
            mutual_exclusion_disjunction.append(mutual_exclusion_conjunction)
        not_skills_conjunction = self.create_empty_conjunction()
        for skill in skills:
            not_skills_conjunction.append(self.add_not_wrapper(self.name2assignment(skill)))
        mutual_exclusion_disjunction.append(not_skills_conjunction)
        return self.add_formula_wrapper(mutual_exclusion_disjunction)

    def create_uncontrollable_inputs_inactivity_during_skillls_formula(self) -> list:
        """Create uncontrollable inputs inactivity during skills formula 
        (pre & skill & post', forall skill in Y, forall(pre, post) in intermediate_states(skill)) 
        -> (var <-> var', forall var in uncontrollable inputs)
        """
        pre_post_pair = self.create_empty_disjunction()
        for skill_name, skill in self.skills_data.items():
            for pre_dict, post_dict_list in skill.get_intermediate_states():
                for post_dict in post_dict_list:
                    if post_dict not in skill.get_final_posts():
                        pre_post_pair.append(self.create_pre_skill_post_conjunction(skill_name, pre_dict, post_dict))
        uncontrollable_inactivity = self.create_empty_conjunction()
        for var in self.uncontrollable_variables:
            uncontrollable_inactivity.append(self.add_biimplication_wrapper(self.name2assignment(var), self.name2assignment(var, is_prime=True)))
        implication = self.add_implication_wrapper(pre_post_pair, uncontrollable_inactivity)
        return self.add_formula_wrapper(implication)


    def generate_env_trans_hard(self) -> None:
        """Reset the env_trans_hard AST based on current vars and skills"""
        env_trans_hard: list = self.asts[self.properties["env_trans_hard"]]
        # env_trans_hard = self.delete_skills(env_trans_hard, self.violated_env_trans_hard_indices)
        env_trans_hard = env_trans_hard[:-1] # delete the skill mutual exclusion assumption
        self.modify_inactivity_without_skills_formulas()
        env_trans_hard.append(self.create_mutual_exclusion_skills_formula())
        if STATIC_WORLD:
            env_trans_hard.append(self.create_uncontrollable_inputs_inactivity_during_skillls_formula())
        self.asts[self.properties["env_trans_hard"]] = env_trans_hard
        
    def generate_sys_trans_hard(self) -> None:
        """Reset the sys_trans_hard AST based on current vars and skills"""
        sys_trans_hard = self.asts[self.properties["sys_trans_hard"]]
        sys_trans_hard = self.delete_skills(sys_trans_hard)
        sys_trans_hard.append(self.create_mutual_exclusion_skills_formula(is_prime=True))
        self.asts[self.properties["sys_trans_hard"]] = sys_trans_hard

    # ==== LLM verification related function ==== #
    def add_llm_repaired_skills(self, skills: dict) -> None:
        """Add skills to ASTs wrt the pre and post conditions, in the format of LLM repair response
        Inputs:
            skills: dict: a dictionary of skills, where the key is the skill name and the value is a list of intermediate transitions
        Outputs:
            None, modify the ASTs in-place to add the new skills
        """
        for name, intermediate_transitions in skills.items():
            if True: print("Adding skill: ", name)
            self.vars['[OUTPUT]'].append(name)
            self.asts['[SYS_INIT]'].append(self.add_formula_wrapper(self.add_not_wrapper(self.name2assignment(name)))) 
            postconditions = self.create_postconditions_llm(name, intermediate_transitions)
            preconditions = self.create_preconditions_llm(name, intermediate_transitions)
            self.asts[self.properties['env_trans']].extend(postconditions)
            self.asts[self.properties['sys_trans']].extend(preconditions)
        self.generate_env_trans_hard()
        self.generate_sys_trans_hard()
        return None
    
    def translate_skill_llm(self, skill_name: str, skill: list) -> Skill:
        """Translate the intermediate transitions of a skill to a Skill object"""
        intermediate_transitions: list = []
        for pre_list, post_list_list in skill:
            new_pre = list2dict_bool_true(pre_list)
            # pop the key not in inputs
            new_pre = self.project_dict_wrt_sets(new_pre, self.vars[self.properties["input"]])
            new_post_list = []
            if isinstance(post_list_list[0], str):
                post_list_list = [post_list_list]
            for post_list in post_list_list:
                new_post = list2dict_bool_true(post_list)
                new_post = self.project_dict_wrt_sets(new_post, self.vars[self.properties["input"]])
                if new_post != new_pre:
                    new_post_list.append(new_post)
                # new_post_list.append(list2dict_bool_true(post_list))
            if new_post_list:
                intermediate_transitions.append((new_pre, new_post_list))
        if intermediate_transitions == []:
            print(f"The skill {skill_name} has no intermediate transitions")
            return None
        intial_pres = self.get_initial_pres_from_intermediate_transitions(intermediate_transitions)
        final_posts = self.get_final_posts_from_intermediate_transitions(intermediate_transitions)
        info = {"name": skill_name, "intermediate_states": intermediate_transitions, "initial_preconditions": intial_pres, "final_postconditions": final_posts}
        return Skill(info)
    
    def project_dict_wrt_sets(self, d: dict, set: list) -> dict:
        """Project the dictionary wrt the set"""
        return {k: v for k, v in d.items() if k in set}

    def get_all_pres_from_intermediate_transitions(self, intermediate_transitions: list) -> list:
        """Get all preconditions from the intermediate transitions"""
        all_pres = []
        for pre, post_list in intermediate_transitions:
            all_pres.append(pre)
        return all_pres
    
    def get_all_posts_from_intermediate_transitions(self, intermediate_transitions: list) -> list:
        """Get all postconditions from the intermediate transitions"""
        all_posts = []
        for pre, post_list in intermediate_transitions:
            all_posts.extend(post_list)
        return all_posts
    
    def get_initial_pres_from_intermediate_transitions(self, intermediate_transitions: list) -> list:
        """Get the initial preconditions from the intermediate transitions"""
        all_posts = self.get_all_posts_from_intermediate_transitions(intermediate_transitions)
        all_pres = self.get_all_pres_from_intermediate_transitions(intermediate_transitions)
        initial_pres = [pre for pre in all_pres if pre not in all_posts]
        return initial_pres
    
    def get_final_posts_from_intermediate_transitions(self, intermediate_transitions: list) -> list:
        """Get the final postconditions from the intermediate transitions"""
        all_posts = self.get_all_posts_from_intermediate_transitions(intermediate_transitions)
        all_pres = self.get_all_pres_from_intermediate_transitions(intermediate_transitions)
        final_posts = [post for post in all_posts if post not in all_pres]
        return final_posts
        
    
    def add_llm_violation(self, violated_observation: dict, violated_assumptions_idx: list, true_only: bool = False) -> None:
        """Add the violated observation and assumptions to the ASTs
        Inputs:
            violated_observation: dict: the violated observation
            violated_assumptions_idx: list: a list of the indices violated assumptions
        Outputs:
            None, modify the ASTs in-place to add the violated observation and assumptions
        """
        if "violations" not in violated_observation.keys():
            violated_observation_new = {}
            violated_observation_new["violations"] = [violated_observation]
            violated_observation = violated_observation_new
        for violation in violated_observation["violations"]:
            if "violation_formula" in violation.keys():
                self.relax_assumption_given_violation_formula(violated_assumptions_idx, violation["violation_formula"])
                continue
            X, Y, X_p = state2dict_bool(inp_state=violation["previous_input_state"], inputs=self.vars[self.properties["input"]], true_only=true_only), \
                        state2dict_bool(inp_state=violation["previous_output_state"], inputs=self.vars[self.properties["output"]], true_only=true_only), \
                        state2dict_bool(inp_state=violation["current_input_state"], inputs=self.vars[self.properties["input"]], true_only=true_only)
            if DEBUG:
                print("previous_input_state: ", violation["previous_input_state"])
                print("inputs: ", self.vars[self.properties["input"]])
                print("previous_output_state: ", violation["previous_output_state"])
                print("current_input_state: ", violation["current_input_state"])
                print("X: ", X)
                print("Y: ", Y)
                print("X_p: ", X_p)
                print("exit due to debug")
                exit()
            self.relax_assumption(violated_assumptions_idx, [], X, Y, X_p, true_only=true_only)
        return None
    
    def get_feedback_from_synthesis(self) -> dict:
        """Get the feedback from the synthesis
        Outputs:
            dict: a dictionary of counterstrategy from synthesis
        """
        # self.check_realizability(plot=False, counterstrategy=True)
        result_err, counterstrategy, states, transitions = self.synthesize_counterstrategy()
        return counterstrategy, states, transitions
    
    def get_informalized_feedback_from_synthesis(self, counterstrategy: str, states: dict, transitions: dict) -> str:
        """Get the informalized feedback from the synthesis"""
        raise NotImplementedError("The function get_informalized_feedback_from_synthesis is not implemented")

    def get_feedback_from_unrealizability_analysis(self, new_skills: dict, counterstrategy: str, states: dict, transitions: dict, iteration: int, log_time: dict) -> str:
        """Get the feedback from the unrealizability analysis
        Inputs:
            new_skills: dict: a dictionary of new skills, each skill is a Skill object
            counterstrategy: str: a string of the counterstrategy from the unrealizability analysis
            states: a dict that maps from state number to ((env_goal, sys_goal), [true propositions])
        Outputs:
            str: a string of the feedback from the unrealizability analysis
        """
        if DEBUG:
            print("==== Counterstrategy ====")
            print(counterstrategy)
            print("==== states ====")
            print(states)
            print(f"type of states: {type(states)}")
            print("==== transitions ====")
            print(transitions)
            print(f"type of transitions: {type(transitions)}")
            # print("==== Exit due to debug ====")
            # sys.exit(0)
        start_time = time.time()
        feedback, later_check_transition = self.get_feedback_from_deadlock_analysis(new_skills, states, transitions)
        time_used = time.time() - start_time
        print(f"Time used for deadlock analysis - {iteration}: {time_used}")
        log_time[f"deadlock_analysis-iter-{iteration}"] = time_used
        log_time[f"deadlock_analysis-iter-{iteration}-feedback"] = feedback
        # if feedback != "":
        #     return feedback
        start_time = time.time()
        feedback += self.get_feedback_from_livelock_analysis(new_skills, states, transitions)
        time_used = time.time() - start_time
        print(f"Time used for liveness analysis - {iteration}: {time_used}")
        log_time[f"liveness_analysis-iter-{iteration}"] = time_used
        log_time[f"liveness_analysis-iter-{iteration}-feedback"] = feedback
        # write time to file
        # with open(log, 'a') as f:
        #     f.write(f"Time used for liveness analysis - {iteration}: {time_used}\n")
        if not feedback:
        # if False:
            start_time = time.time()
            feedback = self.get_feedback_from_deadlock_analysis_later_check_transitions_only(new_skills, states, transitions, later_check_transition)
            time_used = time.time() - start_time
            log_time[f"deadlock_analysis-iter-{iteration}"] += time_used
            log_time[f"deadlock_analysis-iter-{iteration}-feedback"] += feedback
        return feedback
    
    def get_feedback_from_deadlock_analysis(self, new_skills: dict, states: dict, transitions: dict) -> tuple:
        """Get the feedback from the deadlock analysis
        Inputs:
            new_skills: dict: a dictionary of new skills, each skill is a Skill object
        Outputs:
            str: a string of the feedback from the deadlock analysis
        """
        feedback = ""
        self.generate_z3_variables_and_solver()
        self.z3_sys_hard_constraints = self.generate_z3_hard_constraints()
        if DEBUG:
            print("==== z3 hard constraints ====")
            for constraint in self.z3_sys_hard_constraints:
                print(constraint)
            # print(self.z3_sys_hard_constraints)
            # print("==== Exit due to debug ====")
            # sys.exit(0)
        if DEBUG:
            print("==== states ====")
            print(states)
            print("==== transitions ====")
            print(transitions)
        later_check_transition = []
        for pre_state, post_state_list in transitions.items():
            pre_state_skill = self.get_skill_from_state(pre_state, states)
            for post_state in post_state_list:
                if post_state in transitions.keys() and transitions[post_state]:
                    # Only check the transition where post_state is a sink state
                    continue
                if pre_state_skill not in new_skills.keys(): # Only check the transition with new skills
                    # if DEBUG: print(f"The pre_state_skill {pre_state_skill} is not new skill")
                    later_check_transition.append([pre_state, post_state])
                    continue
                violated_constraints_indices = self.check_pre_post_violate_hard_constraints(pre_state, post_state, states)
                if len(violated_constraints_indices) > 0:
                    # violated_skill = self.get_skill_from_state(pre_state, states)
                    # if violated_skill not in new_skills.keys():
                    #     print(f"The violated skill {violated_skill} is not new skill")
                    #     continue
                    violated_constraints = self.get_violated_sys_hard_constraints_from_indices(violated_constraints_indices)
                    if DEBUG:
                        print("==== violated constraints ====")
                        for violated_constraint in violated_constraints:
                            print(violated_constraint)
                        # print(violated_constraints)
                        print("==== violated skill ====")
                        print(pre_state_skill)
                        # print("==== Exit due to debug ====")
                        # sys.exit(0)
                    feedback += f"""{pre_state_skill} violates the hard constraints: "{','.join(violated_constraints)}"\n"""
                    post_state_rank = self.get_sys_rank_from_state(post_state, states)
                    post_state_goal = self.get_sys_live_asts()[post_state_rank]
                    post_state_unsat_goal_feedback = ' '.join(self.generate_structuredslugsplus_formula(post_state_goal, is_prime=False))
                    feedback += f"""The new skills cannot satisfy the liveness goal {post_state_unsat_goal_feedback}\n"""
        return feedback, later_check_transition

    def get_feedback_from_deadlock_analysis_later_check_transitions_only(self, new_skills: dict, states: dict, transitions: dict, transitions_to_check: list) -> str:
        feedback = ""
        for pre_state, post_state in transitions_to_check:
            pre_state_skill = self.get_skill_from_state(pre_state, states)
            violated_constraints_indices = self.check_pre_post_violate_hard_constraints(pre_state, post_state, states)
            if len(violated_constraints_indices) > 0:
                # violated_skill = self.get_skill_from_state(pre_state, states)
                # if violated_skill not in new_skills.keys():
                #     print(f"The violated skill {violated_skill} is not new skill")
                #     continue
                violated_constraints = self.get_violated_sys_hard_constraints_from_indices(violated_constraints_indices)
                if DEBUG:
                    print("==== violated constraints ====")
                    for violated_constraint in violated_constraints:
                        print(violated_constraint)
                    # print(violated_constraints)
                    # print("==== violated skill ====")
                    # print(pre_state_skill)
                    # print("==== Exit due to debug ====")
                    # sys.exit(0)
                feedback += f"""{pre_state_skill} violates the hard constraints: "{','.join(violated_constraints)}"\n"""
                post_state_rank = self.get_sys_rank_from_state(post_state, states)
                post_state_goal = self.get_sys_live_asts()[post_state_rank]
                post_state_unsat_goal_feedback = ' '.join(self.generate_structuredslugsplus_formula(post_state_goal, is_prime=False))
                feedback += f"""The new skills cannot satisfy the liveness goal {post_state_unsat_goal_feedback}\n"""
                return feedback
        return feedback


    
    def get_feedback_from_livelock_analysis(self, new_skills: dict, states: dict, transitions: dict) -> str:
        """Get the feedback from the livelock analysis
        Inputs:
            new_skills: dict: a dictionary of new skills, each skill is a Skill object
        Outputs:
            str: a string of the feedback from the livelock analysis
        """
        # return self.liveness_analysis_iterative_synthesis(new_skills, states, transitions)
        # return self.liveness_analysis_cycle(new_skills, states, transitions)
        # return self.liveness_analysis_cycle_johnson_algo(new_skills, states, transitions)
        return self.liveness_analysis_sinking_cycle(new_skills, states, transitions)
    
    def liveness_analysis_iterative_synthesis(self, new_skills: dict, states: dict, transitions: dict) -> str:
        """Liveness analysis using iterative synthesis
        Inputs:
            new_skills: dict: a dictionary of new skills, each skill is a Skill object
        Outputs:
            str: a string of the feedback from the liveness analysis
        """
        feedback = ""
        liveness_goals_asts = self.get_sys_live_asts()
        if True:
            print("==== liveness goals ====")
            for goal in self.get_sys_live_asts():
                print(goal)
            # print("==== Exit due to debug ====")
            # sys.exit(0)
        self.asts[self.properties["sys_liveness"]] = []
        for i, goal in enumerate(liveness_goals_asts):
            self.asts[self.properties["sys_liveness"]].append(goal)
            realizable = self.check_realizability(plot=False, counterstrategy=False)
            if not realizable:
                if i == 0:
                    feedback += f"The liveness goal {' '.join(self.generate_structuredslugsplus_formula(goal, is_prime=False))} cannot be satisfied.\n"
                elif i == 1:
                    feedback += f"The liveness goal {' '.join(self.generate_structuredslugsplus_formula(goal, is_prime=False))} cannot be satisfied after satisfying the previous liveness goal {' '.join(self.generate_structuredslugsplus_formula(liveness_goals_asts[i-1], is_prime=False))}.\n"
                else:
                    feedback += f"The liveness goal {' '.join(self.generate_structuredslugsplus_formula(goal, is_prime=False))} cannot be satisfied after satisfying the previous liveness goals {', '.join([' '.join(self.generate_structuredslugsplus_formula(liveness_goals_asts[j], is_prime=False)) for j in range(i)])}.\n"
                break
        self.asts[self.properties["sys_liveness"]] = liveness_goals_asts
        return feedback
    
    def get_feedback_from_unsat_live_goals(self, unsat_live_goals: list) -> str:
        """Get the feedback from the unsat live goals"""
        feedback = ""
        liveness_goals_asts = self.get_sys_live_asts()
        if DEBUG:
            print("==== unsat liveness goals ====")
            print("unsat_live_goals: ", unsat_live_goals)
            print("liveness_goals_asts: ", liveness_goals_asts)
            print("==== Exit due to debug ====")
            sys.exit(0)
        for i, goal in enumerate(liveness_goals_asts):
            if i in unsat_live_goals:
                if i == 0:
                    feedback += f"The liveness goal {' '.join(self.generate_structuredslugsplus_formula(goal, is_prime=False))} cannot be satisfied.\n"
                elif i == 1:
                    feedback += f"The liveness goal {' '.join(self.generate_structuredslugsplus_formula(goal, is_prime=False))} cannot be satisfied after satisfying the previous liveness goal {' '.join(self.generate_structuredslugsplus_formula(liveness_goals_asts[i-1], is_prime=False))}.\n"
                else:
                    feedback += f"The liveness goal {' '.join(self.generate_structuredslugsplus_formula(goal, is_prime=False))} cannot be satisfied after satisfying the previous liveness goals {', '.join([' '.join(self.generate_structuredslugsplus_formula(liveness_goals_asts[j], is_prime=False)) for j in range(i)])}.\n"
                if DEBUG:
                    print("==== feedback ====")
                    print(feedback)
                    print("==== Exit due to debug ====")
                    sys.exit(0)
                break
        return feedback
    
    def get_feedback_from_live_goals(self, unsat_live_goals: list, sat_live_goals: list) -> str:
        """Get the feedback from unsat and sat live goals in sinking scc"""
        feedback = ""
        liveness_goals_asts = self.get_sys_live_asts()
        if DEBUG:
            print("==== unsat liveness goals ====")
            print("unsat_live_goals: ", unsat_live_goals)
            print("liveness_goals_asts: ", liveness_goals_asts)
            print("==== Exit due to debug ====")
            sys.exit(0)
        unsat_live_str: str = f"{', '.join([' '.join(self.generate_structuredslugsplus_formula(liveness_goals_asts[j], is_prime=False)) for j in unsat_live_goals])}"
        sat_live_str: str = f"{', '.join([' '.join(self.generate_structuredslugsplus_formula(liveness_goals_asts[j], is_prime=False)) for j in sat_live_goals])}"
        if sat_live_goals:
            feedback += f"The liveness goals {unsat_live_str} cannot be satisfied after satisfying {sat_live_str}."
        else:
            feedback += f"The liveness goals {unsat_live_str} cannot be satisfied."
        return feedback

    def liveness_analysis_sinking_cycle(self, new_skills: dict, states: dict, transitions: dict) -> str:
        """Liveness analysis by identifying a sinking cycle"""
        feedback = ""
        if DEBUG:
            print("==== states ====")
            print(states)
            for state, rank_and_propositions in states.items():
                print(f"state: {state}, rank: {rank_and_propositions[0]}, propositions: {rank_and_propositions[1]}")
            print("==== transitions ====")
            print(transitions)
            for state, next_states in transitions.items():
                print(f"state: {state}, next_states: {next_states}")
            print("==== Exit due to debug ====")
            sys.exit(0)
        unsat_live_goals, sat_live_goals = self.unsat_live_goals_analysis_scc(states, transitions, new_skills)
        if unsat_live_goals or sat_live_goals:
            feedback += self.get_feedback_from_live_goals(unsat_live_goals, sat_live_goals)
            # return feedback
        else:
            feedback=""
        return feedback
        

    def unsat_live_goals_analysis_scc(self, states: dict, transitions: dict, new_skills: dict) -> list:
        """Detect the sinking cycle in liveness analysis"""
        sccs = self.get_strongly_connected_components_tarjan(states, transitions)
        terminal_sccs = self.find_terminal_sccs(sccs, states, transitions)
        goals = self.find_live_goals_in_sinking_sccs_with_cycle_free_prefix(terminal_sccs, states, transitions, new_skills)
        if goals:
            unsat_live_goals, sat_live_goals = goals[0], goals[1]
        else:
            unsat_live_goals, sat_live_goals = None, None
        return unsat_live_goals, sat_live_goals

    def find_live_goals_in_sinking_sccs_with_cycle_free_prefix(self, terminal_sccs: list, states: dict, transitions: dict, new_skills: dict) -> list:
        """Find live goal the sinking sccs with cycle-free prefix"""
        for terminal_scc in terminal_sccs:
            live_goals = self.find_live_goals_in_sinking_scc_with_cycle_free_prefix(terminal_scc, states, transitions, new_skills)
            if live_goals is not None:
                return live_goals
        return None

    def find_live_goals_in_sinking_scc_with_cycle_free_prefix(self, terminal_scc: set, states: dict, transitions: dict, new_skills: dict) -> list:
        """Find the sinking cycle in a terminal strongly connected component
        Procedure:
            0. Ensure that terminal scc contains cycle (i.e. not a singleton)
            1. Find a path from the initial state 0 to any state X in the terminal scc
            2. Check if the path contains new skills
            3. Obtain the live goals wanting to be satisfied in the scc
            4. Obtain the live goals wanting to be, but are unsatisfied, in the scc
            5. Check if the scc contains new skills
            6. Return live goals (unsat, sat) in the scc, or None if the scc is a singleton with no selfloop, or scc+prefix containts no new skils
        """
        # 0. Ensure that terminal scc is not singleton with no selfloop transitions
        if len(terminal_scc) == 1:
            if list(terminal_scc)[0] not in transitions.keys() or not transitions[list(terminal_scc)[0]]:
                return None
        
        # # 1. Find a path from the initial state 0 to any state X in the terminal scc
        # path_0_to_X = self.bfs_path_to_terminal_scc("0", terminal_scc, states, transitions)
        # if not path_0_to_X:
        #     raise Exception("No path from the initial state to the terminal scc")
        
        # # 2. Check if the path contains new skills
        # prefix_contains_new_skills: bool = self.states_contains_new_skills(path_0_to_X, new_skills, states, transitions)

        # 3. Obtain the live goals wanting to be satisfied in the scc
        want_live_goals = self.get_want_live_goals_from_scc(terminal_scc, states, transitions)

        # 4. Obtain the live goals satisfied by scc
        sat_live_goals = self.get_sat_live_goals_from_scc(terminal_scc, states, transitions)

        unsat_live_goals = [live_goal for live_goal in want_live_goals if live_goal not in sat_live_goals]

        # # 5. Check if the scc contains new skills
        # scc_contains_new_skills = self.states_contains_new_skills(terminal_scc, new_skills, states, transitions)

        # # 2. Find problmetic liveness goals not satisfied in the path
        # unsat_live_goals_in_path_0_to_X = self.get_unsat_live_goals_from_cycle(path_0_to_X, states, transitions)
        
        # X = path_0_to_X[-1]

        # # 4. Obtain the problematic liveness goals not satisfied in the scc
        # # also 5. Check if the scc contains new skills
        # unsat_live_goals_in_scc, scc_contains_new_skills = self.get_unsat_live_goals_from_scc_given_unsat_goals_from_prefix(terminal_scc, states, transitions, new_skills, unsat_live_goals_in_path_0_to_X)
        
        # if prefix_contains_new_skills or scc_contains_new_skills or True:
        #     return unsat_live_goals, sat_live_goals
        # else:
        #     return None
        
        return unsat_live_goals, sat_live_goals

        # # 2. Find a cycle inside the SCC that starts from X
        # if cycle_X is None:
        #     cycle_X = self.find_cycle_in_terminal_scc_from_state(X, terminal_scc, states, transitions)
        # if not cycle_X:
        #     return None
        # return path_0_to_X + cycle_X[1:]
    
    def get_unsat_live_goals_from_scc_given_unsat_goals_from_prefix(self, scc: set, states: dict, transitions: dict, new_skills: dict, problematic_liveness_goals_in_path: list) -> tuple:
        """Get the problematic liveness goals from the scc"""
        unsat_live_goals = copy.deepcopy(problematic_liveness_goals_in_path)
        scc_contains_new_skills = False
        
        # 1. Check whether sccs contains new skills
        for state in scc:
            skill = self.get_skill_from_state(state, states)
            if skill in new_skills.keys():
                scc_contains_new_skills = True
                break
        
        # 2. Check whether the transitions from scc satisfy some live goals
        for state in scc:
            curr_live = self.get_sys_rank_from_state(state, states)
            for next_state in transitions[state]:
                assert next_state in scc, f"Next state {next_state} is not in the scc {scc}, i.e. the scc is not sinking"
                next_live = self.get_sys_rank_from_state(next_state, states)
                if curr_live != next_live:
                    # curr_live is sat
                    unsat_live_goals = [goal for goal in unsat_live_goals if goal != curr_live]
        return unsat_live_goals, scc_contains_new_skills

    def states_contains_new_skills(self, cycle: list, new_skills: dict, states: dict, transitions: dict) -> bool:
        """Check if the cycle contains new skills"""
        for state in cycle:
            skill = self.get_skill_from_state(state, states)
            if skill in new_skills.keys():
                return True
        return False
        
    def bfs_path_to_terminal_scc(self, start: str, terminal_scc: set, states: dict, transitions: dict) -> list:
        """BFS to find a path from the start state to the terminal scc"""
        queue = deque([[start]])
        visited = {start}
        while queue:
            path = queue.popleft()
            last = path[-1]
            if last in terminal_scc:
                return path
            if last not in transitions:
                continue
            for next_state in transitions[last]:
                if next_state not in visited:
                    visited.add(next_state)
                    queue.append(path + [next_state])
        return None

    def find_terminal_sccs(self, sccs: list, states: dict, transitions: dict) -> list:
        """Find the terminal strongly connected components"""
        node2scc = {} # map from node to the index of its scc
        for i, scc in enumerate(sccs):
            for node in scc:
                node2scc[node] = i
        
        scc_outgoing = set() # set of sccs that have outgoing transitions

        # Check every edge in the graph
        for node, next_states in transitions.items():
            for next_state in next_states:
                if node2scc[node] != node2scc[next_state]:
                    scc_outgoing.add(node2scc[node])
        
        # Find the terminal sccs
        terminal_sccs = []
        for i, scc in enumerate(sccs):
            if i not in scc_outgoing:
                terminal_sccs.append(scc)
        
        return terminal_sccs

    def get_strongly_connected_components_tarjan(self, states: dict, transitions: dict) -> list:
        """Get the strongly connected components using Tarjan's algorithm"""
        sccs = []
        index_counter = [0]
        stack = []
        lowlink = {}
        index = {}
        on_stack = defaultdict(lambda: False)

        def strongconnect(v: str) -> None:
            # depth index for node v
            index[v] = index_counter[0]
            lowlink[v] = index_counter[0]
            index_counter[0] += 1
            stack.append(v)
            on_stack[v] = True

            # Forall successors of v
            if v in transitions:
                for w in transitions[v]:
                    if w not in index:
                        # Successor w has not yet been visited; recurse on it
                        strongconnect(w)
                        lowlink[v] = min(lowlink[v], lowlink[w])
                    elif on_stack[w]:
                        # Successor w is in stack S and hence in the current SCC
                        lowlink[v] = min(lowlink[v], index[w])
            
            # If v is a root node, pop the stack and generate an SCC
            if lowlink[v] == index[v]:
                scc = set()
                while True:
                    w = stack.pop()
                    on_stack[w] = False
                    scc.add(w)
                    if w == v:
                        break
                sccs.append(scc)
            return None
        
        for v in states.keys():
            if v not in index:
                strongconnect(v)
        return sccs
        

    def liveness_analysis_cycle(self, new_skills: dict, states: dict, transitions: dict) -> str:
        """Liveness analysis using cycle detection
        Inputs: 
            new_skills: dict: a dictionary of new skills, each skill is a Skill object
            states: dict: a map from state to a list of its true propositions
            transitions: dict: a map from state to a list of its next states
        Outputs:
            str: a string of the feedback from the liveness analysis
        """
        feedback = ""
        transitions_removed_self_loop = self.remove_self_loop(transitions)
        if True:
            print("==== states ====")
            for state, rank_and_propositions in states.items():
                print(f"state: {state}, rank: {rank_and_propositions[0]}, propositions: {rank_and_propositions[1]}")
            print("==== transitions ====")
            for state, next_states in transitions.items():
                print(f"state: {state}, next_states: {next_states}")
            print("==== transitions_removed_self_loop ====")
            for state, next_states in transitions_removed_self_loop.items():
                print(f"state: {state}, next_states: {next_states}")
            # print("==== Exit due to debug ====")
            # sys.exit(0)
        
        cycle = self.detect_cycle_in_liveness(states, transitions_removed_self_loop)
        if cycle is None:
            cycle = self.detect_cycle_in_liveness(states, transitions)
        if cycle:
            if True:
                print("==== cycle ====")
                print(cycle)
                # print("==== Exit due to debug ====")
                # sys.exit(0)
            problemetic_liveness_goals = self.get_unsat_live_goals_from_cycle(cycle, states, transitions)
            if True:
                print("==== problemetic liveness goals ====")
                print(problemetic_liveness_goals)
                # print("==== Exit due to debug ====")
                # sys.exit(0)
            feedback = self.get_feedback_from_problematic_liveness_goals(problemetic_liveness_goals)
            return feedback
        else:
            raise Exception("No cycle found in liveness analysis")
        
    def get_unsat_live_goals_from_cycle(self, cycle: list, states: dict, transitions: dict) -> list:
        """Get the liveness goals from the cycle
        Inputs:
            cycle: list: a list of states in the cycle
            states: dict: a map from state to a list of its true propositions
            transitions: dict: a map from state to a list of its next states
        Outputs:
            list: a list of liveness goals
        """
        problemetic_liveness_goals = list(range(len(self.get_sys_live_asts())))
        prev_live = self.get_sys_rank_from_state(cycle[0], states)
        if DEBUG:
            print("==== problemetic liveness goals ====")
            print(problemetic_liveness_goals)
            print("==== prev live ====")
            print(prev_live)
            print("==== Exit due to debug ====")
            sys.exit(0)
        
        for state in cycle[1:]:
            curr_live = self.get_sys_rank_from_state(state, states)
            # print(f"live for state {state}: {curr_live}")
            if curr_live != prev_live:
                problemetic_liveness_goals = [goal for goal in problemetic_liveness_goals if goal != prev_live]
                prev_live = curr_live
        return problemetic_liveness_goals
    
    def get_want_live_goals_from_scc(self, scc: set, states: dict, transitions: dict) -> list:
        """Get the liveness goals wanting to be satisfied in the scc"""
        want_live_goals = []
        for state in scc:
            live = self.get_sys_rank_from_state(state, states)
            if live not in want_live_goals:
                want_live_goals.append(live)
        return want_live_goals
    
    def get_sat_live_goals_from_scc(self, scc: set, states: dict, transitions: dict) -> list:
        """Get the liveness goals satisfied by the scc"""
        sat_live_goals = []
        n: int = len(self.get_sys_live_asts())
        for state in scc:
            live = self.get_sys_rank_from_state(state, states)
            for next_state in transitions[state]:
                next_live = self.get_sys_rank_from_state(next_state, states)
                if live != next_live:
                    if live < next_live:
                        curr_live = live
                        while curr_live < next_live:
                            if curr_live not in sat_live_goals:
                                sat_live_goals.append(curr_live)
                            curr_live += 1
                        # if live not in sat_live_goals:
                        #     sat_live_goals.append(live)
                        #     ...
                    else:
                        curr_live = live
                        while curr_live <= n:
                            if curr_live not in sat_live_goals:
                                sat_live_goals.append(curr_live)
                            curr_live += 1
                        curr_live = 0
                        while curr_live < next_live:
                            if curr_live not in sat_live_goals:
                                sat_live_goals.append(curr_live)
                            curr_live += 1                              
        return sat_live_goals
            
    def get_sys_rank_from_state(self, state: str, states: dict) -> int:
        """Get the rank of the state
        Inputs:
            state: str: the state
            states: dict: a map from state to ((env_rank, sys_rank), [true propositions])
        """
        rank = states[state][0]
        rank_no_parans = rank.strip("()")
        rank_parts = rank_no_parans.split(",")
        second_num = int(rank_parts[1].strip())
        return second_num
        
    def remove_self_loop(self, transitions: dict) -> dict:
        """Remove the self loop in the transitions"""
        transitions_removed_self_loop = {}
        for state, next_states in transitions.items():
            transitions_removed_self_loop[state] = [next_state for next_state in next_states if next_state != state]
        return transitions_removed_self_loop

    def detect_cycle_in_liveness(self, states: dict, transitions: dict) -> list:
        colors = {state: "WHITE" for state in states.keys()}
        parents = {}
        
        def dfs_cycle_detection(u: str) -> list:
            """DFS cycle detection"""
            colors[u] = "GRAY"
            if u not in transitions:
                transitions[u] = []
            for v in transitions[u]:
                if colors[v] == "GRAY":
                    return build_cycle_path(u, v, parents)
                elif colors[v] == "WHITE":
                    parents[v] = u
                    cycle = dfs_cycle_detection(v)
                    if cycle:
                        return cycle
            colors[u] = "BLACK"
            return None
        
        def build_cycle_path(u: str, v: str, parents: dict) -> list:
            """Build the cycle path, given that u -> v, and v is in the cycle"""
            cycle = [u]
            curr = u
            while curr != v:
                curr = parents[curr]
                cycle.append(curr)
            while parents[curr] != None:
                curr = parents[curr]
                cycle.append(curr)
            cycle.reverse()
            return cycle
        
        for state in states.keys():
            if colors[state] == "WHITE":
                parents[state] = None
                cycle = dfs_cycle_detection(state)
                if cycle:
                    return cycle
        return None


    def generate_z3_variables_and_solver(self) -> None:
        """Generate the Z3 variables and solver for the unrealizability analysis"""
        self.z3_vars = self.generate_z3_variables()
        self.z3_solver = Solver()
        return None

    def generate_z3_variables(self) -> dict:
        """Generate the Z3 variables for the unrealizability analysis"""
        z3_vars = {var: Bool(var) for var in self.get_inputs_and_outputs()}
        z3_vars_prime = {f"{var}'": Bool(f"{var}_p") for var in self.get_inputs_and_outputs()}
        return {**z3_vars, **z3_vars_prime}

    def generate_z3_hard_constraints(self) -> list:
        """Generate the system hard constraints in z3 format"""
        sys_hard_constraints = []
        for formula in self.get_sys_trans_hard_asts():
            sys_hard_constraints.append(self.generate_z3_formula(formula))
        return sys_hard_constraints


    def check_pre_post_violate_hard_constraints(self, pre: str, post: str, states: dict) -> list:
        """Check if the pre and post violate the hard constraints
        Inputs:
            pre: str: the precondition state
            post: str: the postcondition state
            states: dict: a map from state to a list of its true propositions
        Outputs:
            list: a list of violated constraints indices
        """
        violated_constraints_indices = []
        pre_post_constraint = self.generate_pre_post_constraint(pre, post, states)
        self.z3_solver.reset()
        self.z3_solver.add(pre_post_constraint)
        for i, constraint in enumerate(self.z3_sys_hard_constraints):
        # for constraint in self.z3_sys_hard_constraints:
            if DEBUG:
                # print("==== pre post constraint ====")
                # print(pre_post_constraint)
                print(f"==== checking {i} system hard constraint ====")
                print(constraint)
                # print(self.z3_solver.check(constraint))
                # print("==== Exit due to debug ====")
                # sys.exit(0)
            self.z3_solver.push()
            self.z3_solver.add(constraint)
            result = self.z3_solver.check()
            if result == unsat:
                violated_constraints_indices.append(i)
            self.z3_solver.pop()
        return violated_constraints_indices

    def generate_pre_post_constraint(self, pre, post, states):
        """Generate the constraint for pre and post"""
        pre = self.generate_state_constraint(pre, states)
        post = self.generate_state_constraint(post, states, isPrime = True)
        return And(pre, post)
    
    def generate_state_constraint(self, state, states, isPrime = False):
        """Generate the constraint for a state"""
        _, propositions = states[state]
        if DEBUG:
            print("==== propositions ====")
            print(propositions)
            print("==== Exit due to debug ====")
            sys.exit(0)
        constraints = []
        if isPrime:
            vars = self.get_inputs_and_outputs_prime()
            propositions = [f"{prop}'" for prop in propositions]
        else:
            vars = self.get_inputs_and_outputs()
        for var in vars:
            if var in propositions:
                constraints.append(self.z3_vars[var])
            else:
                constraints.append(Not(self.z3_vars[var]))
        return And(constraints)
        

    def get_skill_from_state(self, state: str, states: dict) -> str:
        """Get the skill from the state
        Inputs:
            state: str: the state
            states: dict: a map from state to a list of its true propositions
        Outputs:
            str: the skill name
        """
        _, propositions = states[state]
        for skill in self.skills_data.keys():
            if skill in propositions:
                return skill
        return None
    
    def get_violated_sys_hard_constraints_from_indices(self, violated_constraints_indices: list) -> list:
        """Get the violated system hard constraints from the indices, in structuredslugs format"""
        return [self.get_sys_hard_constraint_from_index(i) for i in violated_constraints_indices]

    def get_sys_hard_constraint_from_index(self, index: int) -> str:
        """Get the system hard constraint from the index, in structuredslugs format"""
        return ' '.join(self.generate_structuredslugsplus_formula(self.get_sys_trans_hard_asts()[index], is_prime=False))

    def rename_skills_given_llm_response(self, skills: dict) -> None:
        """Rename skills wrt the skills in compiler
        Inputs:
            compiler: Compiler
            new_skills: dict
        """
        cnt = len(self.get_skills())
        for old_name in list(skills.keys()):
            if old_name not in self.get_skills():
                continue
            skill = skills.pop(old_name)
            name = f"skill{cnt}"
            while name in self.get_skills():
                cnt += 1
                name = f"skill{cnt}"
            skill.name = name
            skill.info['name'] = name
            skills[name] = skill
            cnt += 1
        return None
    
    # ==== Add and remove skills ==== #
    def add_skills(self, skills: dict) -> None:
        """Add skills to ASTs wrt the pre and post conditions
        
        Args:
            skills (dict): A dictionary of skills, where the key is the skill name and the value is the skill object
        """
        self.remove_last_added_skills()

        # Add new skills
        # cnt = len(self.vars['[OUTPUT]'])    
        for name, skill in skills.items():
            if DEBUG: print("Adding skill: ", name)
            #[self.terminals['formula'], [self.terminals['unary'], [self.terminals['not'], ('!',)], [self.terminals['assignment'], name]]])
            self.vars['[OUTPUT]'].append(name)
            self.asts['[SYS_INIT]'].append(self.add_formula_wrapper(self.add_not_wrapper(self.name2assignment(name)))) 
            postconditions = self.create_postconditions(name, skill)
            preconditions = self.create_preconditions(name, skill)
            # print("postcondition: ", postconditions)
            # print("precondition: ", preconditions)
            self.asts['[ENV_TRANS]'].extend(postconditions)
            self.asts['[SYS_TRANS]'].extend(preconditions)
            self.data['env_trans'] += len(postconditions)
            self.data['sys_trans'] += len(preconditions)
        self.data['new_skills'] = len(skills)
        skills = self.change_skill_objects_to_dicts(skills)
        self.skills_data.update(skills)

        # Change the env_trans_hard and sys_trans_hard to remove last added skills and incorporate new added skills
        self.generate_env_trans_hard()
        self.generate_sys_trans_hard()

    def remove_skills(self) -> None:
        self.remove_last_added_skills()
        self.generate_env_trans_hard()
        self.generate_sys_trans_hard()

    def change_skill_objects_to_dicts(self, skills: dict) -> dict:
        """Change skill objects to dictionaries
        Given a dictionary of (skill_name, skill: Skill), 
        convert the objects in the skill to dictionaries
        """
        skills_dict = dict()
        for skill_name, skill in skills.items():
            skills_dict[skill_name] = self.skill_object2dict(skill)
        return skills_dict
    
    def skill_object2dict(self, skill: Skill) -> dict:
        """Change a new skill object to a dictionary"""
        skill_dict = dict()
        skill_info = skill.get_info()
        skill_dict["name"] = skill_info["name"]
        skill_dict["intermediate_states"] = skill_info["intermediate_states"]
        skill_dict["initial_preconditions"] = skill_info["initial_preconditions"]
        skill_dict["final_postconditions"] = skill_info["final_postconditions"]
        if "original_skill" in skill_info.keys():
            original_skill = find_true_symbols(skill_info["original_skill"])[0]
            skill_dict["original_skill"] = original_skill
            skill_dict["primitive_skill"] = self.skills_data[original_skill]["primitive_skill"] # will be changed
            skill_dict["type"] = self.skills_data[original_skill]["type"]
            skill_dict["goal_type"] = self.skills_data[original_skill]["goal_type"]
        else:
            # Set type of skill
            skill_dict["type"] = get_skill_type(skill_data=skill_dict, objects_data=self.objects_data)
        # skill_dict["goal"] = self.skills_data[original_skill]["goal"] # not necessarily correct
        return skill_dict



# ================================================================ #

# =============== Helper functions =============== #
    def reset_after_successful_repair(self) -> None:
        """Reset the ASTs and the data_new_skills"""
        self.reset_dict_num(self.data)
        # self.not_allowed_repair_curr_inp_ls = []

    def add_prime_to_list(self, lst: list) -> list:
        """Given a list of strings, return a list of strings with ' appended to each string"""
        return [name + "'" for name in lst]

    def add_not_wrapper(self, sub: list) -> list:
        """Given subformula sub, return !sub in AST"""
        return [self.terminals['unary'], [self.terminals['not'], ('!',)], sub]
    
    def add_implication_wrapper(self, pre: list, post: list) -> list:
        """Return pre -> post in AST"""
        return [self.terminals['implication'], pre, post]
    
    def add_biimplication_wrapper(self, left: list, right: list) -> list:
        """Return left <-> right in AST"""
        return [self.terminals['biimplication'], left, right]

    def add_formula_wrapper(self, sub: list) -> list:
        """wrap sub by formula"""
        return [self.terminals['formula'], sub]
    
    def add_conjunction_wrapper(self, sub: list) -> list:
        """wrap sub by conjunction"""
        res = [self.terminals['conjunction']]
        res.extend(sub)
        return res
    
    def add_disjunction_wrapper(self, sub: list) -> list:
        """wrap sub by disjunction"""
        res = [self.terminals['disjunction']]
        res.extend(sub)
        return res

    def change_Xp_names(self, X_p: dict) -> dict:
        new_X_p = {}
        for key, val in X_p.items():
            new_X_p[f"{key}'"] = val
        return new_X_p

    def dict_to_ast(self, conjunction: list, env: dict, is_prime: bool = False, true_only: bool = False) -> None:
        """Append the env dict to conjunction in AST form in-place"""
        if is_prime:
            env = self.change_Xp_names(env)
        for key, value in env.items():
            if value:
                conjunction.append([self.terminals['assignment'], key])
            else:
                if not true_only:
                    conjunction.append([self.terminals['unary'], [self.terminals['not'], ('!',)], [self.terminals['assignment'], key]])

    def dict_to_ast_formulas(self, formulas: list, env: dict) -> None:
        """Create formulas for env dict, append to the formulas list"""
        for key, value in env.items():
            formula = [self.terminals['formula']]
            if value:
                formula.append([self.terminals['assignment'], key])
            else:
                formula.append([self.terminals['unary'], [self.terminals['not'], ('!',)], [self.terminals['assignment'], key]])
            formulas.append(formula)

    def print_ast(self, ast: list, depth: int = 0) -> None:
        if not isinstance(ast, list):
            print("  " * depth + str(ast))
            return
        if not isinstance(ast[0], str):
            print("ast[0]: ", ast[0])
            raise Exception("ast[0] should be str")
        print("  " * depth + ast[0])
        for elt in ast[1:]:
            self.print_ast(elt, depth+1)


    def print_asts(self) -> None:
        for property_type in self.structuredslugsplus_property_types:
            print("Printing ASTs for %s" % property_type)
            for ast in self.asts[property_type]:
                self.print_ast(ast, 0)

    def print_ast_env_trans_hard_line2(self) -> None:
        property_type = "[ENV_TRANS_HARD]"
        print("Printing ASTs for %s, line 3" % property_type)
        ast = self.asts[property_type][2]
        self.print_ast(ast, 0)


    def name2assignment(self, name: str, is_prime: bool = False) -> list:
        """Given a name, return its corresponding assignmetn AST"""
        if is_prime:
            name = name + "'"
        return [self.terminals['assignment'], name]
    
    def create_empty_conjunction(self) -> list:
        return [self.terminals['conjunction']]
    
    def create_empty_disjunction(self) -> list:
        return [self.terminals['disjunction']]
    
    def reset_dict_num(self, dict: dict) -> None:
        """Reset dict whose values are numbers"""
        for key, _ in dict.items():
            dict[key] = 0

    def contains_keyword(self, formula: list, keyword: str) -> bool:
        """Given a formula AST, return True if it contains the keyword in any of its sub-formulas"""
        if isinstance(formula, str):
            return keyword in formula
        for sub_formula in formula:
            if self.contains_keyword(sub_formula, keyword):
                return True
        return False

    def contains_keywords(self, formula: list, keywords: list) -> bool:
        """Given a formula AST, return True if it contains all the keywords in any of its sub-formulas"""
        if isinstance(formula, str):
            return all(keyword in formula for keyword in keywords)
        for sub_formula in formula:
            if self.contains_keywords(sub_formula, keywords):
                return True
        return False
    
    def filter_list(self, lst: list, predicate: callable) -> list:
        """Given a list and a predicate, return the list of elements that satisfy the predicate"""
        return [element for element in lst if predicate(element)]
    
    def add_bad_intermediate_transitions(self, bad_trans: list) -> None:
        """Add physically unimplementable intermediate transitions to [NOT_ALLOWED_REPAIR]
        
        Args:
            bad_trans: list of tuples of the form (pre, post)
        """
        for pre_dict, post_dict in bad_trans:
            conjunction = self.create_empty_conjunction()
            self.dict_to_ast(conjunction, pre_dict)
            self.dict_to_ast(conjunction, post_dict, is_prime=True)
            self.asts[self.properties["not_allowed_repair"]].append(self.add_formula_wrapper(self.add_not_wrapper(conjunction)))
        return None


def old_test(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--structuredslug',
                        action='store', dest='filename', required=True)
    args = parser.parse_args()
    print("Generating monitor...")
    compiler = Compiler(args.filename)
    compiler.generate_z3_monitor()
    print("Monitor generated!")
    compiler.add_backup_skills()

def test_transform_asts(input_file: str, filename_json: str = None, output_file: str = None):
    if filename_json is not None:
        file_json = json_load_wrapper(filename_json)
        skills_data = json_load_wrapper(file_json['skills'])
        symbols_data = json_load_wrapper(file_json['symbols'])
        objects_data = json_load_wrapper(file_json['objects'])
        controllabe_variables = find_controllable_symbols(symbols_data, objects_data)
        uncontrollable_variables = find_uncontrollable_symbols(symbols_data, objects_data)
    else:
        skills_data = dict()
        symbols_data = dict()
        objects_data = dict()
        controllabe_variables = []
        uncontrollable_variables = []

    compiler = Compiler(input_file=input_file, 
                        skills_data=skills_data, 
                        symbols_data=symbols_data, 
                        objects_data=objects_data, 
                        controllabe_variables=controllabe_variables,
                        uncontrollable_variables=uncontrollable_variables,)
    if DEBUG: 
        print("vars: ", compiler.get_vars())
        compiler.print_asts()
    if DEBUG:
        print("asts: ")
        print(compiler.asts)
        print("int2bool transform needed: ", compiler.int2bool_transform_needed())
    compiler.transform_asts_int2bool()
    compiler.generate_structuredslugsplus(output_file)

def test_check_realizability(input_file: str, filename_json: str = None):
    compiler = create_compiler(input_file, filename_json)
    # compiler.print_asts()
    # compiler.print_ast_env_trans_hard_line2()
    compiler.check_realizability()

def test_check_realizability_with_readable_strategy(input_file: str, filename_json: str = None):
    compiler = create_compiler(input_file, filename_json)
    
    is_realizable = compiler.check_realizability_with_readable_strategy(plot=False)
    # compiler.synthesize_counterstrategy()
    
    if not is_realizable:
        print("The specification is unrealizable!")
        start_time = time.time()
        result_err, counterstrategy, states, transitions = compiler.synthesize_counterstrategy()
        print("Counterstrategy generated!")
        time_used = time.time() - start_time
        print("time_used: ", time_used)
        print(counterstrategy)

def test_assumption_relaxation(input_file: str, filename_json: str = None):
    compiler = create_compiler(input_file, filename_json)
    inputs = list(compiler.symbols_data.keys())
    outputs = list(compiler.skills_data.keys())
    
    ## ==== Violation 1: Unexpected obstacle behavior (plate/stone) ==== ##
    # violated_assumptions_XYXprime = [17]
    # X_list = ['p_cup_k2', 'p_base_x4', 'p_cone_x3', 'p_block_k1', 'p_plate_k0']
    # Y_list = ['skill4']
    # X_p_list = ['p_cup_k2', 'p_base_x4', 'p_cone_x3', 'p_block_k1', 'p_plate_k3']

    ## ==== Violation 2: Unexpected user input value ==== ##
    # violated_assumptions_XYXprime = [12]
    # X_list = ['p_cup_ee', 'p_base_x2', 'p_cone_x3', 'p_block_k0', 'p_plate_k3']
    # Y_list = ['skill1']
    # X_p_list = ['p_cup_ee', 'p_base_x2', 'p_cone_x3', 'p_block_k0', 'p_plate_k3', 'empty']

    ## ==== Violation 3: Unexpected obstacle behavior (cone) ==== ##
    violated_assumptions_XYXprime = [18]
    X_list = ['p_cup_ee', 'p_base_x4', 'p_cone_x3', 'p_block_k0', 'p_plate_k3']
    Y_list = ['skill1']
    X_p_list = ['p_cup_ee', 'p_base_x4', 'p_cone_x2', 'p_block_k0', 'p_plate_k3']

    X = state2dict_bool(inputs, X_list)
    Y = state2dict_bool(outputs, Y_list)
    X_p = state2dict_bool(inputs, X_p_list)
    compiler.relax_assumption(violated_assumptions_XYXprime, [], X, Y, X_p)
    compiler.generate_structuredslugsplus('tests/spec.structuredslugsplus')
    return None

    

def create_compiler(input_file: str, filename_json: str = None):
    if filename_json is not None:
        file_json = json_load_wrapper(filename_json)
        skills_data = json_load_wrapper(file_json['skills'])
        symbols_data = json_load_wrapper(file_json['inputs_data'])
        objects_data = json_load_wrapper(file_json['objects'])
        locations_data = json_load_wrapper(file_json['locations'])
        user_inputs_data: dict = json_load_wrapper(file_json["user_inputs"])
        controllabe_variables = load_list_values_from_json(file_json['controllable_inputs'])
        uncontrollable_variables = load_list_values_from_json(file_json['uncontrollable_inputs'])
    else:
        skills_data = dict()
        symbols_data = dict()
        objects_data = dict()
        locations_data = dict()
        user_inputs_data = dict()
        controllabe_variables = []
        uncontrollable_variables = []
    if DEBUG:
        print("symbols_data: ", symbols_data)
        print("objects_data: ", objects_data)

   
    grounding: Groundings = Groundings(files_json=file_json, 
                                            objects_data=objects_data, 
                                            locations_data=locations_data, 
                                            user_inputs_data=user_inputs_data,
                                            controller=None,
                                            create_symbols_only=True)
    
    
        
    compiler = Compiler(input_file=input_file,
                            skills_data=skills_data,
                            symbols_data=symbols_data,
                            objects_data=objects_data,
                            controllabe_variables=grounding.controllable_inputs,
                            uncontrollable_variables=grounding.uncontrollable_inputs,
                            controllable_mobile_inputs=grounding.controllable_mobile_inputs,
                            controllable_manipulation_inputs=grounding.controllable_manipulation_inputs,)
    return compiler

def test_contains_controllable_input(input_file: str):
    compiler = Compiler(input_file=input_file, 
                        skills_data=dict(), 
                        symbols_data=dict(), 
                        objects_data=dict(), 
                        controllabe_variables=["x1", "x2"],
                        uncontrollable_variables=[],)
    print(compiler.contains_controllable_input(["Formula", ["Assignment", "x1"]]))

if __name__ == '__main__':
    # objects_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/objects.json')
    # locations_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/locations.json')
    # # print(workspace)
    # symbols = create_symbols_from_objects_and_locations(
    #         objects_data, 
    #         locations_data)
    # skills_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/skills.json')
    argparser = argparse.ArgumentParser()
    argparser.add_argument('-s', '--structuredslugplus', action='store', dest='spec', required=False, default='tests/spec.structuredslugsplus')
    argparser.add_argument('-so', '--structuredslugplusout', action='store', dest='spec_out', required=False, default='tests/spec.structuredslugsplus')
    argparser.add_argument('-f', '--file', action='store', dest='file_json', required=False, default=None)

    # Add a Boolean flag test
    argparser.add_argument('-t', '--test', action='store_true', dest='test', required=False, default=False)

    args = argparser.parse_args()

    if args.test:
        test_transform_asts(args.spec, args.file_json, args.spec_out)
    else:
        test_check_realizability_with_readable_strategy(args.spec, args.file_json)
    # print(args.test)
    # test_contains_controllable_input(args.filename)
    # test_assumption_relaxation(args.spec, args.file_json)