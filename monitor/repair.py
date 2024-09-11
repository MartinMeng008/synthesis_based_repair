#! /usr/bin/env python
import rospy
import sys
import numpy as np
import argparse
from mocomp import Compiler
from tools import (json_load_wrapper,
                   find_controllable_symbols,
                   varlist2prime,
                   varlist2doubleprime,
                   )
repair_dir = '../synthesis_based_repair'
sys.path.insert(0, repair_dir)
from symbolic_repair import run_repair
from skills import Skill
# from grounding import Grounding
import symbolic_repair_msgs.msg
import symbolic_repair_msgs.srv 

DEBUG = False
REPAIR_ONLY = False


class Repair:
    def __init__(self, compiler: Compiler, filename: str, opts: dict, files_json: dict = None, symbolic_repair_only: bool = False) -> None:
        """
        Args:
            compiler: monitor_compiler
            filename: the pure name of 
        """
        self.compiler = compiler
        self.symbolic_repair_only = symbolic_repair_only
        if not self.symbolic_repair_only:
            rospy.init_node('fake_repair_node')
            rospy.wait_for_service('fake_feasibility_check')
        self.opts = opts
        if "max_repair_iter" in self.opts:
            self.max_iter = self.opts["max_repair_iter"]
        else:
            self.max_iter = 50
        self.file_structuredslugsplus = filename
        self.opts["uncontrollable_inputs"] = self.compiler.get_request_inputs()
        print("uncontrollable_inputs: ", self.opts["uncontrollable_inputs"])
        self.opts["terrain_inputs"] = self.compiler.get_terrain_inputs()
        print("terrain_inputs: ", self.opts["terrain_inputs"])
        # self.compiler.update_terrain_vars_dir(self.opts)
        # if DEBUG: print("terrain_vars_dir: ", self.opts["up_terrain_vars"])
        # sys.exit(0)
        uncontrollable_inputs = self.opts["uncontrollable_inputs"]
        self.opts["reactive_variables_current"] = uncontrollable_inputs
        self.opts["reactive_variables"] = uncontrollable_inputs + varlist2prime(uncontrollable_inputs) + varlist2doubleprime(uncontrollable_inputs)
        if "terrain_inputs" in self.opts:
            terrain_inputs = self.opts["terrain_inputs"]
            self.opts["terrain_variables_current"] = terrain_inputs
            self.opts["terrain_variables"] = terrain_inputs + varlist2prime(terrain_inputs) + varlist2doubleprime(terrain_inputs)
            self.opts["terrain_variables_p"] = varlist2prime(terrain_inputs)
            self.opts["terrain_variables_p_dp"] = varlist2prime(terrain_inputs) + varlist2doubleprime(terrain_inputs)
            self.opts["terrain_variables_dp"] = varlist2doubleprime(terrain_inputs)
        
        self.opts["original_skills"] = self.compiler.get_original_skills()
        
        self.feasibility_check = rospy.ServiceProxy('fake_feasibility_check', symbolic_repair_msgs.srv.FeasibilityCheck)
            
    def run_symbolic_repair(self) -> dict:
        """Run the symbolic repair module
        
        Returns:
            skills: dict: a dictionary of suggested new skills
        """
        is_realizable = False
        cnt = 0
        skills = {}
        self.opts['existing_skills'] = self.compiler.get_skills()
        if DEBUG:
            print("self.opts['existing_skills']: ", self.opts['existing_skills'])
            exit(0)
        if DEBUG: breakpoint()
        prev_post_first = self.opts['post_first']
        seed = self.opts["n_seeds"]
        while not is_realizable and cnt <= self.max_iter:
            np.random.seed(seed)
            seed += 1
            self.opts['only_synthesis'] = True
            is_realizable, _ = run_repair(self.file_structuredslugsplus, self.opts)
            print(f"is_realizable at iteration {cnt}: ", is_realizable)
            if is_realizable:
                print("The spec is realizable")
                self.compiler.remove_backup_skills()
                return skills
            self.opts['only_synthesis'] = False
            found_suggestion = False
            while not found_suggestion:
                try:
                    # print("post first? ", self.opts['post_first'])
                    # self.opts['post_first'] = not self.opts['post_first']
                    prev_post_first = self.opts['post_first'] = bool((prev_post_first + cnt) % 2)
                    print("post first? ", self.opts['post_first'])
                    if DEBUG: breakpoint()
                    _, suggestions = run_repair(self.file_structuredslugsplus, self.opts)
                    found_suggestion = True
                except Exception as e:
                    print("Exception: ", e)
                    break
            skills = {}
            if found_suggestion:
                for _, suggestion in suggestions.items():
                    skills[suggestion['name']] = Skill(suggestion, location_inputs=self.compiler.get_location_inputs(), terrain_inputs=self.compiler.get_terrain_inputs())
                self.compiler.remove_backup_skills()
                add_skills_with_reduced_size(skills, self.compiler)
                skill_array_msg = symbolic_repair_msgs.msg.SkillArray()
                for _, skill in skills.items():
                    skill.print_dict()
                    skill.write_to_file(f"{self.opts['new_skills_file']}_{skill.get_name()}.txt")
                    skill_msg = skill.get_skill_msg()
                    if True:
                        print(skill_msg)
                    skill_array_msg.skills.append(skill_msg)
                if self.symbolic_repair_only:
                    return skills
                skill_array_msg.header.stamp = rospy.Time.now()
                try:
                    resp = self.feasibility_check(skill_array_msg)
                    print(resp.feasibilities_checked)
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                
                ## Feasibility check
                # 1. go through each feasibility in resp.feasibilities_checked
                # 2. for each feasibility, 
                #       if feasibility.feasibility is False:
                #           mocomp.add_not_allowed_constraints(skills[feasibility.name])
                # 3. if any skill is not feasible, generate a new spec from AST, and run repair again
                # 4. Otherwise, exist
                bad_new_transitions = []
                bad_skill_names = []
                for feasibility_resp in resp.feasibilities_checked.feasibilities:
                    if not feasibility_resp.feasibility:
                        bad_skill_name = feasibility_resp.name
                        bad_skill_names.append(bad_skill_name)
                        bad_skill = skills[bad_skill_name]
                        # breakpoint()
                        bad_new_transitions.append((bad_skill.init_pres[0], bad_skill.final_posts[0]))
                if len(bad_new_transitions) > 0:
                    self.compiler.add_bad_intermediate_transitions(bad_new_transitions)
                    self.compiler.remove_skills()
                    good_skills = {name: skill for name, skill in skills.items() if name not in bad_skill_names}
                    if len(good_skills) > 0:
                        self.compiler.add_skills(good_skills)
                        self.compiler.reset_after_successful_repair()
                    self.compiler.add_backup_skills()
                    breakpoint()
                    self.compiler.generate_structuredslugsplus(self.file_structuredslugsplus)
                    is_realizable = False
                else:
                    return skills
                    raise Exception("stop here")
            print("----------------------------------")
            print("----------------------------------")
            print(f"Finish {cnt} iteration in the while loop")
            print("skills", list(skills.keys()))
            print("----------------------------------")
            print("----------------------------------")
            if DEBUG: breakpoint()
            # if skills:
            # # Pop old skills from and add new skills to the ASTs, re-generate the spec  
            #     self.compiler.add_skills(skills)
            #     # print("============ New Skills ==========")
            #     print(self.compiler.vars[self.compiler.properties["output"]])
            #     if DEBUG: print("new spec name: ", self.file_structuredslugsplus)
            #     self.compiler.generate_structuredslugsplus(self.file_structuredslugsplus)
                # break
            cnt += 1
        
        return skills
            
def test_symbolic_repair(filename_structuredslugsplus, opts, files=None):
    """Run the repair module
    Args:
        X: dict: environment state, which is supposed to be the env_init of the new specification
    """
    # # Write unsat spec AST
    # print("Writing unsat spec ASTs to file")
    # fid = open("tests/unsat_asts.txt", 'w')
    # fid.write(str(self.compiler.asts))
    # fid.close()

    # Generate relaxed structuredslugsplus based on current ASTs
    # self.filename_structuredslugsplus = f'{self.filename}_{self.repair_cnt}.structuredslugsplus'
    # # print_debug("self.filename_structuredslugsplus in repair:")
    # # print_debug(self.filename_structuredslugsplus)
    if files:
        skills_data = json_load_wrapper(files["skills"])
        symbols_data = json_load_wrapper(files["inputs"])
        objects_data = json_load_wrapper(files["objects"])
        controllable_symbols = find_controllable_symbols(symbols_data, objects_data)
        uncontrollable_symbols = [s for s in symbols_data.keys() if s not in controllable_symbols]
    else:
        skills_data = dict()
        symbols_data = dict()
        objects_data = dict()
        controllable_symbols = []
        uncontrollable_symbols = []


    compiler = Compiler(filename_structuredslugsplus, skills_data, symbols_data, objects_data, controllable_symbols, uncontrollable_symbols)
    # input_file = f"tests/original_{filename_structuredslugsplus.split('/')[-1]}"
    # compiler.generate_structuredslugsplus(input_file)
    # breakpoint()
    input_file = filename_structuredslugsplus
    compiler.add_backup_skills()
    add_skilll_file = f'build/backup_skill_added_{filename_structuredslugsplus.split("/")[-1]}'
    compiler.generate_structuredslugsplus(add_skilll_file)
    # repair = Repair(compiler, add_skilll_file, opts)
    repair = Repair(compiler, add_skilll_file, opts, files_json=files, symbolic_repair_only=REPAIR_ONLY)
    new_skills = repair.run_symbolic_repair()
    # exit(0)

    if True:
        add_skilll_file = f'build/repaired_skill_added_{filename_structuredslugsplus.split("/")[-1].split(".")[0]}.slugsin'
        compiler.generate_slugsin(add_skilll_file)
    # compiler.remove_backup_skills()
    # add_skills_with_reduced_size(new_skills, compiler)
    # self.compiler.add_skills(new_skills)

    # Edit the input file
    # input_file_structuredslugsplus = f'tests/repaired_{filename_structuredslugsplus.split("/")[-1]}'
    # compiler.generate_structuredslugsplus(input_file_structuredslugsplus)

    # Reset the compiler
    # self.compiler = Compiler(self.input_file_structuredslugsplus, self.opts['reactive_variables_current'])
    
    return new_skills

def test_symbolic_repair_wo_add_skills(filename_structuredslugsplus, opts, files=None):
    if files:
        skills_data = json_load_wrapper(files["skills"])
        symbols_data = json_load_wrapper(files["inputs"])
        objects_data = json_load_wrapper(files["objects"])
        controllable_symbols = find_controllable_symbols(symbols_data, objects_data)
        uncontrollable_symbols = [s for s in symbols_data.keys() if s not in controllable_symbols]
    else:
        skills_data = dict()
        symbols_data = dict()
        objects_data = dict()
        controllable_symbols = []
        uncontrollable_symbols = []

    compiler = Compiler(filename_structuredslugsplus, skills_data, symbols_data, objects_data, controllable_symbols, uncontrollable_symbols)
    repair = Repair(compiler, filename_structuredslugsplus, opts, files_json=files)
    new_skills = repair.run_symbolic_repair()
    return new_skills


def add_skills_with_reduced_size(new_skills: dict, compiler: Compiler) -> None:
    """Add skills with reduced size to the compiler
        The goal of this function is to replace self.compiler.add_skills(new_skills)
        
        Side-effects:
            remove skills in new_skills that are not needed
    """
    for skill_name in list(new_skills.keys()):
        removed_skill = new_skills.pop(skill_name)
        compiler.add_skills(new_skills)
        print("Skills to check: ", compiler.get_skills())
        is_realizable = compiler.check_realizability()
        print("is_realizable: ", is_realizable)
        if not is_realizable:
            new_skills[skill_name] = removed_skill
    rename_new_skills(compiler, new_skills)
    compiler.add_skills(new_skills)

def rename_new_skills(compiler: Compiler, skills: dict) -> None:
        """Rename the new skills to avoid name conflicts"""
        compiler.remove_skills()
        cnt = len(compiler.get_skills())
        for old_name in list(skills.keys()):
            skill = skills.pop(old_name)
            name = f"skill_{cnt}"
            assert name not in compiler.get_skills()
            skill.name = name
            skill.info['name'] = name
            skills[name] = skill
            cnt += 1

if __name__ == "__main__":
    # rospy.init_node('fake_repair_node')
    # rospy.wait_for_service('fake_feasibility_check')
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--spec',
                        action='store', dest='filename', required=True, help='Input file name in structuredslugsplus')
    parser.add_argument('-o', '--opts', action='store', dest='opts', required=True, help='Options in json')
    # parser.add_argument('-o', "--output", action='store', dest='output', required=False, help='skill json file', default=None)
    # parser.add_argument('-f', "--files_json", action='store', dest='files_json', required=True, help='files_json')
    parser.add_argument('-a', "--add_skills", action='store_true', dest='add_skills', required=False, help='add skills', default=False)
    parser.add_argument('-ro', "--repair_only", action='store_true', dest='repair_only', required=False, help='repair only', default=False)
    args = parser.parse_args()
    sys.setrecursionlimit(100000) 
    if not args.repair_only:
        REPAIR_ONLY = False
    else:
        REPAIR_ONLY = True
    if args.add_skills:
        test_symbolic_repair(args.filename, json_load_wrapper(args.opts))
    else:
        test_symbolic_repair_wo_add_skills(args.filename, json_load_wrapper(args.opts))

            