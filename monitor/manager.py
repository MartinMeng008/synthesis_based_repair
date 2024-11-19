#!/usr/bin/env python3

import sys
import argparse
import math
import copy
import time
from collections import defaultdict
from mocomp import Compiler
from repair_base import (
    Repair,   
)
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
    list_minus,
    )

repair_dir = '../synthesis_based_repair'
sys.path.insert(0, repair_dir)
from skills import Skill

DEBUG = False

class Manager:
    """Manager class for the monitor and repair system.
        Inputs:
            a json file containing all relevant file names:
                1. a spec with integer inputs
                2. a set of possible terrain states
                3. a skill abstraction dictionary
        Maintains:
            1. M_Y that maps a terrain state to a set of relevant skills
            2. M_I that maps a terrain state to a set of physically infeasible transitions
        Outputs:
            a set of new skills that repair the spec
    """
    def __init__(self, filename_json: str) -> None:
        self._setup(filename_json)
        self.dirs = [(0,1), (0,-1), (-1,0), (1,0)] # 4 directions

        self.compiler: Compiler = Compiler(input_file=self.spec_int,
                                           skills_data=dict(),
                                           symbols_data=dict(),
                                           objects_data=dict(),
                                           controllabe_variables=[],
                                           uncontrollable_variables=[],
                                           opts=self.opts)
        
        self.main()
        
    def _setup(self, filename_json: str) -> None:
        file_json: dict = json_load_wrapper(filename_json)
        self.spec_int: str = file_json["input_file_int"]
        self.spec_bool: str = file_json["input_file_bool"]
        self.modulo_spec: str = file_json["modulo_spec_structuredslugsplus"]
        self.repaired_spec: str = file_json["output_file_structuredslugsplus"]
        self.repaired_spec_slugsin = file_json["output_file_slugsin"]
        self.opts: dict = json_load_wrapper(file_json["opts"])
        if DEBUG:
            print(self.opts)
            sys.exit(0)
        self.num_grid: int = self.opts["num_grid"]
        self.ws_range: int = int(math.sqrt(self.num_grid))
        self.num_terrain_types: int = self.opts["num_terrain_types"]
        json_data: dict = json_load_wrapper(file_json["json_data"])
        self.skills_data = json_data["skill_list"]
        self.terrain_states = json_data["terrain_states_list"]
        self.request_states = json_data["request_states_list"]
        print("==== Setup data ====")
        print(f"grid size: {self.ws_range}x{self.ws_range}")
        print(f"num of terrain types: {self.num_terrain_types}")
        print(f"num of terrain states: {len(self.terrain_states)}")
        print(f"num of request states: {len(self.request_states)}")
        print("====================")
        if DEBUG:
            sys.exit(0)
            print("==== Exit due to debugging ====")

    def main(self) -> None:

        # 0. Create mappings
        self.create_mappings()

        # 1. Transform the spec to boolean
        # 1.1. Add assumptions about possible terrain states and possible request states
        self.compiler.add_terrain_states_as_env_trans_hard(self.terrain_states)
        self.compiler.add_request_states_as_env_trans_hard(self.request_states)
        # self.compiler.add_change_constraints(self.opts)
        self.generate_bool_spec()

        # 2. Make a copy of the compiler
        self.repair_compiler: Compiler = copy.deepcopy(self.compiler)
        
        # 3. Remove terrains, requests, and add repair constraints
        self.repair_compiler.remove_terrains_in_vars_and_asts()
        self.repair_compiler.remove_requests_in_vars_and_asts()
        self.repair_compiler.add_change_constraints(self.opts)
        self.repair_compiler.add_not_allowed_repair(self.opts)

        FIRST_TIME = True

        # 4. For each terrain state, repair the spec
        for terrain_state in self.terrain_states:
            for request_state in self.request_states:
                # 4.1. Get relevant skills
                skills2transitions = self.m_y(terrain_state)
                if DEBUG:
                    print("skills2transitions:\n", skills2transitions)
                    sys.exit(0)

                # 4.2. Get relevant infeasible transitions
                infeasible_transitions = self.terrain_state2invalid_trans(terrain_state)

                
                # self.repair_compiler: Compiler = copy.deepcopy(self.compiler)
                
                # 4.3. Remove skills
                # keep track of time used for removing skills and terrains
                if DEBUG: 
                    print("Time for removing skills and terrains:")
                    start_time = time.time()
                self.repair_compiler.remove_skills_in_vars_and_asts()
                if DEBUG: print("--- %s seconds ---" % (time.time() - start_time))

                if DEBUG:
                    print("Before adding skills:")
                    self.repair_compiler.generate_structuredslugs(self.modulo_spec)
                    sys.exit(0)
                
                # 4.4. Add relevant skills
                skills = self._form_skills(skills2transitions)
                if DEBUG:
                    print("Skills to be added:")
                    for _, skill in skills.items():
                        skill.print_dict()
                    sys.exit(0)
                self.repair_compiler.add_skills_no_intermediate_states(skills)

                # 4.5. Add infeasible transitions
                self.repair_compiler.add_infeasible_trans_to_not_allowed_repair(infeasible_transitions)
                
                if DEBUG:
                    infeasible_transitions = [(0,0,0,1), (0,0,1,0)]
                    self.repair_compiler.add_infeasible_trans_to_not_allowed_repair(infeasible_transitions)
                    self.repair_compiler.generate_structuredslugsplus(self.modulo_spec)
                    sys.exit(0)

            # # 4.6. Go through each request state and set it as liveness goal
            # for request_state in self.request_states:
                if DEBUG:
                    print("Request state:", request_state)
                
                # 4.6.1. Add liveness goal
                self.repair_compiler.add_liveness_goal(self.request2robot(request_state))
                
                # 4.6.2. Add backup skills
                self.repair_compiler.add_backup_skills()
                self.repair_compiler.generate_structuredslugsplus(self.modulo_spec)
                if DEBUG:
                    print("==== Exit due to debugging ====")
                    sys.exit(0)

                # 4.6.3. Repair
                repair = Repair(compiler=self.repair_compiler, 
                                filename=self.modulo_spec, 
                                opts=self.opts, 
                                symbolic_repair_only=self.opts["symbolic_repair_only"])
                print(f"==== Repairing for terrain state: {terrain_state}, request state: {request_state} ====")
                print(f"==== Relevant skills: {skills2transitions} ====")
                if True:
                    start_time = time.time()
                new_skills = repair.run_symbolic_repair()
                # self.repair_compiler.remove_backup_skills()

                if True:
                    print("Time for repair:")
                    print("--- %s seconds ---" % (time.time() - start_time))
                
                if len(new_skills) > 0:
                    # 4.6.4. Parse news skills to ideal format
                    new_skills, new_M_y = self.parse_new_skills(new_skills, terrain_state)
                    if not self.opts["symbolic_repair_only"]:
                        # 4.6.5. Perform physical check, todo
                        raise NotImplementedError("Physical check is not implemented yet")

                    # 4.6.6. Add new skills back to the original spec
                    self.rename_skills(self.compiler, new_skills, new_M_y)
                    self.compiler.add_skills_no_intermediate_states(new_skills)
                    self.compiler.reset_after_successful_repair()

                    # 4.6.7. Update M_y
                    self.M_y.update(new_M_y)
                if True:
                    if len(new_skills) > 0:
                        print("==== New skills ====")
                        for _, skill in new_skills.items():
                            skill.print_dict()
                        if DEBUG:
                            self.compiler.generate_structuredslugsplus(self.repaired_spec)
                            self.compiler.generate_slugsin(self.repaired_spec_slugsin)
                            print("==== Exit due to debugging ====")
                            sys.exit(0)
            if DEBUG:
                sys.exit(0)
                print("==== Exit due to debugging ====")
        
        # 5. Generate repaired specs
        self.compiler.generate_structuredslugsplus(self.repaired_spec)
        self.compiler.generate_slugsin(self.repaired_spec_slugsin)
        print("==== M_Y ====")
        print(self.M_y)

    def rename_skills(self, compiler: Compiler, skills: dict, new_M_y: dict) -> None:
        """Rename skills in the compiler
        Inputs:
            compiler: Compiler
            new_skills: dict
            new_M_y: dict
        """
        cnt = len(compiler.get_skills())
        for old_name in list(skills.keys()):
            skill = skills.pop(old_name)
            tuple_key = list(filter(lambda key: new_M_y[key] == old_name, new_M_y))[0]
            name = f"skill_{cnt}"
            assert name not in compiler.get_skills()
            skill.name = name
            skill.info['name'] = name
            skills[name] = skill
            new_M_y[tuple_key] = name
            cnt += 1
                
    def parse_new_skills(self, new_skills: dict, terrain_state: dict) -> dict:
        """Parse new skills to ideal format
        Inputs:
            new_skills: dict of Skill obects
        Outputs:
            1. parsed_new_skills: dict of Skill objects
                each new skill corresponds to a motion primitive
            2. M_y_new that maps a tuple (dir, curr_terrain, next_terrain) to a str of new skill name
                where dir \in (0,1), (0,-1), (-1,0), (1,0)
        Procedure:
            For each skill, each intermediate state:
                Find the corresponding tuple (dir, curr_terrain, next_terrain)
                Check if the tuple already exists in M_y or new_M_y
                If not, add it to new_M_y
                Expand the skill object with initial pre, final post, and intermediate states to correspond to the tuple 
        """
        parsed_new_skills = dict()
        new_M_y = dict()
        for skill_name, skill in new_skills.items():
            for pre_dict, post_dict_list in skill.intermediate_states:
                # assert len(post_dict_list) == 1, "Only one postcondition per intermediate transition is allowed"
                # Instead, we just add the first postcondition
                for i in range(len(post_dict_list)):
                    post_dict = post_dict_list[i]
                    x, y = self.get_x_y_from_state_dict(pre_dict)
                    nx, ny = self.get_x_y_from_state_dict(post_dict)
                    dir_tuple = (nx-x, ny-y)
                    if dir_tuple in self.dirs:
                        break
                assert dir_tuple in self.dirs, f"Invalid direction tuple: {dir_tuple}, x: {x}, y: {y}, nx: {nx}, ny: {ny}"
                next_terrain = self.get_terrain_type_from_xy(terrain_state, nx, ny)
                curr_terrain = self.get_terrain_type_from_xy(terrain_state, x, y)
                tuple_key = (dir_tuple, curr_terrain, next_terrain)
                if tuple_key in self.M_y or tuple_key in new_M_y:
                    continue
                new_M_y[tuple_key] = skill_name
                new_skill_dict = self._motion_primitive_to_skill_dict(skill_name, tuple_key)
                parsed_new_skills[skill_name] = Skill(info=new_skill_dict)
                if DEBUG:
                    print("pre_dict:", pre_dict)
                    print("post_dict:", post_dict)
                    print("dir_tuple:", dir_tuple)
                    print("curr_terrain:", curr_terrain)
                    print("next_terrain:", next_terrain)
                    print("terrain state:", terrain_state)
                    print("is tuple in M_y:", (dir_tuple, curr_terrain, next_terrain) in self.M_y)
                    print("new_skill_dict:", new_skill_dict)
                    print("Exit due to debugging")
                    sys.exit(0)
        return parsed_new_skills, new_M_y
    
    def _motion_primitive_to_skill_dict(self, skill_name: str, tuple_key: tuple) -> dict:
        """Expand a motion primitive to a skill object"""
        dir_tuple, curr_terrain, next_terrain = tuple_key
        skill_dict = dict()
        skill_dict["name"] = skill_name
        skill_dict["initial_preconditions"] = []
        skill_dict["final_postconditions"] = []
        skill_dict["intermediate_states"] = []
        for x in range(self.ws_range):
            for y in range(self.ws_range):
                nx, ny = x + dir_tuple[0], y + dir_tuple[1]
                if nx < 0 or nx >= self.ws_range or ny < 0 or ny >= self.ws_range:
                    continue
                pre_dict = self._form_input_state_dict_from_xy(x, y)
                post_dict = self._form_input_state_dict_from_xy(nx, ny)
                self.update_state_dict_with_terrain_binary_representation(pre_dict, x, y, curr_terrain)
                self.update_state_dict_with_terrain_binary_representation(pre_dict, nx, ny, next_terrain)
                skill_dict["initial_preconditions"].append(pre_dict)
                skill_dict["final_postconditions"].append(post_dict)
                skill_dict["intermediate_states"].append([pre_dict, [post_dict]])
                if DEBUG:
                    print("x, y, nx, ny:", x, y, nx, ny)
                    print("tuple_key:", tuple_key)
                    print("pre_dict:", pre_dict)
                    print("post_dict:", post_dict)
                    print("Exit due to debugging")
                    sys.exit(0)
        return skill_dict
    
    def _form_input_state_dict_from_xy(self, x: int, y: int) -> dict:
        """Form an input state dictionary from x, y, and terrain type"""
        state_dict = dict()
        for i in range(self.ws_range):
            state_dict[f"x{i}"] = False
            state_dict[f"y{i}"] = False
        state_dict[f"x{x}"], state_dict[f"y{y}"] = True, True
        return state_dict
    
    def update_state_dict_with_terrain_binary_representation(self, state_dict: dict, x: int, y: int, terrain_type: int) -> None:
        """Update state_dict with terrain type"""
        terrain_vars_list: list = self.compiler.int_to_bool_vars[self._form_terrain_input(x, y)]
        # Use the binary representation of terrain_type to update state_dict
        for terrain_var in terrain_vars_list:
            state_dict[terrain_var] = bool(terrain_type % 2)
            terrain_type = terrain_type // 2

        if DEBUG:
            print("terrain_vars_list: ", terrain_vars_list)
            print("state_dict: ", state_dict)
            print("Exit due to debugging")
            sys.exit(0)
        return None



    def request2robot(self, request_state: dict) -> dict:
        """Parse request_state as a robot state"""
        robot_state = dict()
        for key, value in request_state.items():
            key = key[0]
            robot_state[key] = value
        return robot_state

    def _form_skills(self, skills2transitions: dict) -> dict:
        """Form a dictionary of skills to be added to the compiler
        Inputs:
            skills2transitions: dict
        Outputs:
            skills: dict
        """
        skills = dict()
        skill_name_counter = 0
        for skill_name, transitions in skills2transitions.items():
            if DEBUG:
                print("Skill name:", skill_name)
                print("Transitions:", transitions)
                sys.exit(0)
            skill_name = f"skill_{skill_name_counter}"
            skill_name_counter += 1
            initial_preconditions: list = []
            # pre_post_pair: list = []
            final_postconditions: list = []
            intermediate_states: list = []
            for trans in transitions:
                pre_dict, post_dict = self._form_pre_and_post_dict_from_transitions(trans)
                if DEBUG:
                    print("pre:", pre_dict)
                    print("post:", post_dict)
                    sys.exit(0)
                # pre_post_pair.append((pre_dict, post_dict))
                # if pre_dict not in initial_preconditions:
                initial_preconditions.append(pre_dict)
                final_postconditions.append(post_dict)
                intermediate_states.append([pre_dict, [post_dict]])
            info: dict = dict()
            info["name"] = skill_name
            info["initial_preconditions"] = initial_preconditions
            info["final_postconditions"] = final_postconditions
            info["intermediate_states"] = intermediate_states
            skills[skill_name] = Skill(info)    
        return skills

    def _form_pre_and_post_dict_from_transitions(self, trans) -> tuple:
        """Form pre and post dictionaries from transitions
        Inputs:
            trans: tuple of (x, y, nx, ny)
        Outputs:
            pre_dict: dict
            post_dict: dict
        """
        x,y,nx,ny = trans
        pre_dict = dict()
        for i in range(self.ws_range):
            pre_dict[f"x{i}"] = False
            pre_dict[f"y{i}"] = False
        pre_dict[f"x{x}"], pre_dict[f"y{y}"] = True, True


        post_dict = dict()
        for i in range(self.ws_range):
            post_dict[f"x{i}"] = False
            post_dict[f"y{i}"] = False
        post_dict[f"x{nx}"], post_dict[f"y{ny}"] = True, True       
        return pre_dict, post_dict

    def create_mappings(self) -> None:
        """Create mappings M_Y and M_I
        M_Y maps a terrain state to a set of relevant skills
        M_I maps a terrain state to a set of physically infeasible transitions
        M_y maps a tuple (dir, curr_terrain, next_terrain) to a str of skill name
            where dir \in (0,1), (0,-1), (-1,0), (1,0)
        """
        # M_y: (dir, curr_terrain, next_terrain) -> skill_name
        self.M_y = dict()

        # M_i: (dir, curr_terrain, next_terrain) -> Boolean
        self.M_i = dict()

        self.create_M_y_mapping()
        if DEBUG:
            print(self.M_y)
            sys.exit(0)
    
    def m_y(self, terrain_state: dict) -> dict:
        """Given a terrain state, return a mapping from relevant skills to transitions"""
        skills2transitions = defaultdict(list)
        for x in range(self.ws_range):
            for y in range(self.ws_range):
                for dir in self.dirs:
                    nx = x + dir[0]
                    ny = y + dir[1]
                    if nx < 0 or nx >= self.ws_range or ny < 0 or ny >= self.ws_range:
                        continue
                    key_tuple = (dir, terrain_state[self._form_terrain_input(x,y)], terrain_state[self._form_terrain_input(nx,ny)])
                    if key_tuple in self.M_y:
                        skills2transitions[self.M_y[key_tuple]].append((x, y, nx, ny))
        return dict(skills2transitions)
    
    def terrain_state2invalid_trans(self, terrain_state: dict) -> dict:
        """Given a terrain state, return a list of invalid transitions"""
        invalid_trans = []
        for x in range(self.ws_range):
            for y in range(self.ws_range):
                for dir in self.dirs:
                    nx = x + dir[0]
                    ny = y + dir[1]
                    if nx < 0 or nx >= self.ws_range or ny < 0 or ny >= self.ws_range:
                        continue
                    key_tuple = (dir, terrain_state[self._form_terrain_input(x,y)], terrain_state[self._form_terrain_input(nx,ny)])
                    if key_tuple in self.M_i and self.M_i[key_tuple]:
                        invalid_trans.append((x, y, nx, ny))
        return invalid_trans

    def _form_terrain_input(self, x: int, y: int) -> str:
        return f"x_{x}_y_{y}_terrain"

    def create_M_y_mapping(self) -> None:
        """Create mapping M_Y
        M_y maps a tuple (dir, curr_terrain, next_terrain) to a str of skill name
            where dir \in (0,1), (0,-1), (-1,0), (1,0)
        """
        for skill_name, skill_data in self.skills_data.items():
            skill_data = skill_data[0]
            skill_dir = self.get_skill_dir(skill_data)
    
            if DEBUG: 
                print(skill_dir)
                sys.exit(0)
            
            key_tuple = (skill_dir, skill_data["current_terrain"], skill_data["next_terrain"])
            if key_tuple in self.M_y:
                raise ValueError(f"Duplicate key in M_y: {skill_name} vs {self.M_y[key_tuple]}")
            self.M_y[key_tuple] = skill_name
        return None
            
    def get_skill_dir(self, skill_data: dict) -> tuple:
        """Return the direction of a skill
        Inputs:
            skill_data: dict
        Outputs:
            dir: tuple
        """
        # print(skill_data)
        return (skill_data["x_prime"] - skill_data["x"], skill_data["y_prime"] - skill_data["y"])
    
    def get_pre_post_dir(self, pre_dict: dict, post_dict: dict) -> tuple:
        """Return the direction of a skill
        Inputs
            pre_dict: dict
            post_dict: dict
        Outputs:
            dir: tuple
        """
        x, y = self.get_x_y_from_state_dict(pre_dict)
        nx, ny = self.get_x_y_from_state_dict(post_dict)
        return (nx-x, ny-y)
    
    def get_x_y_from_state_dict(self, state_dict: dict) -> tuple:
        """Return x, y from state_dict
        Inputs:
            state_dict: dict
        Outputs:
            x, y: tuple
        """
        x, y = None, None
        for key, value in state_dict.items():
            if value:
                if "x" in key:
                    x = int(key[1])
                elif "y" in key:
                    y = int(key[1])
                else:
                    raise ValueError(f"Invalid key: {key}")
        return x, y
    
    def get_terrain_type_from_xy(self, terrain_state: dict, x: int, y: int) -> int:
        """Return the terrain type from x, y
        Inputs:
            terrain_state: dict
            x: int
            y: int
        Outputs:
            terrain_type: int
        """
        return terrain_state[self._form_terrain_input(x, y)]
        


    def generate_bool_spec(self) -> None:
        self.compiler.transform_asts_int2bool()
        self.compiler.generate_structuredslugsplus(self.spec_bool)
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor and Repair system for MOCOMP")
    parser.add_argument("-f", "--filename_json", type=str, help="json file with input file names")
    args = parser.parse_args()
    manager = Manager(args.filename_json)
