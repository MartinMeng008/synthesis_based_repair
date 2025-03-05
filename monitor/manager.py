#!/usr/bin/env python3

import sys
import argparse
import math
import copy
import time
from collections import defaultdict
import rospy

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
    dump_json,
    dict_key_tuple2str,
    dict_key_list2str,
    dict_key_tuple2list2str,
    dict_key_str2tuple,
    create_symbols_from_objects_and_locations,
    find_true_symbols,
    is_manipulation_object,
    is_mobile_object,
    find_controllable_symbols,
    find_uncontrollable_symbols,
    find_controllable_mobile_symbols,
    find_controllable_manipulation_symbols,
    list_minus,
    find_next_skill_name,
    )

repair_dir = '../synthesis_based_repair'
sys.path.insert(0, repair_dir)
from skills import Skill

from symbolic_repair_msgs.msg import AtomicProposition, TerrainAndRequestStates, OnlineRepairResult, FeasibilityArray, Feasibility, SkillPrimitiveArray, SkillPrimitive, State
from symbolic_repair_msgs.srv import OnlineRepairWithNewTerrainAndRequest, OnlineRepairWithNewTerrainAndRequestResponse, PrimitiveFeasibilityCheck, PrimitiveFeasibilityCheckResponse

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
        
        # log number of original skills
        self.log_dict["num_original_skills"] = len(self.compiler.get_skills())
        dump_json(self.log_file, self.log_dict)
        
        # self.offline_repair()
        
    def _setup(self, filename_json: str) -> None:
        file_json: dict = json_load_wrapper(filename_json)
        self.spec_int: str = file_json["input_file_int"]
        self.spec_bool: str = file_json["input_file_bool"]
        self.modulo_spec: str = file_json["modulo_spec_structuredslugsplus"]
        self.mapping_file: str = file_json["mapping_file"]
        self.repaired_spec: str = file_json["output_file_structuredslugsplus"]
        self.repaired_spec_slugsin = file_json["output_file_slugsin"]
        self.log_file: str = file_json["log_file"]
        self.log_dict: dict = dict()
        self.opts: dict = json_load_wrapper(file_json["opts"])
        self.num_grid: int = self.opts["num_grid"]
        self.ws_range: int = int(math.sqrt(self.num_grid))
        self.num_terrain_types: int = self.opts["num_terrain_types"]
        json_data: dict = json_load_wrapper(file_json["json_data"])
        self.skills_data = json_data["skill_list"]
        self.terrain_states = json_data["terrain_states_list"]
        # self.request_states = json_data["request_states_list"]
        print("==== Setup data ====")
        print(f"grid size: {self.ws_range}x{self.ws_range}")
        print(f"num of terrain types: {self.num_terrain_types}")
        print(f"num of terrain states: {len(self.terrain_states)}")
        # print(f"num of request states: {len(self.request_states)}")
        print(f"mapping_file: {self.mapping_file}")
        # print(f"num of skills: {len(self.skills_data)}")
        print("====================")
        
        ## ==== Log info ==== ##
        self.log_dict["grid_size"] = f"{self.ws_range}x{self.ws_range}"
        self.log_dict["num_terrain_types"] = self.num_terrain_types
        self.log_dict["num_terrain_states"] = len(self.terrain_states)
        # total_request_states = set()
        # for terrain_state in self.terrain_states:
        #     total_request_states.update(terrain_state["request_states_list"])
        # self.log_dict["num_request_states"] = len(total_request_states)
        # json dump the log_dict
        dump_json(self.log_file, self.log_dict)

        if DEBUG:
            print("==== Exit due to debugging ====")
            sys.exit(0)
        self._setup_for_runtime_repair(file_json)
        self._setup_for_ros(self.opts)
        return None
    
    def _setup_for_runtime_repair(self, file_json: dict) -> None:
        """Setup for runtime repair"""
        self.runtime_repair_spec: str = file_json["online_output_file_structuredslugsplus"]
        self.runtime_repair_spec_slugsin: str = file_json["online_output_file_slugsin"]
        return None
    
    def _setup_for_ros(self, opts: dict) -> None:
        """Setup for ROS"""
        if opts["symbolic_repair_only"]:
            return None
        else:
            rospy.init_node("repair")
            print("==== ROS node repair initialized ====")
            rospy.wait_for_service("/feasibility_check")
            self.feasibility_check_service = rospy.ServiceProxy("/feasibility_check", PrimitiveFeasibilityCheck)
        return None

            
    def offline_setup(self) -> None:
        """Setup for offline repair"""
        # 0. Create mappings
        self.create_mappings()

        # 1. Transform the spec to boolean
        self.generate_bool_spec(self.compiler)

        # # 2. Make a copy of the compiler
        # self.repair_compiler = self._make_repair_compiler(self.compiler)

        return None
    
    def runtime_setup(self) -> None:
        """Setup for runtime repair"""
        # 0. Setup the runtie repair compiler
        self.compiler = Compiler(input_file=self.repaired_spec,
                                    skills_data=dict(),
                                    symbols_data=dict(),
                                    objects_data=dict(),
                                    controllabe_variables=[],
                                    uncontrollable_variables=[],
                                    opts=self.opts)
        
        # 1. Create mappings
        self.load_mappings()
        
        # # 2. Make a copy of the compiler
        # self.repair_compiler: Compiler = self._make_repair_compiler(self.compiler)
        return None

    def _make_repair_compiler(self, compiler: Compiler) -> Compiler:
        """Make a copy of the compiler for repair"""
        repair_compiler = copy.deepcopy(compiler)
        repair_compiler.remove_terrains_in_vars_and_asts()
        repair_compiler.remove_requests_in_vars_and_asts()
        repair_compiler.add_change_constraints(self.opts)
        repair_compiler.add_not_allowed_repair(self.opts)
        return repair_compiler

    def offline_repair(self) -> None:
        """The main function for offline repair"""
        self.offline_setup()
        # if not self.opts["symbolic_repair_only"]:
        #     rospy.init_node('repair_node')
        #     rospy.wait_for_service('/feasibility_check')
        self.modulo_repair(self.terrain_states)
        return None

    def runtime_repair(self) -> None:
        """The main function for runtime repair"""
        self.runtime_setup()
        if self.opts["symbolic_repair_only"]:
            self.modulo_repair(self.terrain_states)
        else:
            self.repair_service: rospy.Service = rospy.Service("/symbolic_repair/online_repair", OnlineRepairWithNewTerrainAndRequest, self.runtime_repair_callback)
            rospy.spin()

    def runtime_repair_callback(self, req: OnlineRepairWithNewTerrainAndRequest) -> OnlineRepairWithNewTerrainAndRequestResponse:
        """Callback function for runtime repair"""
        print("==== Received request for runtime repair ====")
        print(req)
        terrain_state: dict = self.state_msg2dict(req.terrain_request_states.terrain_state)
        request_state: dict = self.state_msg2dict(req.terrain_request_states.request_state)
        print("==== Runtime Terrain state ====")
        print(terrain_state)
        print("==== Runtime Request state ====")
        print(request_state)
        print("=============")
        self.modulo_repair([terrain_state], [request_state])
        # Generate the repair module spec
        self.repair_compiler.remove_backup_skills_modulo_spec()
        self.repair_compiler.generate_structuredslugsplus(self.runtime_repair_spec)
        self.repair_compiler.generate_slugsin(self.runtime_repair_spec_slugsin)

        # Return the repaired module spec
        response = OnlineRepairWithNewTerrainAndRequestResponse()
        response.repair_result = OnlineRepairResult()
        response.repair_result.repair_needed = True
        response.repair_result.slugsin_location = self.runtime_repair_spec_slugsin

        print("==== Sending response for runtime repair ====")
        print(response)
        return response


    def state_msg2dict(self, state: list) -> dict:
        """Convert a TerrainState message to a dictionary"""
        state_dict: dict = {}
        for inp_prop in state:
            inp_info = inp_prop.atomic_proposition
            state_dict[inp_info[0]] = int(inp_info[1])
        return state_dict
    
    # def request_state_msg2dict(self, request_state: list) -> dict:
    #     """Convert an AtomicProposition message to a dictionary"""
    #     raise NotImplementedError

    def modulo_repair(self, terrain_states: list, request_states: list = None) -> None:
        """The main function for modulo repair"""
        #  For each terrain state, and request state, repair the spec
        # modulo_repair_start_time = time.time()
        unrepairable_terrain_and_request_states = []
        repaired_terrain_and_request_states = []
        no_need_repair_terrain_and_request_states = []
        if False:
            # Hacking runtime 2.2
            self.M_y[((0, 1), 8, 1)] = "skill_38"
            self.M_y[((0, -1), 1, 8)] = "skill_39"
        for idx_terrain_state, terrain_state in enumerate(terrain_states):
            request_states = terrain_state["request_states_list"]
            for idx_request_state, request_state in enumerate(request_states):
                physical_feasible = False
                # modulo_repair_iteration_start_time = time.time()
                symbolic_repair_time_iteration = 0
                physical_checker_time_iteration = 0
                while not physical_feasible:
                    # 1. Get relevant skills
                    skills2transitions = self.m_y(terrain_state)
                    self.add_self_loop_skill_to_skills2transitions(skills2transitions, request_state["xrequest"], request_state["yrequest"])
                    if True:
                        print("skills2transitions:\n", skills2transitions)
                        # sys.exit(0)

                    # 2. Get relevant infeasible transitions
                    infeasible_transitions = self.terrain_state2invalid_trans(terrain_state)

                    # 2.5. Get obstacle constraints
                    obstacle_constraints = self.m_o(terrain_state)
                    if DEBUG:
                        if obstacle_constraints:
                            print("==== Obstacle constraints ====")
                            print(obstacle_constraints)
                            print("====")
                            print("Exit due to debugging")
                            sys.exit(0)
                    if DEBUG:
                        if idx_terrain_state == 1 and idx_request_state == 2:
                            breakpoint()
                    
                    # 3. Remove skills
                    # keep track of time used for removing skills and terrains
                    if DEBUG: 
                        print("Time for removing skills and terrains:")
                        start_time = time.time()
                    self.repair_compiler = self._make_repair_compiler(self.compiler)
                    self.repair_compiler.remove_skills_in_vars_and_asts()
                    self.repair_compiler.has_inacitivity_without_skills = False
                    self.repair_compiler.has_skill_mutual_exclusion = False

                    if DEBUG: print("--- %s seconds ---" % (time.time() - start_time))

                    if DEBUG:
                        print("Before adding skills:")
                        self.repair_compiler.generate_structuredslugsplus(self.modulo_spec)
                        sys.exit(0)
                    
                    # 4. Add relevant skills
                    skills = self._form_skills(skills2transitions)
                    if DEBUG:
                        print("Skills to be added:")
                        for _, skill in skills.items():
                            skill.print_dict()
                        sys.exit(0)
                    self.repair_compiler.add_skills_no_intermediate_states_modulo_spec(skills)

                    # 5. Add infeasible transitions
                    self.repair_compiler.add_infeasible_trans_to_not_allowed_repair(infeasible_transitions)

                    # 5.2. Add obstacle constraints to system hard constraints
                    self.repair_compiler.add_obstacle_constraints_to_sys_hard(obstacle_constraints)
                    
                    if DEBUG:
                        infeasible_transitions = [(0,0,0,1), (0,0,1,0)]
                        self.repair_compiler.add_infeasible_trans_to_not_allowed_repair(infeasible_transitions)
                        self.repair_compiler.generate_structuredslugsplus(self.modulo_spec)
                        sys.exit(0)

                    if DEBUG:
                        print("Request state:", request_state)
                    
                    # 6.1. Add liveness goal
                    self.repair_compiler.add_liveness_goal(self.request2robot(request_state))
                    
                    # 6.2. Add backup skills
                    self.repair_compiler.add_backup_skills_modulo_spec()
                    # breakpoint()
                    self.repair_compiler.generate_structuredslugsplus(self.modulo_spec)
                    if DEBUG:
                        breakpoint()
                        # print("==== Exit due to debugging ====")
                        # sys.exit(0)
                    if True:
                        print("==== skills before repair ====")
                        print(self.repair_compiler.get_skills())
                        print("====")
                    # 6.3. Repair
                    repair = Repair(compiler=self.repair_compiler, 
                                    filename=self.modulo_spec, 
                                    opts=self.opts, 
                                    symbolic_repair_only=self.opts["symbolic_repair_only"])
                    print(f"==== Repairing for terrain state_idx: {idx_terrain_state}, request state_idx: {idx_request_state} ====")
                    print(f"==== Repairing for terrain state: {terrain_state}, request state: {request_state} ====")
                    print(f"==== Relevant skills: {skills2transitions} ====")
                    if DEBUG:
                        if terrain_state == {'x_0_y_0_terrain': 3, 'x_0_y_1_terrain': 4, 'x_0_y_2_terrain': 4, 'x_1_y_0_terrain': 0, 'x_1_y_1_terrain': 0, 'x_1_y_2_terrain': 4, 'x_2_y_0_terrain': 0, 'x_2_y_1_terrain': 0, 'x_2_y_2_terrain': 4}:
                            if request_state == {'xrequest': 0, 'yrequest': 1}:
                                print("==== Target terrain and request states ====")
                                breakpoint()
                                
                    
                    if True:
                        start_time = time.time()
                    new_skills, repair_needed, is_repaired = repair.run_symbolic_repair()
                    # self.repair_compiler.remove_backup_skills()

                    if True:
                        symbolic_repair_time_iteration += time.time() - start_time
                        print("Time for symbolic repair:")
                        print("--- %s seconds ---" % (time.time() - start_time))
                        # print("Time for repair:")
                        # print("--- %s seconds ---" % (time.time() - start_time))
                        self.log_dict[f"terrain_{idx_terrain_state}_request_{idx_request_state}_symbolic_repair_time"] = symbolic_repair_time_iteration
                        dump_json(self.log_file, self.log_dict)
                    self.repair_compiler.remove_backup_skills_modulo_spec()
                    if repair_needed and len(new_skills) == 0:
                        print("==== Unrepairable terrain and request states ====")
                        print("Terrain state:", terrain_state)
                        print("Request state:", request_state)
                        unrepairable_terrain_and_request_states.append((terrain_state, request_state))
                    elif repair_needed and len(new_skills) > 0:
                        repaired_terrain_and_request_states.append((terrain_state, request_state))
                    else:
                        no_need_repair_terrain_and_request_states.append((terrain_state, request_state))

                    if len(new_skills) > 0:
                        if True:
                            print("==== New skills ====")
                            for _, skill in new_skills.items():
                                print(skill)
                            # print("exit due to debugging")
                            # sys.exit(0)
                        # 6.4. Parse news skills to ideal format
                        if DEBUG: breakpoint()
                        new_skills, new_M_y = self.parse_new_skills(new_skills, terrain_state)

                        # 6.5. Minimize new skills to only include the needed ones
                        new_skills, new_M_y = self.minimize_and_rename_new_skills(self.repair_compiler, new_skills, new_M_y)
                        if DEBUG:
                            print("==== New skills ====")
                            for _, skill in new_skills.items():
                                skill.print_dict()
                            print("==== New M_Y ====")
                            print(new_M_y)
                            print("==== Exit due to debugging ====")
                            sys.exit(0)

                        # 6.6. Rename new skills wrt to the global compiler
                        self.rename_skills(self.compiler, new_skills, new_M_y)

                        if not self.opts["symbolic_repair_only"]:
                            # 6.7. Perform physical check
                            if True:
                                start_time = time.time()
                            infeasible_M_y = self.perform_physical_check(new_skills, new_M_y, terrain_state)
                            if True:
                                physical_checker_time_iteration += time.time() - start_time
                                print("Time for physical checker:")
                                print("--- %s seconds ---" % (time.time() - start_time))
                                self.log_dict[f"terrain_{idx_terrain_state}_request_{idx_request_state}_physical_checker_time"] = physical_checker_time_iteration
                                dump_json(self.log_file, self.log_dict)
                            if len(infeasible_M_y) == 0:
                                physical_feasible = True
                            else:
                                # 6.7.1. Add infeasible transitions to M_i
                                for key, _ in infeasible_M_y.items():
                                    self.M_i[key] = True
                        else:
                            physical_feasible = True
                        # 6.8. Add new skills back to the original spec
                        self.compiler.add_skills_no_intermediate_states(new_skills)
                        self.compiler.reset_after_successful_repair()

                        # 6.9. Update M_y
                        self.M_y.update(new_M_y)
                        self.all_new_M_y.update(new_M_y)
                    else:
                        physical_feasible = True

                # Record the time and new skills for each iteration
                # modulo_repair_iteration_end_time = time.time()
                if repair_needed:
                    # self.log_dict[f"terrain_{idx_terrain_state}_request_{idx_request_state}_time"] = modulo_repair_iteration_end_time - modulo_repair_iteration_start_time
                    if len(new_skills) > 0:
                        new_M_y_list_key = dict_key_tuple2list2str(new_M_y)
                        self.log_dict[f"terrain_{idx_terrain_state}_request_{idx_request_state}_new_skills"] = new_M_y_list_key
                        # breakpoint()
                        self.log_dict[f"terrain_{idx_terrain_state}_request_{idx_request_state}_num_new_skills"] = len(new_skills)
                    else:
                        self.log_dict[f"terrain_{idx_terrain_state}_request_{idx_request_state}_num_new_skills"] = 0
                    dump_json(self.log_file, self.log_dict)  
                
                # # Generate module spec
                # self.repair_compiler.generate_slugsin(self.runtime_repair_spec_slugsin)
                # self.repair_compiler.generate_structuredslugsplus(self.runtime_repair_spec)

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
                print("==== Exit due to debugging ====")
                sys.exit(0)
        
        # 5. Generate repaired specs
        self.compiler.generate_structuredslugsplus(self.repaired_spec)
        self.compiler.generate_slugsin(self.repaired_spec_slugsin)
        print("==== M_Y ====")
        print(self.M_y)
        self._check_M_y_soundness(self.compiler, self.M_y)
        print("==== M_I ====")
        print(self.M_i)
        print("==== M_O ====")
        # print(self.M_o)
        for key, value in self.M_o.items():
            print(key, value)
        self.store_mappings(self.M_y, self.M_i, self.M_o)

        if True:
            print("==== unrepairable terrain and request states ====")
            print(unrepairable_terrain_and_request_states)
            print("==================================================")
            # Record the unrepairable terrain and request states
            self.log_dict["unrepairable_terrain_and_request_states"] = unrepairable_terrain_and_request_states
            self.log_dict["repaired_terrain_and_request_states"] = repaired_terrain_and_request_states
            self.log_dict["no_need_repair_terrain_and_request_states"] = no_need_repair_terrain_and_request_states
            self.log_dict["num_unrepairable_terrain_and_request_states"] = len(unrepairable_terrain_and_request_states)
            self.log_dict["num_repaired_terrain_and_request_states"] = len(repaired_terrain_and_request_states)
            self.log_dict["num_no_need_repair_terrain_and_request_states"] = len(no_need_repair_terrain_and_request_states)
            # self.log_dict["total_time"] = time.time() - modulo_repair_start_time
            self.log_dict["total_num_new_skills"] = len(self.compiler.get_skills()) - self.log_dict["num_original_skills"]
            dump_json(self.log_file, self.log_dict)

            # ==== Calculate total symbolic time ==== #
            total_symbolic_repair_time = 0
            for key in self.log_dict:
                if "_symbolic_repair_time" in key:
                    total_symbolic_repair_time += self.log_dict[key]
            self.log_dict["total_symbolic_repair_time"] = total_symbolic_repair_time
            dump_json(self.log_file, self.log_dict)

            # ==== Calculate total physical checker time ==== #
            total_physical_checker_time = 0
            for key in self.log_dict:
                if "_physical_checker_time" in key:
                    total_physical_checker_time += self.log_dict[key]
            self.log_dict["total_physical_checker_time"] = total_physical_checker_time
            dump_json(self.log_file, self.log_dict)
            
        return None
        

    def perform_physical_check(self, new_skills: dict, new_M_y: dict, terrain_state: dict) -> dict:
        """Perform physical check on new skills
        Inputs:
            new_skills: dict
            new_M_y: dict
                M_y_new that maps a tuple (dir, curr_terrain, next_terrain) to a str of new skill name
                where dir \in (0,1), (0,-1), (-1,0), (1,0)
            terrain_state: dict
        Outputs:
            infeasible_M_y: dict:
                a set of infeasible transitions
        """
        Skills = SkillPrimitiveArray()
        Skills.primitives = []
        for key_tuple, skill_name in new_M_y.items():
            skill_primitive = SkillPrimitive()
            skill_primitive.name = skill_name
            skill_primitive.dir = list(key_tuple[0])
            skill_primitive.current_terrain_type = key_tuple[1]
            skill_primitive.next_terrain_type = key_tuple[2]
            Skills.primitives.append(skill_primitive)
        Skills.header.stamp = rospy.Time.now()
        

        try:
            response = self.feasibility_check_service(Skills)
            print("==== Feasibility check response ====")
            print(response)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        return self._parse_response_from_physical_check(response, new_M_y)

    def _parse_response_from_physical_check(self, response: PrimitiveFeasibilityCheckResponse, new_M_y) -> dict:
        """Parse response from physical check
        """
        infeasible_M_y = dict()
        name_2_tuple = {v: k for k, v in new_M_y.items()}
        feasibilities_checked = response.feasibilities_checked.feasibilities
        for feasibility in feasibilities_checked:
            if not feasibility.feasibility:
                infeasible_M_y[name_2_tuple[feasibility.name]] = True
        return infeasible_M_y

    def _check_M_y_soundness(self, compiler: Compiler, M_y: dict) -> None:
        """Check if M_y is sound
        Inputs:
            compiler: Compiler
            M_y: dict
        """
        # make sure all skills in the compiler are in M_y
        for skill in compiler.get_skills():
            if skill not in M_y.values():
                raise ValueError(f"Skill {skill} is not in M_y")

    def rename_skills(self, compiler: Compiler, skills: dict, new_M_y: dict) -> None:
        """Rename skills wrt the skills in compiler
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
            while name in compiler.get_skills():
                cnt += 1
                name = f"skill_{cnt}"
            assert name not in compiler.get_skills(), f"Name {name} already exists in the compiler: {compiler.get_skills()}"
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
            skill_name = find_next_skill_name(skill_name, new_M_y)
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
                skill_name = find_next_skill_name(skill_name, new_M_y)
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
    
    def minimize_and_rename_new_skills(self, compiler: Compiler, new_skills: dict, new_M_y: dict) -> tuple:
        # if DEBUG:
        if len(new_skills) <= 1:
            if True:
                print(" only one new skill: ", new_skills)
                print("==== compiler current skills ====")
                for skill in compiler.get_skills():
                    print(skill)
            pass
        else:
            # breakpoint()
            self.rename_skills(compiler, new_skills, new_M_y)
            for skill_name in list(new_skills.keys()):
                compiler_copy: Compiler = copy.deepcopy(compiler)
                removed_skill = new_skills.pop(skill_name)
                new_skills_modulo_terrains = self._make_new_skills_modulo_terrains_copy(new_skills)
                compiler_copy.add_skills_no_intermediate_states_modulo_spec(new_skills_modulo_terrains)
                if True:
                    print("==== skills to check ====")
                    print(compiler_copy.get_skills())
                    print("====")
                is_realizable = compiler_copy.check_realizability()
                if not is_realizable:
                    new_skills[skill_name] = removed_skill
                else:
                    # Remove the skill from new_M_y
                    key_to_remove = list(filter(lambda key: new_M_y[key] == skill_name, new_M_y))[0]
                    new_M_y.pop(key_to_remove)
                if len(new_skills) <= 1:
                    break
        self.rename_skills(compiler, new_skills, new_M_y)
        return new_skills, new_M_y
        
    def _make_new_skills_modulo_terrains_copy(self, new_skills: dict) -> dict:
        """Make a copy of new_skills modulo terrains"""
        new_skills_modulo_terrains = dict()
        for name, skill in new_skills.items():
            new_skills_modulo_terrains[name] = self._make_skill_modulo_terrains(skill)
        return new_skills_modulo_terrains
    
    def _make_skill_modulo_terrains(self, skill: Skill) -> Skill:
        """Make a copy of skill modulo terrains"""
        new_skill_dict = dict()
        new_skill_dict["name"] = skill.name
        new_skill_dict["initial_preconditions"] = []
        new_skill_dict["final_postconditions"] = []
        new_skill_dict["intermediate_states"] = []
        for pre_dict, post_dict_list in skill.intermediate_states:
            new_pre_dict = self._make_state_modulo_terrains(pre_dict)
            new_post_dict_list = [self._make_state_modulo_terrains(post_dict) for post_dict in post_dict_list]
            new_skill_dict["initial_preconditions"].append(new_pre_dict)
            new_skill_dict["intermediate_states"].append([new_pre_dict, new_post_dict_list])
            new_skill_dict["final_postconditions"].append(new_post_dict_list[-1])
        return Skill(info=new_skill_dict)
    
    def _make_state_modulo_terrains(self, state_dict: dict) -> dict:
        """Make a copy of state_dict modulo terrains"""
        new_state_dict = dict()
        for key, value in state_dict.items():
            if "terrain" in key:
                continue
            new_state_dict[key] = value
        return new_state_dict

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
        m_y maps a terrain state to a set of relevant skills
        M_I maps a terrain state to a set of physically infeasible transitions
        M_y maps a tuple (dir, curr_terrain, next_terrain) to a str of skill name
            where dir \in (0,1), (0,-1), (-1,0), (1,0)
        M_o maps a tuple (terrain_input_int, terrain_type) to a list of ASTs formula about the obstacle constraints
        """
        # M_y: (dir, curr_terrain, next_terrain) -> skill_name
        self.M_y = dict()
        self.all_new_M_y = dict()

        # M_i: (dir, curr_terrain, next_terrain) -> Boolean
        self.M_i = dict()

        # M_o: terrain = (terrain_input_int: str, terrain_type: int) -> a list of AST formulas
        self.M_o = dict()

        self.create_M_y_mapping()
        self.create_M_o_mapping()
        if True:
            print("==== M_y ====")
            print(self.M_y)
            print("==== M_o ====")
            print(self.M_o)
            # print("==== exit due to debug ====")
            # sys.exit(0)

    def load_mappings(self) -> None:
        """Load mappings from a file"""
        file_json: dict = json_load_wrapper(self.mapping_file)
        self.M_y = dict_key_str2tuple(file_json["M_y"])
        self.M_i = dict_key_str2tuple(file_json["M_i"])
        self.M_o = dict_key_str2tuple(file_json["M_o"])
        self.all_new_M_y = dict()
        return None
    
    def store_mappings(self, M_y: dict, M_i: dict, M_o: dict) -> None:
        """Store mappings to a file"""
        file_json: dict = dict()
        M_y = dict_key_tuple2str(M_y)
        M_i = dict_key_tuple2str(M_i)
        M_o = dict_key_tuple2str(M_o)
        file_json["M_y"] = M_y
        file_json["M_i"] = M_i
        file_json["M_o"] = M_o
        dump_json(self.mapping_file, file_json)
        return None
    
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
    
    def add_self_loop_skill_to_skills2transitions(self, skills2transitions: dict, x, y) -> None:
        """Add self loop skill to skills2transitions"""
        skills2transitions['skill_selfloop'] = [(x, y, x, y)]
        return None

    def m_o(self, terrain_state: dict) -> list:
        """Given a terrain state, return a list of AST formulas representing invalid locations due to obstacle
        Inputs:
            terrain_state: dict
        Outputs:
            a list of AST formulas, each representing an invalid location due to obstacle in the terrain state
        """
        # breakpoint()
        invalid_locations_ast_formulas: list = []
        for terrain_input_int, terrain_input_type in terrain_state.items():
            if "terrain" not in terrain_input_int: continue
            if (terrain_input_int, terrain_input_type) not in self.M_o:
                continue
            invalid_locations_ast_formulas.append(self.M_o[(terrain_input_int, terrain_input_type)])
        if DEBUG:
            print("==== invalid locations due to obstacle ====")
            print(invalid_locations_ast_formulas)
            print("==== Exit due to debugging ====")
            sys.exit(0)
        return invalid_locations_ast_formulas
    
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
    
    def create_M_o_mapping(self) -> None:
        """Create mapping M_o
        M_o maps a terrain input (terrain_input_int: str, terrain_type: int) to a list of AST formulas representing obstacle locations
        """
        for sys_trans_hard_formula in self.compiler.get_sys_trans_hard_asts():
            if not self.compiler.contains_keyword(sys_trans_hard_formula, "terrain"):
                break
            implication = sys_trans_hard_formula[1]
            left, right = implication[1], implication[2]
            terrain_input_int_and_type: tuple = self.get_terrain_input_and_type_from_ast(left)
            constraints: list = self.get_ast_constraints_from_ast(right)
            if DEBUG:
                print("==== terrain_input_int_and_type ==== ")
                print(terrain_input_int_and_type)
                print("==== constraints ====")
                print(constraints)
                print("==== exit due to debugging ====")
                sys.exit(0)
            assert terrain_input_int_and_type not in self.M_o, f"terrain_input {terrain_input_int_and_type} is already in M_o"
            self.M_o[terrain_input_int_and_type] = constraints
            if DEBUG:
                print("==== ast for obstacle constraints ==== ")
                print(self.compiler.get_sys_trans_hard_asts()[0])
                print("is terrain input in the ast?: ", self.compiler.contains_keyword(self.compiler.get_sys_trans_hard_asts()[0], "terrain"))
                print("====")
                print("implication: ", implication)
                print("====")
                print("left: ", left)
                print("====")
                print("right: ", right)
                print("==== exit due to debugging ====")
                sys.exit(0)

    def get_terrain_input_and_type_from_ast(self, ast: list) -> tuple:
        """Extract terrain input and type from ast
        Inputs:
            ast: list: a CalculationSubformula
        Outputs:
            (terrain_input, terrain_type) where
            terrain_input: str: the terrain integer input variable
            terrain_type: int: the terrain type
        """
        terrain_input_numid: list = ast[1]
        terrain_input: str = terrain_input_numid[1]
        terrain_type_numeral: list = ast[3]
        terrain_type: int = int(terrain_type_numeral[1])
        return (terrain_input, terrain_type)
    
    def get_ast_constraints_from_ast(self, ast: list) -> list:
        """Extract obstacle constraint formula from conjunction ast"""
        return self.compiler.add_formula_wrapper(ast)
            
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
        


    def generate_bool_spec(self, compiler: Compiler) -> None:
        compiler.transform_asts_int2bool()
        compiler.generate_structuredslugsplus(self.spec_bool)

# def setup_for_ros(filename_json: str) -> None:
#     file_dict: dict = json_load_wrapper(filename_json)
#     opts: dict = json_load_wrapper(file_dict["opts"])
#     if opts["symbolic_repair_only"]:
#         return None
#     else:
#         rospy.init_node("mocomp")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="High-level manager for quadruped locomotion")
    parser.add_argument("-f", "--filename_json", type=str, help="json file with input file names")
    parser.add_argument("-o", "--offline", action="store_true", help="offline repair")
    parser.add_argument("-r", "--runtime", action="store_true", help="runtime repair")
    args = parser.parse_args()
    manager = Manager(args.filename_json)
    # setup_for_ros(args.filename_json)
    if args.offline:
        manager.offline_repair()
    elif args.runtime:
        manager.runtime_repair()
    else:
        raise ValueError("Please specify either offline or runtime repair")
