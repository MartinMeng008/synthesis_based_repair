#! /usr/bin/env python
import sys
import argparse
import copy
import rospy
from mocomp import Compiler
from repair import Repair
# sys.path.insert(0, '../../')
from tools import (
    json_load_wrapper,
    )
    
def main(filename_json: str) -> None:
    files_json = json_load_wrapper(filename_json)
    # print(files_json)
    skills_data = dict()
    symbols_data = dict()
    objects_data = dict()
    controllabe_variables = []
    uncontrollable_variables = []

    # Transform integer variables to Boolean
    input_file_int_filename = files_json["input_file_int"]
    input_file_bool_filename = files_json["input_file_bool"]
    output_filename_structuredslugsplus = files_json["output_file_structuredslugsplus"]
    output_filename_slugsin = files_json["output_file_slugsin"]
    output_filename_structuredslugs = files_json["output_file_structuredslugs"]
    opts_filename = files_json["opts"]
    backup_skill_added_filename = files_json["input_file_backup_skill_added"]
    terrain_states: list = json_load_wrapper(files_json['terrain_states'])["terrain_states_list"]
    compiler = Compiler(input_file=input_file_int_filename, 
                        skills_data=skills_data, 
                        symbols_data=symbols_data, 
                        objects_data=objects_data, 
                        controllabe_variables=controllabe_variables,
                        uncontrollable_variables=uncontrollable_variables,
                        opts=opts_filename)
    compiler.transform_asts_int2bool()
    compiler.add_change_constraints()
    compiler.add_not_allowed_repair()
    compiler.generate_structuredslugsplus(input_file_bool_filename)

    if not compiler.opts["symbolic_repair_only"]:
        rospy.init_node('repair_node')
        rospy.wait_for_service('/feasibility_check')
    
    # return None
    # Go through each terrain state, add it to env_init, and repair
    for terrain_state in terrain_states:
        compiler.add_init_terrain_state(terrain_state)
        compiler.add_backup_skills()
        compiler.generate_structuredslugsplus(backup_skill_added_filename)
        repair = Repair(compiler=compiler, filename=backup_skill_added_filename, opts=compiler.opts, symbolic_repair_only=compiler.opts["symbolic_repair_only"])
        print("Repairing for terrain state: ", terrain_state)
        new_skills = repair.run_symbolic_repair()
        compiler.reset_after_successful_repair()
        compiler.remove_init_terrain_state(terrain_state)
    
    compiler.add_init_terrain_states(terrain_states)
    compiler.add_terrain_states_as_env_trans_hard(terrain_states)
    compiler.generate_structuredslugsplus(output_filename_structuredslugsplus)
    compiler.generate_slugsin(output_filename_slugsin)
    compiler.generate_structuredslugs(output_filename_structuredslugs)
    return None

if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('-f', '--file', action='store', dest='file_json', required=False, default=None)
    args = argparser.parse_args()
    main(args.file_json)