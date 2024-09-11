#! /usr/bin/env python
import sys
import argparse
import copy
from mocomp import Compiler
from repair import Repair
from importlib import reload
# sys.path.insert(0, '../../')
from tools import (
    json_load_wrapper,
    create_build_monitor,
    )
import rospy

create_build_monitor()
sys.path.insert(0, 'build')
import generated_monitor
from generated_monitor import Monitor

from symbolic_repair_msgs.msg import AtomicProposition, TerrainState, OnlineRepairResult
from symbolic_repair_msgs.srv import OnlineRepairWithNewTerrain, OnlineRepairWithNewTerrainResponse

class OnlineRepair:
    def __init__(self, filename_json: str) -> None:
        """Execute online repair for a given terrain state
        """
        files_json = json_load_wrapper(filename_json)
        # print(files_json)
        skills_data = dict()
        symbols_data = dict()
        objects_data = dict()
        controllabe_variables = []
        uncontrollable_variables = []

        self.input_filename = files_json["output_file_structuredslugsplus"]
        self.output_filename_structuredslugsplus = files_json["online_output_file_structuredslugsplus"]
        self.output_filename_slugsin = files_json["online_output_file_slugsin"]
        self.output_filename_structuredslugs = files_json["online_output_file_structuredslugs"]
        # self.backup_skill_added_filename = files_json["online_backup_added_output_file_structuredslugsplus"]
        self.opts_filename = files_json["opts"]

        self.compiler = Compiler(input_file=self.input_filename, 
                            skills_data=skills_data, 
                            symbols_data=symbols_data, 
                            objects_data=objects_data, 
                            controllabe_variables=controllabe_variables,
                            uncontrollable_variables=uncontrollable_variables,
                            opts=self.opts_filename)
        
        self.terrain_states_formula = copy.deepcopy(self.compiler.refer_to_terrain_assumptions())
        self.reload_monitor()
        
        self.repair_service = rospy.Service('/symbolic_repair/online_repair', OnlineRepairWithNewTerrain, self.online_repair_callback)
        
        # new_terrain_state_node = "to_do"
        # new_terrain_state_node_type = "to_do"
        # self.sub = rospy.Subscriber(new_terrain_state_node, new_terrain_state_node_type, self.online_repair_callback, queue_size=1)

    def reload_monitor(self):
        """Reload monitor
        """
        self.compiler.generate_z3_monitor("build/generated_monitor.py")
        reload(generated_monitor)
        from generated_monitor import Monitor
        self.monitor = Monitor()

    def online_repair_callback(self, req):
        """Perform online repair for a given terrain state specified in msg
        """
        print("Received request")
        print(req.terrain_state_requested)
        terrain_state: dict = self._get_terrain_state_from_req(req)
        results = self.monitor.monitor_curr_input_state(terrain_state)
        res = OnlineRepairWithNewTerrainResponse()
        res.repair_result = OnlineRepairResult()
        res.repair_result.repair_needed = None
        res.repair_result.slugsin_location = self.compiler.opts["online_output_file_slugsin"]

        if self.monitor.no_violation(results):
            print("No violation")
            res.repair_result.repair_needed = False
        else:
            print("Violation detected")
            res.repair_result.repair_needed = True
            violated_indices = self.monitor.filter_violation(results)
            self.compiler.relax_hard_assumption_X_only(terrain_state=terrain_state, indices=violated_indices)
            self.compiler.remove_init_terrain_state_formula()
            self.compiler.add_init_terrain_state(self, terrain_state)
            # self.compiler.generate_structuredslugsplus(self.output_filename_structuredslugsplus)
            self.compiler.add_backup_skills()
            self.compiler.generate_structuredslugsplus(self.output_filename_structuredslugsplus)
            repair = Repair(compiler=self.compiler, filename=self.output_filename_structuredslugsplus, opts=self.compiler.opts, symbolic_repair_only=self.compiler.opts["symbolic_repair_only"])
            print("Repairing for terrain state: ", terrain_state)
            new_skills = repair.run_symbolic_repair()
            self.compiler.reset_after_successful_repair()
            self.compiler.remove_init_terrain_state(terrain_state)
            self.compiler.add_init_terrain_states_from_env_trans_hard()
            self.compiler.generate_structuredslugsplus(self.output_filename_structuredslugsplus)
            self.compiler.generate_slugsin(self.output_filename_slugsin)
            self.compiler.generate_structuredslugs(self.output_filename_structuredslugs)
            self.reload_monitor()
        return res
    
    def _get_terrain_state_from_req(self, req) -> dict:
        """Get terrain state from request
        """
        terrain_state_msg = req.terrain_state_requested.terrain_state
        terrain_state: dict = {}
        for atomic_prop in terrain_state_msg:
            atomic_prop = atomic_prop.atomic_proposition
            terrain_state[atomic_prop[0]] = atomic_prop[0] == '1'
        return terrain_state
    

if __name__ == '__main__':
    rospy.init_node('online_repair_node')
    argparser = argparse.ArgumentParser()
    argparser.add_argument('-f', '--file', action='store', dest='file_json', required=False, default=None)
    args = argparser.parse_args()
    online_repair = OnlineRepair(args.file_json)
    print("Ready to take new terrain states for online repair")
    rospy.spin()
    # online_repair(args.file_json)