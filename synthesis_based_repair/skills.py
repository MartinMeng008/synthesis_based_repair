#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from synthesis_based_repair.tools import dict_to_formula, pre_posts_to_env_formula, sym_list_to_dict
import os
from synthesis_based_repair.symbols import in_symbols, find_symbols_true_and_false, load_symbols, plot_symbolic_state
import json
import symbolic_repair_msgs.msg
# from symbolic_repair_msgs.srv import FeasibilityCheck, FeasibilityCheckResponse

def dict_bool2list(dic: dict) -> list:
    ls = []
    for name, val in dic.items():
        if val:
            ls.append(name)
    return ls

class Skill:

    def __init__(self, info, suggestion=False, stretch_eff=0.1, terrain_inputs: list = [], location_inputs: list = []):
        self.info = info
        self.location_inputs = location_inputs
        self.terrain_inputs = terrain_inputs
        if False:
            print("location inputs order: ", self.location_inputs)
            print("terrain inputs order: ", self.terrain_inputs)
        self.name = info['name']
        self.suggestion = suggestion
        self.stretch_eff = stretch_eff
        if suggestion:
            self.original_skill = info['original_skill']
            self.new_skill = info['new_skill']
        self.intermediate_states = info['intermediate_states']
        if "initial_preconditions" not in info.keys():
            info['initial_preconditions'] = []
        self.init_pres = info['initial_preconditions']
        if "final_postconditions" not in info.keys():
            info['final_postconditions'] = []
        self.final_posts = info['final_postconditions']
        # self.unique = info['unique_states']
        if 'folder_train' in info.keys():
            self.folder_train = info['folder_train']
        if 'folder_val' in info.keys():
            self.folder_val = info['folder_val']

    def get_info(self):
        return self.info

    def get_name(self):
        return self.name
    
    def get_skill_msg(self) -> symbolic_repair_msgs.msg.Skill:
        """Output a dictionary representation of skill
        Output:
            skill_out = symbolic_repair_msgs.msg.Skill 
            skill_out.initial_precondition: str[str[]]: [["x0", "0"], ["y0", "0"]] 
            skill_out.final_postcondition: str[str[]]: [["x0", "0"], ["y0", "1"]] 
            skill_out.name: str 
        """
        skill_msg = symbolic_repair_msgs.msg.Skill()
        skill_msg.name = self.name
        skill_msg.initial_preconditions = []
        skill_msg.final_postconditions = []
        assert len(self.init_pres) == 1, f"too many preconditions: {len(self.init_pres)}"
        assert len(self.final_posts) == 1, f"too many postconditions: {len(self.final_posts)}"
        for input in self.location_inputs + self.terrain_inputs:
            key_val_pair = symbolic_repair_msgs.msg.AtomicProposition()
            key_val_pair.atomic_proposition = [input, "1" if self.init_pres[0][input] else "0"]
            skill_msg.initial_preconditions.append(key_val_pair)
        for input in self.location_inputs:
            key_val_pair = symbolic_repair_msgs.msg.AtomicProposition()
            key_val_pair.atomic_proposition = [input, "1" if self.final_posts[0][input] else "0"]
            skill_msg.final_postconditions.append(key_val_pair)
        return skill_msg




    def print_dict(self) -> None:
        print("========")
        print(self.name)
        print("initial preconditions: ")
        for init_dict in self.init_pres:
            print(dict_bool2list(init_dict))
        print("====")
        print("intermediate states: ")
        for pre_dict, post_dict_list in self.intermediate_states:
            print("pre: ", dict_bool2list(pre_dict))
            for post_dict in post_dict_list:
                print("post: ", dict_bool2list(post_dict))
        print("====")
        print("final postconditions: ")
        for post_dict in self.final_posts:
            print(dict_bool2list(post_dict))
        try:
            print("original skill: ", self.original_skill)
            print("new skill: ", self.new_skill)
        except AttributeError:
            pass
        print("")
        # print("info", self.info)
        print("========")
        print("")

    def write_to_file(self, file_name: str) -> None:
        fid = open(file_name, "w")
        self.write_to_fid(fid)
        fid.close()

    def write_to_fid(self, fid) -> None:
        fid.write("========\n")
        fid.write(self.name + "\n")
        fid.write("initial preconditions: \n")
        for init_dict in self.init_pres:
            fid.write(str(dict_bool2list(init_dict)) + "\n")
        fid.write("====\n")
        fid.write("intermediate states: \n")
        for pre_dict, post_dict_list in self.intermediate_states:
            fid.write("pre: " + str(dict_bool2list(pre_dict)) + "\n")
            for post_dict in post_dict_list:
                fid.write("post: " + str(dict_bool2list(post_dict)) + "\n")
        fid.write("====\n")
        fid.write("final postconditions: \n")
        for post_dict in self.final_posts:
            fid.write(str(dict_bool2list(post_dict)) + "\n")
        try:
            fid.write("original skill: " + str(self.original_skill) + "\n")
            fid.write("new skill: " + str(self.new_skill) + "\n")
        except AttributeError:
            pass
        fid.write("========\n")


    def get_skill_str(self, include_false=True):
        out = "Skill name: " + self.name + "\n"
        if self.suggestion:
            out += "Original name: " + dict_to_formula(self.original_skill, include_false=False) + "\n"
        out += "Initial preconditions: \n"
        for init_pre in self.init_pres:
            out += "{}\n".format(dict_to_formula(init_pre, prime=False, include_false=include_false))
        out += "Intermediate states: \n"
        for int_state in self.intermediate_states:
            out += "{}\n".format(
                pre_posts_to_env_formula(self.name, int_state[0], int_state[1]))
        out += "Final Postconditions:\n"
        for post in self.final_posts:
            out += "{}\n".format(dict_to_formula(post))

        return out

    def write_skill_str(self, file_name, include_false=True):
        fid = open(file_name, "a")
        fid.write(self.get_skill_str(include_false=include_false))
        fid.write('**********************\n')
        fid.close()

    def stretch_joints_to_cartesian(self, data):
        """ Convert the stretch joint angles to cartesian end effector positions"""
        # x, y, theta, arm length, z, wrist theta
        x = data[:, 0]
        y = data[:, 1]
        theta = data[:, 2]
        arm = data[:, 3]
        z = data[:, 4]
        theta_wrist = data[:, 5]

        x_ee = self.stretch_eff * np.cos(theta + theta_wrist) + arm * np.sin(theta) + x
        y_ee = self.stretch_eff * np.sin(theta + theta_wrist) - arm * np.cos(theta) + y
        z_ee = z

        out = np.hstack((x_ee[:, np.newaxis], y_ee[:, np.newaxis], z_ee[:, np.newaxis]))
        return out

    def plot(self, ax, data, **kwargs):
        for d in data:
            if d.shape[1] == 2:
                ax.plot(d[:, 0], d[:, 1], **kwargs)
            elif d.shape[1] == 3:
                ax.plot(d[:, 0], d[:, 1], d[:, 2], **kwargs)
            elif d.shape[1] == 6:
                ee = self.stretch_joints_to_cartesian(d)
                ax.plot(ee[:, 0], ee[:, 1], ee[:, 2], **kwargs)
                ax.plot(d[:, 0], d[:, 1], np.zeros(ee[:, 1].shape), **kwargs)

    def plot_original(self, ax, train=True, **kwargs):
        if train:
            folder_trajectories = self.folder_train
        else:
            folder_trajectories = self.folder_val
        files_folder = [f for f in os.listdir(folder_trajectories) if
                        os.path.isfile(os.path.join(folder_trajectories, f))]
        files_traj = [f for f in files_folder if 'rollout' in f]
        data = np.stack([np.loadtxt(os.path.join(folder_trajectories, rp), ndmin=2) for rp in files_traj])

        self.plot(ax, data, **kwargs)

    def plot_nice(self, ax, plot_limits, symbols, train=True, idx=0, **kwargs):
        if train:
            folder_trajectories = self.folder_train
        else:
            folder_trajectories = self.folder_val
        files_folder = [f for f in os.listdir(folder_trajectories) if
                        os.path.isfile(os.path.join(folder_trajectories, f))]
        files_traj = [f for f in files_folder if 'rollout' in f]
        data = np.stack([np.loadtxt(os.path.join(folder_trajectories, rp), ndmin=2) for rp in files_traj])
        d = data[idx, :, :]
        ee = self.stretch_joints_to_cartesian(d)

        ax.set_xlim(plot_limits[0])
        ax.set_ylim(plot_limits[1])
        ax.set_zlim(plot_limits[2])
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

        for sym_name, sym in symbols.items():
            sym.plot(ax, dim=3, fill=True, lw=1, alpha=0.1)

        ax.plot(ee[:, 0], ee[:, 1], ee[:, 2], color='g')
        ax.plot(ee[:, 0], ee[:, 1], np.zeros(ee[:, 1].shape), ':', color='g')
        ax.plot(d[:, 0], d[:, 1], np.zeros(ee[:, 1].shape), color='r')
        for ii in range(0, np.shape(d)[0], 20):
            ax.plot(np.array([d[ii, 0], d[ii, 0]]), np.array([d[ii, 1], d[ii, 1]]), np.array([0, 0.4]), color='k')
            ax.plot(np.array([d[ii, 0], ee[ii, 0]]), np.array([d[ii, 1], ee[ii, 1]]), np.array([d[ii, 4], d[ii, 4]]), color='k')

    def sample_start_end_states(self, limits, sym_defs, n_points=32, n_rows=2, dim=2):
        out_points = np.zeros([n_rows, dim, n_points])

        # Preconditions
        for ii in range(n_points):
            point = limits[:, 0] + np.random.random(dim) * (limits[:, 1] - limits[:, 0])
            pre_dict = self.init_pres[np.random.randint(len(self.init_pres))]
            while not in_symbols(point, pre_dict, sym_defs):
                point = limits[:, 0] + np.random.random(dim) * (limits[:, 1] - limits[:, 0])
            out_points[0, :, ii] = point

        # Postconditions
        for ii in range(n_points):
            point = limits[:, 0] + np.random.random(dim) * (limits[:, 1] - limits[:, 0])
            post_dict = self.init_pres[np.random.randint(len(self.final_posts))]
            while not in_symbols(point, post_dict, sym_defs):
                point = limits[:, 0] + np.random.random(dim) * (limits[:, 1] - limits[:, 0])
            out_points[1, :, ii] = point

        return out_points

    def plot_dmp(self, ax, start_end_states):
        pass

    def get_skill_dict(self):
        pass

    def to_json(self):
        outdict = dict()
        outdict['name'] = self.name
        if self.suggestion:
            outdict['original_skill'] = self.original_skill
            outdict['new_skill'] = self.new_skill
        outdict['intermediate_states'] = self.intermediate_states
        outdict['initial_preconditions'] = self.init_pres
        outdict['final_postconditions'] = self.final_posts
        outdict['folder_train'] = self.folder_train
        outdict['folder_val'] = self.folder_val
        return outdict

    def get_intermediate_states(self):
        return self.intermediate_states

    def get_final_posts(self):
        return self.final_posts

    def get_initial_pres(self):
        return self.init_pres

    def is_suggestion(self):
        return self.suggestion

    def plot_symbolic_skill(self, symbols, xlims, ylims, **kwargs):
        ncols = 2
        for _, posts in self.intermediate_states:
            ncols = max([ncols, len(posts) + 1])
        nrows = len(self.intermediate_states)

        fig, ax = plt.subplots(nrows=nrows, ncols=ncols)
        ax[0, 0].set_title("Precondition")
        for ii in range(1, ncols):
            ax[ii, 0].set_title("Postcondition")
        for ii, (pre, posts) in enumerate(self.intermediate_states):
            plot_symbolic_state(pre, symbols, ax[nrows-(ii+1), 0], xlims, ylims, **kwargs)
            ax[ii, 0].set_xticks([])
            ax[ii, 0].set_yticks([])
            for jj, post in enumerate(posts):
                plot_symbolic_state(post, symbols, ax[nrows-(ii+1), jj+1], xlims, ylims, **kwargs)
                ax[ii, jj+1].set_xticks([])
                ax[ii, jj+1].set_yticks([])
        return fig, ax


def write_skills_str(skills, file_str, only_suggestions=False, include_false=False):
    for _, skill in skills.items():
        if only_suggestions and not skill.is_suggestion():
            continue
        skill.write_skill_str(file_str, include_false=include_false)

    fid = open(file_str, 'a')
    fid.write("================================\n")
    fid.close()

def write_skills_json(skills, file_json):
    outdict = dict()
    for skill_name, skill in skills.items():
        outdict[skill_name] = skill.to_json()
    with open(file_json, "w") as outfile:
        json.dump(outdict, outfile, indent=1)


def find_one_skill_intermediate_states(arg_folder_traj, arg_symbols):
    files_folder = [f for f in os.listdir(arg_folder_traj) if os.path.isfile(os.path.join(arg_folder_traj, f))]
    files_traj = [f for f in files_folder if 'rollout' in f]

    intermediate_states = []
    poss_changes = []

    for f in files_traj:
        data = np.loadtxt(arg_folder_traj + "/" + f, delimiter=" ", dtype=float)
        traj_syms = find_traj_in_syms(data, arg_symbols)

        for ii in range(0, len(traj_syms) - 1):
            if traj_syms[ii] != traj_syms[ii + 1]:
                poss_changes.append([traj_syms[ii], [traj_syms[ii + 1]]])

    for poss_change in poss_changes:
        pre_already_entered = False
        for ii, (pre, posts) in enumerate(intermediate_states):
            if pre == poss_change[0]:
                pre_already_entered = True
                post_entered = False
                for post in posts:
                    if post == poss_change[1][0]:
                        post_entered = True
                if not post_entered:
                    intermediate_states[ii][1].append(poss_change[1][0])
                break
        if not pre_already_entered:
            intermediate_states.append(poss_change)

    return intermediate_states


def find_traj_in_syms(arg_data, arg_symbols):
    syms_out = []
    for ii, d in enumerate(arg_data):
        syms_out.append(find_symbols_true_and_false(d, arg_symbols))

    return syms_out


def find_one_skill_pre_or_post(arg_folder_traj, arg_symbols, find_pre):

    files_folder = [f for f in os.listdir(arg_folder_traj) if os.path.isfile(os.path.join(arg_folder_traj, f))]
    files_traj = [f for f in files_folder if 'rollout' in f]

    syms_lists = []

    if find_pre:
        idx = 0
    else:
        idx = -1

    for f in files_traj:
        data = np.loadtxt(arg_folder_traj + "/" + f, delimiter=" ", dtype=float)
        traj_syms = find_traj_in_syms(data, arg_symbols)
        if traj_syms[idx] not in syms_lists:
            syms_lists.append(traj_syms[idx])

    return syms_lists


def load_skills_from_trajectories(folder_trajectory_base, skill_names, sym_defs):
    skills = dict()
    for skill_name in skill_names:
        skill = dict()
        skill['folder_train'] = folder_trajectory_base + '/' + skill_name + "/train/"
        skill['folder_val'] = folder_trajectory_base + '/' + skill_name + "/train/"
        folder_trajectories = skill['folder_train']
        skill['name'] = skill_name
        skill['intermediate_states'] = find_one_skill_intermediate_states(folder_trajectories, sym_defs)
        skill['initial_preconditions'] = find_one_skill_pre_or_post(folder_trajectories, sym_defs, True)
        skill['final_postconditions'] = find_one_skill_pre_or_post(folder_trajectories, sym_defs, False)
        for final_post in skill['final_postconditions']:
            for ii, (pre, posts) in enumerate(skill['intermediate_states']):
                if final_post == pre:
                    skill['intermediate_states'][ii][1].append(final_post)

        skills[skill_name] = Skill(skill)

    return skills


def load_skills_from_json(file_json):
    fid = open(file_json, 'r')
    data = json.load(fid)
    fid.close()

    skills =  dict()
    for skill_name, skill in data.items():
        skills[skill_name] = Skill(skill)

    return skills


if __name__ == "__main__":
    folder_trajectories = '../data/nine_squares/trajectories/'
    skill_names = ["skill0", 'skill1', 'skill2', 'skill3', 'skill4', 'skill5', 'skill6', 'skill7', 'skill8', 'skill9', 'skill10']
    f_symbols = "../data/nine_squares/nine_squares_symbols.json"
    f_plot = "../data/nine_squares/plots/"
    f_skills = "../data/nine_squares/nine_squares_skills.json"

    # folder_trajectories = '../data/stretch/trajectories/'
    # skill_names = ["skillStretch0", 'skillStretch1']
    # f_symbols = "../data/stretch/stretch_symbols.json"
    # f_plot = "../data/stretch/plots/"
    # f_skills = "../data/stretch/stretch_skills.json"

    symbols = load_symbols(f_symbols)
    skills = load_skills_from_trajectories(folder_trajectories, skill_names, symbols)
    for skill_name, skill in skills.items():
        fig, ax = plt.subplots()
        skill.plot_original(ax)
        ax.set_xlim([-0.5, 3.5])
        ax.set_ylim([-0.5, 3.5])
        ax.set_xticks([0, 1, 2, 3])
        ax.set_yticks([0, 1, 2, 3])
        plt.savefig(f_plot + skill_name + ".png")
        plt.close()
    write_skills_json(skills, f_skills)
