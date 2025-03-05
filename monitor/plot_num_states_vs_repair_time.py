#! /usr/bin/env python3
import matplotlib.pyplot as plt
import json

# Load the two json data files
data_3x3_unstructured_4 = json.load(open("../data/perceptive_locomotion/3x3_go2_unstructured_4_terrain/log_3x3_go2_unstructured_4_terrain--5.json"))
data_3x3_unstructured_8 = json.load(open("../data/perceptive_locomotion/3x3_go2_unstructured_8_terrain/log_3x3_go2_unstructured_8_terrain--3.json"))
data_3x3_rebar_7 = json.load(open("../data/perceptive_locomotion/3x3_chotu_rebar_7_terrain/log_3x3_chotu_rebar_7_terrain--3.json"))
data_3x3_rebar_14 = json.load(open("../data/perceptive_locomotion/3x3_chotu_rebar_14_terrain/log_3x3_chotu_rebar_14_terrain--2.json"))

data_5x5_unstructured_4 = json.load(open("../data/perceptive_locomotion/5x5_go2_unstructured_4_terrain/log_5x5_go2_unstructured_4_terrain--2.json"))
data_5x5_unstructured_8 = json.load(open("../data/perceptive_locomotion/5x5_go2_unstructured_8_terrain/log_5x5_go2_unstructured_8_terrain--1.json"))
data_5x5_rebar_7 = json.load(open("../data/perceptive_locomotion/5x5_chotu_rebar_7_terrain/log_5x5_chotu_rebar_7_terrain--3.json"))
data_5x5_rebar_14 = json.load(open("../data/perceptive_locomotion/5x5_chotu_rebar_14_terrain/log_5x5_chotu_rebar_14_terrain--1.json"))

datasets = [data_3x3_unstructured_4, data_3x3_unstructured_8, data_3x3_rebar_7, data_3x3_rebar_14, data_5x5_unstructured_4, data_5x5_unstructured_8, data_5x5_rebar_7, data_5x5_rebar_14]
datasets_legend = ["3x3_unstructured_4", "3x3_unstructured_8", "3x3_rebar_7", "3x3_rebar_14", "5x5_unstructured_4", "5x5_unstructured_8", "5x5_rebar_7", "5x5_rebar_14"]
accumulative_repair_time_for_datasets = []

# Extract the symbolic repair time for each (terrain state, request state) pair, and accumulate them
for dataset in datasets:
    accumulative_repair_time = []
    accumulator = 0
    for key, val in dataset.items():
        if "_symbolic_repair_time" in key and "total" not in key:
            accumulator += val
            accumulative_repair_time.append(accumulator)
    accumulative_repair_time_for_datasets.append(accumulative_repair_time)

# Use a nicer style
# plt.style.use('seaborn-whitegrid')
plt.figure(figsize=(10, 7))

for i in range(len(accumulative_repair_time_for_datasets)):
    plt.plot(accumulative_repair_time_for_datasets[i], label=datasets_legend[i], linewidth=3) #, marker='o', markersize=6)
# plt.plot(accumulative_repair_time_for_datasets[1], label="5x5", linewidth=3)
# plt.plot(accumulative_repair_time_for_datasets[0], label="3x3", linewidth=3)

plt.xlabel("Number of (terrain state, request state) pairs", fontsize=25)
plt.ylabel("Symbolic repair time (s)", fontsize=25)

plt.xticks(fontsize=22)
plt.yticks(fontsize=22)

plt.legend(fontsize=18)

plt.grid(True)

# Ensure axes start at zero
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.tight_layout()


plt.show()

