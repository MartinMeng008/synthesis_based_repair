import argparse
from tools import (
    json_load_wrapper
)

def data_analysis_new_skills(data_file):
    data = json_load_wrapper(data_file)
    print(data)
    new_skills = set()
    for key, val in data.items():
        # if key ends with "_new_skills"
        if key.endswith("_new_skills"):
            # if val is a dict
            if isinstance(val, dict):
                # add the values to new_skills
                new_skills.update(val.keys())
    print("==== Analysis ====")
    print("New skills: ", new_skills)
    print("Num of new skills: ", len(new_skills))
    return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Data analysis")
    parser.add_argument("-d", "--data_file", help="Data file")
    args = parser.parse_args()
    data_analysis_new_skills(args.data_file)
