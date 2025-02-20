#!/usr/bin/env python3
"""Given a structuredslugsplus file name with integer inputs, generate a files.json"""
import json
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a files.json file for structuredslugsplus")
    parser.add_argument("-f", "--filename", type=str, help="The name of the structuredslugsplus file")
    args = parser.parse_args()
    breakpoint()
    filename = args.filename
    filename_prefix = '.'.join(filename.split(".")[:-1])
    location = '/'.join(filename_prefix.split("/")[:-1])
    filename_name = filename_prefix.split("/")[-1]

    files_json = {
        "input_file_int": filename,
        "input_file_bool": f"{location}/transformed_{filename_name}.structuredslugsplus",
        "input_file_backup_skill_added": f"{location}/backup_added_transformed_{filename_name}.structuredslugsplus", 
        "output_file_structuredslugsplus": f"{location}/repaired_transformed_{filename_name}.structuredslugsplus",
        "output_file_slugsin": f"{location}/repaired_transformed_{filename_name}.slugsin",
        "output_file_structuredslugs": f"{location}/repaired_transformed_{filename_name}.structuredslugs",
        "online_output_file_structuredslugsplus": f"{location}/online_repaired_transformed_{filename_name}.structuredslugsplus",
        "online_output_file_slugsin": f"{location}/online_repaired_transformed_{filename_name}.slugsin",
        "online_output_file_structuredslugs": f"{location}/online_repaired_transformed_{filename_name}.structuredslugs",
        "opts": f"{location}/opts.json",
        "json_data": f"{location}/{filename_name}.json",
        "modulo_spec_structuredslugsplus": f"{location}/modulo_transformed_{filename_name}.structuredslugsplus",
        "mapping_file": f"{location}/mapping_{filename_name}.json",
        "log_file": f"{location}/log_{filename_name}.json",
    }

    files_json_name = f"{location}/files_{filename_name}.json"
    with open(files_json_name, 'w') as f:
        json.dump(files_json, f, indent=4)

    if True:
        print(json.dumps(files_json, indent=4))