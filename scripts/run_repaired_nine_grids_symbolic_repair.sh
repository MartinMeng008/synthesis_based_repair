#! /usr/bin/bash

python run_create_specification.py --user_spec "../data/repaired_nine_grids/repaired_nine_grids_a.json" --file_names "../data/repaired_nine_grids/repaired_nine_grids_files.json" --sym_opts "../data/repaired_nine_grids/repaired_nine_grids_sym_opts.json"
python run_symbolic_repair.py --user_spec "../data/repaired_nine_grids/repaired_nine_grids_a.json" --file_names "../data/repaired_nine_grids/repaired_nine_grids_files.json" --sym_opts "../data/repaired_nine_grids/repaired_nine_grids_sym_opts.json"