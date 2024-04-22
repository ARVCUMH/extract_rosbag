# you may configure a python virtual environment and a set of paths to automate the extraction of the rosbag
PYTHON_VIRTUAL_ENV='/home/arvc/Applications/venv/bin/python'

# indoor
rosbag_filename='/media/arvc/INTENSO/DATASETS/INDOOR/I3-2024-04-22-15-21-28.bag'
output_directory='/media/arvc/INTENSO/DATASETS/INDOOR/I3-2024-04-22-15-21-28'
$PYTHON_VIRTUAL_ENV extract_rosbag.py -i $rosbag_filename -o $output_directory

# O2
rosbag_filename='/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-04-22-13-27-47.bag'
output_directory='/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-04-22-13-27-47'
$PYTHON_VIRTUAL_ENV extract_rosbag.py -i $rosbag_filename -o $output_directory

# O3
rosbag_filename='/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-04-22-13-45-50.bag'
output_directory='/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-04-22-13-45-50'
$PYTHON_VIRTUAL_ENV extract_rosbag.py -i $rosbag_filename -o $output_directory
