import os
import csv
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
from functions.export_and_plot_shape import export_and_plot_shape
from functions.trajectories import *
from functions.create_active_csv import create_active_csv

# Ensure the working directory is correct
os.chdir("/home/seiger/mavsdk_drone_show")
print("Current Working Directory:", os.getcwd())

# Ensure the shapes folder exists
if not os.path.exists("shapes"):
    os.makedirs("shapes")

# Example usage
num_repeats = 1
shape_name = "heart_shape"
diameter = 30.0
direction = 1
maneuver_time = 60.0
start_x = 0
start_y = 0
initial_altitude = 15
climb_rate = 1.0
move_speed = 2.0  # m/s
hold_time = 4.0  # s
step_time = 0.05  # s

# Use absolute path for the output file
output_file = "/home/seiger/mavsdk_drone_show/shapes/newposi.csv"

create_active_csv(
    shape_name=shape_name,
    num_repeats=num_repeats,
    diameter=diameter,
    direction=direction,
    maneuver_time=maneuver_time,
    start_x=start_x,
    start_y=start_y,
    initial_altitude=initial_altitude,
    climb_rate=climb_rate,
    move_speed=move_speed,
    hold_time=hold_time,
    step_time=step_time,
    output_file=output_file,
)

export_and_plot_shape(output_file)
