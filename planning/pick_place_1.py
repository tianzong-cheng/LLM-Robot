from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse

import numpy as np
from controllers.pick_place import PickPlaceController
from omni.isaac.core import World
from tasks.pick_place import PickPlace
import time, os

parser = argparse.ArgumentParser()
parser.add_argument(
    "--test", default=False, action="store_true", help="Run in test mode"
)
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
