#!/usr/bin/env python3
import argparse
import os
import shutil
from dataclasses import dataclass
from typing import cast, Dict, List, Mapping
import duckietown_challenges as dc
import numpy as np
from geometry import SE2
from matplotlib import pyplot
from duckietown_world import pose_from_friendly
from zuper_commons.fs import AbsDirPath, AbsFilePath, DirPath, FilePath, locate_files, read_ustring_from_utf8_file
from zuper_ipce import IEDO, IESO, ipce_from_object, object_from_ipce
from collision_checker import check_collision
import yaml
from dt_protocols import (CollisionCheckQuery, CollisionCheckResult, logger, MapDefinition,
                          plot_geometry, protocol_collision_checking)
from matplotlib import pyplot

COLOR_BG = '#ccc9c6'
COLOR_COLLISION = 'red'
COLOR_COLLISION_WRONG = 'pink'
COLOR_NOCOLLISION = 'blue'
COLOR_NOCOLLISION_WRONG = 'orange'
COLOR_OBSTACLES = '#824504'

coll_colors = {
    (True, True): COLOR_COLLISION,
    (False, False): COLOR_NOCOLLISION,
    (True, False): COLOR_COLLISION_WRONG,
    (False, True): COLOR_NOCOLLISION_WRONG,

}

def check_and_mate(dirname):
    a = locate_files(dirname, "*.tests.yaml")
    H = W = 5
    for fn in a:
        data = read_ustring_from_utf8_file(fn)
        ydata = yaml.load(data, Loader=yaml.Loader)

        
        @dataclass
        class Interaction:
            query: CollisionCheckQuery
            gt: CollisionCheckResult
            
        @dataclass
        class Data:
            params: MapDefinition
            interactions: List[Interaction]

        inside = object_from_ipce(ydata, Data)

        f = pyplot.figure()
        pyplot.tight_layout()  # rect=(0, 0, W, H))
        ax: pyplot.Axes = pyplot.gca()
        plot_geometry(ax, SE2.identity(), inside.params.environment, COLOR_OBSTACLES, 0)
        ax.add_artist(pyplot.Rectangle((0, 0), width=W, height=H, color=COLOR_BG, zorder=-10))
        
        for interaction in inside.interactions:
            collided = check_collision(
                Wcoll=inside.params.environment, robot_body=inside.params.body, robot_pose=interaction.query.pose
            )
            if collided == interaction.gt.collision:
                print(f'OK TEST result {fn} {collided} {interaction.gt.collision}')
            else:
                print(f'FAIL TEST result {fn} {collided} {interaction.gt.collision}')
            color = coll_colors[(interaction.gt.collision,collided)]
            p1 = interaction.query.pose
            s0 = pose_from_friendly(p1)
            plot_geometry(ax, s0, inside.params.body, color, 10)
        ax.set_aspect(1.0)
        pyplot.axis((0, W, 0, H))
        pyplot.axis("off")
        pyplot.savefig(os.path.basename(fn)+".png", dpi=200)
        pyplot.close(f)


check_and_mate('data')
