#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gives dictionary specifying how to link topics together and do the input/output shaping

@author: Elian NEPPEL
@laboratory: Moonshot, Space Robotic Lab, Tohoku University
"""


# {Higher level pub  : Subscriber to the u2d2_dyna_controller}
angle_map = {
    f"read_joint{1}_{1}": f"angle_port_{2}_mot_{1}",
    f"read_joint{1}_{2}": f"angle_port_{2}_mot_{2}",
    f"read_joint{1}_{3}": f"angle_port_{2}_mot_{3}",
    f"read_joint{2}_{1}": f"angle_port_{0}_mot_{1}",
    f"read_joint{2}_{2}": f"angle_port_{0}_mot_{2}",
    f"read_joint{2}_{3}": f"angle_port_{0}_mot_{3}",
    f"read_joint{3}_{1}": f"angle_port_{3}_mot_{1}",
    f"read_joint{3}_{2}": f"angle_port_{3}_mot_{2}",
    f"read_joint{3}_{3}": f"angle_port_{3}_mot_{3}",
    f"read_joint{4}_{1}": f"angle_port_{1}_mot_{1}",
    f"read_joint{4}_{2}": f"angle_port_{1}_mot_{2}",
    f"read_joint{4}_{3}": f"angle_port_{1}_mot_{3}",
}

# {Higher level sub  : Publisher to the u2d2_dyna_controller}
set_map = {
    f"ang_joint{1}_{1}_set": f"set_port_{2}_mot_{1}",
    f"ang_joint{1}_{2}_set": f"set_port_{2}_mot_{2}",
    f"ang_joint{1}_{3}_set": f"set_port_{2}_mot_{3}",
    f"ang_joint{2}_{1}_set": f"set_port_{0}_mot_{1}",
    f"ang_joint{2}_{2}_set": f"set_port_{0}_mot_{2}",
    f"ang_joint{2}_{3}_set": f"set_port_{0}_mot_{3}",
    f"ang_joint{3}_{1}_set": f"set_port_{3}_mot_{1}",
    f"ang_joint{3}_{2}_set": f"set_port_{3}_mot_{2}",
    f"ang_joint{3}_{3}_set": f"set_port_{3}_mot_{3}",
    f"ang_joint{4}_{1}_set": f"set_port_{1}_mot_{1}",
    f"ang_joint{4}_{2}_set": f"set_port_{1}_mot_{2}",
    f"ang_joint{4}_{3}_set": f"set_port_{1}_mot_{3}",
}

# {Higher level sub  : input shaping funciton, applied very first on incoming angles}
# here I am applying a saturation on the angles
input_shaping_map = {
    f"ang_joint{1}_{1}_set": lambda x: min(max(x, -1.6), 1.6),
    f"ang_joint{1}_{2}_set": lambda x: min(max(x, -1.57), 1.5),
    f"ang_joint{1}_{3}_set": lambda x: min(max(x, -0.5), 1.9),

    f"ang_joint{2}_{1}_set": lambda x: min(max(x, -1.6), 1.6),
    f"ang_joint{2}_{2}_set": lambda x: min(max(x, -1.57), 1.5),
    f"ang_joint{2}_{3}_set": lambda x: min(max(x, -0.5), 1.9),

    f"ang_joint{3}_{1}_set": lambda x: min(max(x, -1.6), 1.6),
    f"ang_joint{3}_{2}_set": lambda x: min(max(x, -1.57), 1.5),
    f"ang_joint{3}_{3}_set": lambda x: min(max(x, -0.5), 1.9),

    f"ang_joint{4}_{1}_set": lambda x: min(max(x, -1.6), 1.6),
    f"ang_joint{4}_{2}_set": lambda x: min(max(x, -1.57), 1.5),
    f"ang_joint{4}_{3}_set": lambda x: min(max(x, -0.5), 1.9),
}

# {Higher level sub  : Set a offset on the angles (before the gain is applied)}
offset_map = {
    f"ang_joint{1}_{1}_set": 0,
    f"ang_joint{1}_{2}_set": 0,
    f"ang_joint{1}_{3}_set": 0,
    f"ang_joint{2}_{1}_set": 0,
    f"ang_joint{2}_{2}_set": 0,
    f"ang_joint{2}_{3}_set": 0,
    f"ang_joint{3}_{1}_set": 0,
    f"ang_joint{3}_{2}_set": 0,
    f"ang_joint{3}_{3}_set": 0,
    f"ang_joint{4}_{1}_set": 0,
    f"ang_joint{4}_{2}_set": 0,
    f"ang_joint{4}_{3}_set": 0,
}

# {Higher level sub  : Set a gain on the angles (can be negative to flip the motor)}
gain_map = {
    f"ang_joint{1}_{1}_set": 1,
    f"ang_joint{1}_{2}_set": 1,
    f"ang_joint{1}_{3}_set": 1,
    f"ang_joint{2}_{1}_set": 1,
    f"ang_joint{2}_{2}_set": 1,
    f"ang_joint{2}_{3}_set": 1,
    f"ang_joint{3}_{1}_set": 1,
    f"ang_joint{3}_{2}_set": 1,
    f"ang_joint{3}_{3}_set": 1,
    f"ang_joint{4}_{1}_set": 1,
    f"ang_joint{4}_{2}_set": 1,
    f"ang_joint{4}_{3}_set": 1,
}

# {Higher level sub  : output shaping function, applied on outgoing angles}
output_shaping_map = {
    f"ang_joint{1}_{1}_set": lambda x: x,
    f"ang_joint{1}_{2}_set": lambda x: x,
    f"ang_joint{1}_{3}_set": lambda x: x,

    f"ang_joint{2}_{1}_set": lambda x: x,
    f"ang_joint{2}_{2}_set": lambda x: x,
    f"ang_joint{2}_{3}_set": lambda x: x,

    f"ang_joint{3}_{1}_set": lambda x: x,
    f"ang_joint{3}_{2}_set": lambda x: x,
    f"ang_joint{3}_{3}_set": lambda x: x,

    f"ang_joint{4}_{1}_set": lambda x: x,
    f"ang_joint{4}_{2}_set": lambda x: x,
    f"ang_joint{4}_{3}_set": lambda x: x,

}
