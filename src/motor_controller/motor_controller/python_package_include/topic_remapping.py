#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gives dictionary specifying how to link topics together

@author: Elian NEPPEL
@laboratory: Moonshot, Space Robotic Lab, Tohoku University
"""


# Higher level pub  : Subscriber to the u2d2_dyna_controller
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

# Higher level sub  : Publisher to the u2d2_dyna_controller
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

# Higher level sub  : Publisher to the u2d2_dyna_controller
gain_map = {
    f"ang_joint{1}_{1}_set": -1,
    f"ang_joint{1}_{2}_set": -1,
    f"ang_joint{1}_{3}_set": 1,

    f"ang_joint{2}_{1}_set": -1,
    f"ang_joint{2}_{2}_set": -1,
    f"ang_joint{2}_{3}_set": -1,

    f"ang_joint{3}_{1}_set": -1,
    f"ang_joint{3}_{2}_set": -1,
    f"ang_joint{3}_{3}_set": -1,

    f"ang_joint{4}_{1}_set": -1,
    f"ang_joint{4}_{2}_set": -1,
    f"ang_joint{4}_{3}_set": -1,
}
