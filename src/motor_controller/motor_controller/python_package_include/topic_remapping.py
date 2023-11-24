#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gives dictionary specifying how to link topics together

@author: Elian NEPPEL
@laboratory: Moonshot, Space Robotic Lab, Tohoku University
"""


# Higher level pub  : Subscriber to the u2d2_dyna_controller
angle_map = {
    f"angle_{0}_{0}": f"angle_port_{2}_mot_{1}",
    f"angle_{0}_{1}": f"angle_port_{2}_mot_{2}",
    f"angle_{0}_{2}": f"angle_port_{2}_mot_{3}",

    f"angle_{1}_{0}": f"angle_port_{1}_mot_{1}",
    f"angle_{1}_{1}": f"angle_port_{1}_mot_{2}",
    f"angle_{1}_{2}": f"angle_port_{1}_mot_{3}",

    f"angle_{2}_{0}": f"angle_port_{0}_mot_{1}",
    f"angle_{2}_{1}": f"angle_port_{0}_mot_{2}",
    f"angle_{2}_{2}": f"angle_port_{0}_mot_{3}",

    f"angle_{3}_{0}": f"angle_port_{3}_mot_{1}",
    f"angle_{3}_{1}": f"angle_port_{3}_mot_{2}",
    f"angle_{3}_{2}": f"angle_port_{3}_mot_{3}",

}

# Higher level sub  : Publisher to the u2d2_dyna_controller
set_map = {
    f"set_joint_{0}_{0}_real": f"set_port_{2}_mot_{1}",
    f"set_joint_{0}_{1}_real": f"set_port_{2}_mot_{2}",
    f"set_joint_{0}_{2}_real": f"set_port_{2}_mot_{3}",

    f"set_joint_{1}_{0}_real": f"set_port_{1}_mot_{1}",
    f"set_joint_{1}_{1}_real": f"set_port_{1}_mot_{2}",
    f"set_joint_{1}_{2}_real": f"set_port_{1}_mot_{3}",

    f"set_joint_{2}_{0}_real": f"set_port_{0}_mot_{1}",
    f"set_joint_{2}_{1}_real": f"set_port_{0}_mot_{2}",
    f"set_joint_{2}_{2}_real": f"set_port_{0}_mot_{3}",

    f"set_joint_{3}_{0}_real": f"set_port_{3}_mot_{1}",
    f"set_joint_{3}_{1}_real": f"set_port_{3}_mot_{2}",
    f"set_joint_{3}_{2}_real": f"set_port_{3}_mot_{3}",

}
