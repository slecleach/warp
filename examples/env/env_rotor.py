#%%
# Copyright (c) 2022 NVIDIA CORPORATION.  All rights reserved.
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

###########################################################################
# Rotor environment
###########################################################################

import os
import math
import warp as wp
import warp.sim
import matplotlib.pyplot as plt
import numpy as np

from environment import Environment, run_env, RenderMode, IntegratorType


class RotorEnvironment(Environment):
    sim_name = "env_rotor"
    env_offset = (2.0, 0.0, 2.0)
    opengl_render_settings = dict(scaling=1.0)
    usd_render_settings = dict(scaling=100.0)

    sim_substeps_euler = 32
    sim_substeps_xpbd = 5

    xpbd_settings = dict(
        iterations=2,
        joint_linear_relaxation=0.7,
        joint_angular_relaxation=0.5,
        rigid_contact_relaxation=1.0,
        rigid_contact_con_weighting=True,
    )

    def create_articulation(self, builder):
        wp.sim.parse_mjcf(
            os.path.join(os.path.dirname(__file__), "../assets/rotor.xml"),
            builder,
            stiffness=0.0,
            damping=0.1,
            armature=0.007,
            armature_scale=10.0,
            contact_ke=1.0e4,
            contact_kd=1.0e2,
            contact_kf=1.0e2,
            contact_mu=0.5,
            contact_restitution=0.0,
            limit_ke=1.0e2,
            limit_kd=1.0e1,
            enable_self_collisions=True,
            up_axis="y",
        )

        builder.joint_q[:] = [0.0]
        builder.joint_qd[:] = [-2.0]

#%%     
env = RotorEnvironment()
env.integrator_type = IntegratorType.XPBD
env.render_mode = RenderMode.NONE
env.num_envs = 1
env.profile = False
env.plot_joint_coords = True
env.init()

#%%
plot_joint_coords = env.run()
plt.plot(plot_joint_coords)
plt.xlabel("time_step")
plt.ylabel("angle in radian")
#%%  
# if __name__ == "__main__":
    # run_env(RotorEnvironment)

# %%
