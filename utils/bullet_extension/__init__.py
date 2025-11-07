import atexit

import pybullet as p
import pybullet_data


# Panda: https://github.com/PaulPauls/franka_emika_panda_pybullet


def launch(connect_method: int = p.GUI,
           time_step: float = 1e-3,
           urdf_path: list[str] = ["plane.urdf"]):
    """ Launch a PyBullet simulation environment. """
    p.connect(connect_method)
    p.setTimeStep(time_step)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    for up in urdf_path: p.loadURDF(up)
    atexit.register(p.disconnect)


if __name__ == '__main__':
    launch()
