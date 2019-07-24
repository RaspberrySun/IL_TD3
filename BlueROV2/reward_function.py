import numpy as np
from math import sqrt, exp
from numpy import linalg, array


def reward_three_dof(delta_z, delta_phi, delta_theta, w, p, q, law, tau):
    if abs(delta_z) <= 0.05 and abs(delta_phi) <= 0.01 and abs(delta_theta) <= 0.01:
        return -0.012*sum([abs(t) for t in tau]) - 2e-3*linalg.norm(array(law)-array(tau)) - 0.1*(w**2 + p**2 + q**2)
    if abs(delta_z) >= 2.0 or abs(delta_phi) >= 0.2 or abs(delta_theta) >= 0.2:
        return -300
    else:
        Delta_Matrix = np.mat([delta_z, delta_phi, delta_theta]) * np.diag([3.33, 10, 10]) * np.mat([[delta_z], [delta_phi], [delta_theta]])
        return -5*(Delta_Matrix[0, 0])-0.015*sum([abs(t) for t in tau]) - 1.5e-3*linalg.norm(array(law)-array(tau)) - 0.1*(w**2 + p**2 + q**2)


if __name__ == '__main__':
    print(reward_three_dof(1.99, 0., 0., 0, 0, 0, tau=[-60, -60, -60, -60], law=[60, 60, 60, 60]))

