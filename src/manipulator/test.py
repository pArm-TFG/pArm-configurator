#                             src
#                  Copyright (C) 2019 - Javinator9889
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#      the Free Software Foundation, either version 3 of the License, or
#                   (at your option) any later version.
#
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#               GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#    along with this program. If not, see <http://www.gnu.org/licenses/>.
from numpy import ndindex
from time import time

from . import DHTable
from . import Symbol
from . import pi
from . import Manipulator
from . import ManualInverseKinematics
from . import cos
from . import sin
from . import to_latrix
from sympy import latex
from sympy import simplify
from sympy import symbols
from sympy import atan2
from sympy import sqrt
from collections import namedtuple


def main():
    table = DHTable()
    t1, t2, t3 = symbols("theta_1 theta_2 theta_3")
    table.add(theta=t1, d=106.1, a=13.2, alpha=(pi / 2)) \
        .add(theta=t2, d=0, a=142, alpha=pi) \
        .add(theta=t3, d=0, a=158.9, alpha=0)
    manipulator = Manipulator(params=table)
    print(manipulator.direct_kinematics["A03"])
    manipulator.direct_kinematics.set_phi(expression=t2 - t3)
    c1 = manipulator.point({t1: 0, t2: 0, t3: 0})
    c2 = manipulator.point({t1: 0, t2: -pi / 2, t3: pi / 2 + 10})
    c3 = manipulator.point({t1: -pi / 2, t2: 0, t3: 0})
    c4 = manipulator.point({t1: pi, t2: pi / 2, t3: pi / 4})
    print("(0, 0, 0)")
    print(c1)
    print("(0, -pi/2, pi / 2 + 10)")
    print(c2)
    print("(-pi/2, 0, 0)")
    print(c3)
    print("(pi, pi/2, pi/4)")
    print(c4)

    Xe, Ye, Ze, phi_23 = symbols("X_e Y_e Z_e phi_{23}")
    cos_t3 = ((Xe ** 2) + (Ze ** 2) - (142 ** 2) - (158.9 ** 2)) / (2 * 142 * 158.9)
    sin_t3 = sqrt(1 - (cos_t3 ** 2))
    Ik = namedtuple("Ik", ['t1', 't2', 't3'])
    inv = Ik(t1=atan2(Ye, Xe + 106.1),
             t2=phi_23 - t3,
             t3=atan2(sin_t3, cos_t3))
    p1 = {Xe: 13.2 - 158.9*cos(10), Ye: 0, Ze: 158.9*sin(10) - 35.9, phi_23: -10 - pi}
    print(f"t1: {inv.t1.subs(p1).evalf(chop=True)}")
    print(f"t3: {inv.t3.subs(p1).evalf(chop=True)}")
    rs = inv.t3.subs(p1)
    p1[t3] = rs
    print(f"t2: {inv.t2.subs(p1).evalf(chop=True)}")

    print(manipulator.point({t1: inv.t1.subs(p1).evalf(chop=True),
                   t2: inv.t2.subs(p1).evalf(chop=True),
                   t3: inv.t3.subs(p1).evalf(chop=True)}))
