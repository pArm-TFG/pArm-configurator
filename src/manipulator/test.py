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
from time import time

from . import DHTable
from . import pi
from . import Manipulator

from sympy import Matrix
from sympy import symbols
from sympy import simplify


def main():
    table = DHTable()
    t1, t2, t3 = symbols("theta_1 theta_2 theta_3")
    table.add(theta=t1, d=106.1, a=13.2, alpha=(pi / 2)) \
         .add(theta=t2, d=0, a=142, alpha=pi) \
         .add(theta=t3, d=0, a=158.9, alpha=0)
    table.Tx = 44.5
    table.Tz = -13.2
    print("Tabla de Denavit-Hartenberg")
    print(table)
    startt = time()
    manipulator = Manipulator(params=table)
    endt = time()
    print("Tiempo de obtención de las matrices: {:.3f}s".format(endt - startt))
    manipulator.direct_kinematics.set_phi(expression=t2 - t3)
    print("Estudio de diferentes casos")
    startt = time()
    c1 = manipulator.point({t1: 0, t2: 0, t3: 0})
    c2 = manipulator.point({t1: 0, t2: -pi / 2, t3: pi / 2 + 10})
    c3 = manipulator.point({t1: -pi / 2, t2: 0, t3: 0})
    c4 = manipulator.point({t1: pi, t2: pi / 2, t3: pi / 4})
    endt = time()
    print(f"q = (0, 0, 0)\n{c1}")
    print(f"q = (0, -pi/2, pi / 2 + 10)\n{c2}")
    print(f"q = (-pi/2, 0, 0)\n{c3}")
    print(f"q = (pi, pi/2, pi/4)\n{c4}")
    print("Tiempo de cómputo: {:.3f}s".format(endt - startt))

    print("Estudio de la inversa - si el resultado es un número imaginario, "
          "entonces es un punto al cual el robot no puede llegar")
    startt = time()
    i1 = manipulator.eval(c1[0], c1[1], c1[2], c1[3])
    i2 = manipulator.eval(c2[0], c2[1], c2[2], c2[3])
    i3 = manipulator.eval(c3[0], c3[1], c3[2], c3[3])
    i4 = manipulator.eval(c4[0], c4[1], c4[2], c4[3])
    endt = time()
    print(i1)
    print(i2)
    print(i3)
    print(i4)
    print("Tiempo de cómputo: {:.3f}s".format(endt - startt))

    print("Matriz Jacobiana")
    startt = time()
    manipulator.set_phi('x', t2 - t3)
    manipulator.set_phi('y', 0)
    manipulator.set_phi('z', t1)
    j = manipulator.jacobian(subs=[t1, t2, t3])
    endt = time()
    print(j)
    print(f"Jacobiana inversa:\n{manipulator.inverse}")
    print("Tiempo de cómputo: {:.3f}s".format(endt - startt))

    print("Estudio de casos para la matriz Jacobiana")
    startt = time()
    j1 = j.subs({t1: 0, t2: 0, t3: 0}) * Matrix([0, 0, 0])
    j2 = j.subs({t1: 0, t2: -pi / 2, t3: pi / 2 + 10}) * Matrix([0, -pi / 2, pi / 2 + 10])
    j3 = j.subs({t1: -pi / 2, t2: pi / 2, t3: pi / 2 + pi}) * Matrix([-pi / 2, pi / 2,
                                                                      pi / 2 + pi])
    j4 = j.subs({t1: pi, t2: 0, t3: 0}) * Matrix([pi, 0, 0])
    endt = time()
    print(f"q = (0, 0, 0)\n{j1}")
    print(f"q = (0, -pi/2, pi / 2 + 10)\n{j2}")
    print(f"q = (-pi/2, 0, 0)\n{j3}")
    print(f"q = (pi, 0,0)\n{j4}")
    print("Tiempo de cómputo: {:.3f}s".format(endt - startt))

    print("Matriz Jacobiana inversa")
    print(f"Determinante: {manipulator.inverse_kinematics.det}")

    print(simplify(manipulator.inverse * j1[:3, :]))
    print(simplify(manipulator.inverse * j2[:3, :]))
    print(simplify(manipulator.inverse * j3[:3, :]))
    print(simplify(manipulator.inverse * j4[:3, :]))
