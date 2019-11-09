#                             manipulator
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
from typing import Union
from typing import Tuple
from typing import Dict
from typing import Any

from sympy import Matrix
from sympy import atan2

from . import to_latrix
from . import DHTable
from . import Symbol
from . import sin
from . import cos


class DirectKinematics:
    def __init__(self, params: DHTable, optimize: bool = True):
        self.params = params
        self.transformation_matrices: Dict[str, Matrix] = {}
        self._calc_matrices(optimize)

    def _calc_matrices(self, optimize: bool):
        for i, theta, d, a, alpha in self.params:
            self.transformation_matrices[f"A{i - 1}{i}"] = \
                self._matrix(theta, d, a, alpha)
        for i in range(2, self.params.max + 1):
            self.transformation_matrices[f"A0{i}"] = \
                self.transformation_matrices[f"A0{i - 1}"] * \
                self.transformation_matrices[f"A{i - 1}{i}"]
            if optimize:
                self.transformation_matrices[f"A0{i}"].simplify()

    def point(self, symbols: Dict[Symbol, Any], matrix_index: str = None) -> Matrix:
        if matrix_index is None:
            matrix_index = f"A0{self.params.max}"
        return self.transformation_matrices[matrix_index].subs(symbols)

    def __getitem__(self, item):
        return self.transformation_matrices.get(item)

    @staticmethod
    def _matrix(theta: Union[Symbol, float],
                d: Union[Symbol, float],
                a: Union[Symbol, float],
                alpha: Union[Symbol, float]) -> Matrix:
        return Matrix(
            [[cos(theta), - cos(alpha) * sin(theta), sin(alpha) * sin(theta),
              a * cos(theta)],
             [sin(theta), cos(alpha) * cos(theta), - sin(alpha) * cos(theta),
              a * sin(theta)],
             [0, sin(alpha), cos(alpha), d],
             [0, 0, 0, 1]])


class InverseKinematics:
    def __init__(self, direct_kinematics: DirectKinematics):
        def calc_add_of_squares():
            self._add = (self.Xe ** 2) + (self.Ye ** 2) + (self.Ze ** 2)
            self._add = self._add.simplify()

        self._end_effector_matrix = direct_kinematics[f"A0{direct_kinematics.params.max}"]
        self._phi_e = None
        self.params = direct_kinematics.params
        self.Xe = self._end_effector_matrix[0, 3]
        self.Ye = self._end_effector_matrix[1, 3]
        self.Ze = self._end_effector_matrix[2, 3]
        calc_add_of_squares()

    def set_phie(self, expression):
        self._phi_e = expression

    def solve(self, xyz: Tuple[float, float, float], phi: Symbol):
        from sympy.solvers import nonlinsolve
        system = [self.Xe - xyz[0], self.Ye - xyz[1], self.Ze - xyz[2], self._phi_e - phi]
        print("Solving equations:")
        for eq in system:
            print("\t{}".format(str(eq)))
        return nonlinsolve(system, list(self.params.symbols))


class Manipulator:
    def __init__(self, params: DHTable, optimize: bool = True):
        self.params = params
        self.direct_kinematics = DirectKinematics(params, optimize)
        self.inverse_kinematics = InverseKinematics(self.direct_kinematics)

    def point(self, symbols: Dict[Symbol, Any], matrix_index: str = None) -> Matrix:
        return self.direct_kinematics.point(symbols, matrix_index)

    def set_phi(self, expression):
        self.inverse_kinematics.set_phie(expression)

    def solve(self, xyz: Tuple[float, float, float], phi: Symbol):
        return self.inverse_kinematics.solve(xyz, phi)

    def to_latrix(self, matrix_type: str, matrix_index: str) -> str:
        return to_latrix(matrix_type, self.direct_kinematics[matrix_index])
