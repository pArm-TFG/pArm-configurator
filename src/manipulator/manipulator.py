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

from numbers import Number

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
        self.transformation_matrices[f"A0{self.params.max}"][0, 3] += self.params.Tx
        self.transformation_matrices[f"A0{self.params.max}"][1, 3] += self.params.Ty
        self.transformation_matrices[f"A0{self.params.max}"][2, 3] += self.params.Tz

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
    def __init__(self, direct_kinematics: DirectKinematics, phi_e: dict = None):
        self._end_effector_matrix = direct_kinematics[f"A0{direct_kinematics.params.max}"]
        self._phi_e = phi_e if phi_e is not None else dict()
        self.params = direct_kinematics.params
        self.Xe = self._end_effector_matrix[0, 3]
        self.Ye = self._end_effector_matrix[1, 3]
        self.Ze = self._end_effector_matrix[2, 3]
        self.det = None
        self.upper_jacobian = None
        self.lower_jacobian = None
        self.m_jacobian = None

    def set_phi(self, xyz: str, expression: Union[Symbol, Number]):
        if xyz.lower() not in ['x', 'y', 'z']:
            raise AttributeError("xyz attribute must be ['x', 'y', 'z']")
        self._phi_e[xyz.lower()] = expression

    def jacobian(self, symbols: list = None) -> Matrix:
        smatrix = Matrix([self.Xe,
                          self.Ye,
                          self.Ze,
                          self._phi_e['x'],
                          self._phi_e['y'],
                          self._phi_e['z']])
        if symbols is None:
            symbols = self.params.symbols
        self.m_jacobian = smatrix.jacobian(symbols)
        self.upper_jacobian = self.m_jacobian[:3, :]
        self.lower_jacobian = self.m_jacobian[3:, :]
        self.det = self.upper_jacobian.det().simplify()
        return self.m_jacobian


class Manipulator:
    def __init__(self, params: DHTable, optimize: bool = True):
        self.params = params
        self.direct_kinematics = DirectKinematics(params, optimize)
        self.inverse_kinematics = InverseKinematics(self.direct_kinematics)

    def point(self, symbols: Dict[Symbol, Any], matrix_index: str = None) -> Matrix:
        return self.direct_kinematics.point(symbols, matrix_index)

    def set_phi(self, xyz: str, expression: Union[Symbol, Number]):
        self.inverse_kinematics.set_phi(xyz, expression)

    def jacobian(self, symbols: list = None) -> Matrix:
        return self.inverse_kinematics.jacobian(symbols)

    def solve(self, xyz: Tuple[float, float, float], phi: Symbol):
        pass
        # return self.inverse_kinematics.solve(xyz, phi)

    def to_latrix(self, matrix_type: str, matrix_index: str) -> str:
        return to_latrix(matrix_type, self.direct_kinematics[matrix_index])
