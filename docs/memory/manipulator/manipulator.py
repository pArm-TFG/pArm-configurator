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
from sympy import symbols
from sympy import simplify

from numbers import Number

from . import sin
from . import cos
from . import sqrt
from . import atan2
from . import Symbol
from . import DHTable
from . import to_latrix


class ForwardKinematics:
    """
    Container for the Forward Kinematics (FK) for an arbitrary manipulator.
    By using the Denavit-Hartenberg table, generates the required matrices
    in order to use them later.
    Refer to: https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    for more information.
    The accessible params are:
         - params: DHTable.
         - transformation_matrices: dict with the forward transformation matrices.
         - phi_e: expression for phi_e.
    Matrices are accessible by using square brackets: fk["A03"].
    """

    def __init__(self, params: DHTable, optimize: bool = True):
        """
        Generates a new instance for the class. It calculates the forward
        transformation matrices (symbolically) in order to use them later
        and not calculating them every time they are needed.
        :param params: the Denavit-Hartenberg params.
        :param optimize: whether to optimize or not the matrices - requires more
        computation time - default: True
        """
        self.params = params
        self.transformation_matrices: Dict[str, Matrix] = {}
        self._calc_matrices(optimize)
        self.phi_e = None

    def _calc_matrices(self, optimize: bool):
        """
        Internal function which iteratively calculates the required transformation
        matrices.
        :param optimize: whether to optimize or not the matrices.
        """
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

    def set_phi(self, expression: Union[Symbol, Number]):
        """
        Sets the phi_e expression.
        :param expression: expression - can be a Symbol or a number.
        """
        self.phi_e = expression

    def point(self,
              subs: Dict[Symbol, Any],
              matrix_index: str = None) -> Tuple[Number, Number, Number, Any]:
        """
        Obtain the (X, Y, Z, Phi) coordinates by changing the articulations.
        :param subs: the articulations' values.
        :param matrix_index: the transformation matrix in which apply the values.
        By default, it is the forward transformation matrix.
        :return: (X, Y, Z, Phi) as a tuple.
        """
        if matrix_index is None:
            matrix_index = f"A0{self.params.max}"
        return self.transformation_matrices[matrix_index].subs(subs)[0, 3], \
               self.transformation_matrices[matrix_index].subs(subs)[1, 3], \
               self.transformation_matrices[matrix_index].subs(subs)[2, 3], \
               self.phi_e.subs(subs) if self.phi_e is not None else None

    def __getitem__(self, item):
        return self.transformation_matrices.get(item)

    @staticmethod
    def _matrix(theta: Union[Symbol, float],
                d: Union[Symbol, float],
                a: Union[Symbol, float],
                alpha: Union[Symbol, float]) -> Matrix:
        """
        Forward transformation matrix template.
        :param theta: "theta" param.
        :param d: 'd' param.
        :param a: 'a' param.
        :param alpha: "alpha" param.
        :return: the forward transformation matrix.
        """
        return Matrix(
            [[cos(theta), - cos(alpha) * sin(theta), sin(alpha) * sin(theta),
              a * cos(theta)],
             [sin(theta), cos(alpha) * cos(theta), - sin(alpha) * cos(theta),
              a * sin(theta)],
             [0, sin(alpha), cos(alpha), d],
             [0, 0, 0, 1]])


class InverseKinematics:
    """
    Container for the Inverse Kinematics (IK) for an arbitrary manipulator.
    By using the Forward Kinematics for that manipulator, generates and
    calculates the Jacobian matrix that can be used for both direct
    manipulation and inverse manipulation, relating linear speed (end-effector)
    and angular speed (joints).

    The accessible params are:
     - params: the DHTable params.
     - Xe: expression for X.
     - Ye: expression for Y.
     - Ze: expression for Z.
     - det: the determinant of the Jacobian.
     - upper_jacobian: the first part of the Jacobian matrix (linear velocity).
     - lower_jacobian: the lower part of the Jacobian matrix (angular velocity).
     - m_jacobian: Jacobian matrix.
     - i_jacobian: inverse Jacobian.
     - pinv_jacobian: pseudo-inverse Jacobian.

    For accessing the inverse matrix, it is better to use the "inverse" property,
    as it will return the pseudo-inverse or the inverse, in case the latest one
    does not exists.
    """

    def __init__(self, forward_kinematics: ForwardKinematics, phi_e: dict = None):
        """
        Generates a new instance for the inverse kinematics class.
        :param forward_kinematics: the forward kinematics for the manipulator.
        :param phi_e: the Phi_e dict which relates the 'x', 'y' and 'z' expressions.
        """
        self._end_effector_matrix = forward_kinematics[
            f"A0{forward_kinematics.params.max}"]
        self._phi_e = phi_e if phi_e is not None else dict()
        self.params = forward_kinematics.params
        self.Xe = self._end_effector_matrix[0, 3]
        self.Ye = self._end_effector_matrix[1, 3]
        self.Ze = self._end_effector_matrix[2, 3]
        self.det = None
        self.upper_jacobian = None
        self.lower_jacobian = None
        self.m_jacobian = None
        self.i_jacobian = None
        self.pinv_jacobian = None

    def set_phi(self, xyz: str, expression: Union[Symbol, Number]):
        """
        Sets the Phi_e expression, which relates the angle to an axis.
        :param xyz: the axis in which update the expression - must be: {'x', 'y', 'z'}.
        :param expression: the expression for the Phi - can be a number or an expression.
        :raises AttributeError when xyz not in 'x', 'y', 'z'.
        """
        if xyz.lower() not in ['x', 'y', 'z']:
            raise AttributeError("xyz attribute must be ['x', 'y', 'z']")
        self._phi_e[xyz.lower()] = expression

    def jacobian(self, subs: list = None) -> Matrix:
        """
        Calculates the Jacobian matrix. If the determinant is '0', then it
        calculates the pseudo-inverse.
        :param subs: list of symbols that will be used for calculating the
        difference for the Jacobian.
        :return: the Jacobian matrix.
        """
        smatrix = Matrix([self.Xe,
                          self.Ye,
                          self.Ze,
                          self._phi_e['x'],
                          self._phi_e['y'],
                          self._phi_e['z']])
        if subs is None:
            subs = self.params.symbols
        self.m_jacobian = smatrix.jacobian(subs)
        self.upper_jacobian = self.m_jacobian[:3, :]
        self.lower_jacobian = self.m_jacobian[3:, :]
        self.det = self.upper_jacobian.det().simplify()
        if self.det != 0:
            self.i_jacobian = simplify(self.upper_jacobian ** -1)
        else:
            self.pinv_jacobian = self.upper_jacobian.pinv()
        return self.m_jacobian

    @property
    def inverse(self):
        """
        :return: the inverse Jacobian.
        """
        return self.pinv_jacobian if self.i_jacobian is None else self.i_jacobian


class UArmInverseKinematics:
    """
    uArm Inverse Kinematics class wrapper for the uArm robotic arm.
    Accessible values are:
     - X_e: X value.
     - Y_e: Y value.
     - Z_e: Z value.
     - phi: phi value.
     - theta_1: expression for theta_1.
     - theta_2: expression for theta_2.
     - theta_3: expression for theta_3.
    """

    def __init__(self, params: DHTable):
        """
        Generates a new instance for the uArmInverseKinematics class.
        :param params: the Denavit-Hartenberg params.
        """
        self.X_e, self.Y_e, self.Z_e, self.phi = symbols("X_e Y_e Z_e phi_e")
        cos_t3 = (
                (self.X_e ** 2) + (self.Z_e ** 2) -
                (params[1]['a'] ** 2) - (params[2]['a'] ** 2)
                /
                2 * params[1]['a'] * params[2]['a']
        )
        sin_t3 = (
            sqrt(1 - (cos_t3 ** 2))
        )
        self.theta_1 = atan2(self.Y_e, self.X_e + params.Tx + params[0]['d'])
        self.theta_3 = atan2(sin_t3, cos_t3)
        self.theta_2 = self.phi + self.theta_3

    def eval(self,
             Xe: Union[Symbol, Number],
             Ye: Union[Symbol, Number],
             Ze: Union[Symbol, Number],
             phi: Union[Symbol, Number]) -> Tuple[Union[Symbol, Number],
                                                  Union[Symbol, Number],
                                                  Union[Symbol, Number]]:
        """
        With a given point, returns the joints at which the robotic arm achieves
        that position.
        :param Xe: X position.
        :param Ye: Y position.
        :param Ze: Z position.
        :param phi: phi value.
        :return: (theta_1, theta_2, theta_3) as a tuple.
        """
        subs = {self.X_e: Xe, self.Y_e: Ye, self.Z_e: Ze, self.phi: phi}
        theta_1 = self.theta_1.subs(subs).evalf(chop=True)
        theta_3 = self.theta_3.subs(subs).evalf(chop=True)
        theta_2 = self.theta_2.subs(subs).evalf(chop=True)
        return theta_1, theta_2, theta_3


class Manipulator:
    """
    Wrapper class for working with the manipulator.
    Accessible params are:
     - direct_kinematics: the direct kinematics for the DHTable.
     - inverse_kinematics: the inverse kinematics for the DHTable.
     - uarm_ik: the uArm inverse kinematics.
    """

    def __init__(self, params: DHTable, optimize: bool = True):
        self.params = params
        self.direct_kinematics = ForwardKinematics(params, optimize)
        self.inverse_kinematics = InverseKinematics(self.direct_kinematics)
        self.uarm_ik = UArmInverseKinematics(params)

    def point(self, subs: Dict[Symbol, Any],
              matrix_index: str = None) -> Tuple[Number, Number, Number, Any]:
        """
        Obtain the (X, Y, Z, Phi) coordinates by changing the articulations.
        :param subs: the articulations' values.
        :param matrix_index: the transformation matrix in which apply the values.
        By default, it is the forward transformation matrix.
        :return: (X, Y, Z, Phi) as a tuple.
        """
        return self.direct_kinematics.point(subs, matrix_index)

    def set_phi(self, xyz: str, expression: Union[Symbol, Number]):
        """
        Sets the Phi_e expression, which relates the angle to an axis.
        :param xyz: the axis in which update the expression - must be: {'x', 'y', 'z'}.
        :param expression: the expression for the Phi - can be a number or an expression.
        :raises AttributeError when xyz not in 'x', 'y', 'z'.
        """
        self.inverse_kinematics.set_phi(xyz, expression)

    def jacobian(self, subs: list = None) -> Matrix:
        """
        Calculates the Jacobian matrix. If the determinant is '0', then it
        calculates the pseudo-inverse.
        :param subs: list of symbols that will be used for calculating the
        difference for the Jacobian.
        :return: the Jacobian matrix.
        """
        return self.inverse_kinematics.jacobian(subs)

    @property
    def inverse(self):
        """
        :return: the inverse Jacobian.
        """
        return self.inverse_kinematics.inverse

    def eval(self,
             Xe: Union[Symbol, Number],
             Ye: Union[Symbol, Number],
             Ze: Union[Symbol, Number],
             phi: Union[Symbol, Number]) -> Tuple[Union[Symbol, Number],
                                                  Union[Symbol, Number],
                                                  Union[Symbol, Number]]:
        """
        With a given point, returns the joints at which the robotic arm achieves
        that position.
        :param Xe: X position.
        :param Ye: Y position.
        :param Ze: Z position.
        :param phi: phi value.
        :return: (theta_1, theta_2, theta_3) as a tuple.
        """
        return self.uarm_ik.eval(Xe, Ye, Ze, phi)

    def to_latrix(self, matrix_type: str, matrix_index: str) -> str:
        """
        With a given Matrix, obtain its representation as a LaTeX matrix.
        :param matrix_type: the type of the matrix. Possible values can be:
        ['b', 'p', 'v', 'V', '']. View
        https://en.wikibooks.org/wiki/LaTeX/Mathematics#Matrices_and_arrays
        for more information.
        :param matrix: the Matrix from which obtain the representation.
        :return: the LaTeX string representation.
        """
        return to_latrix(matrix_type, self.direct_kinematics[matrix_index])
