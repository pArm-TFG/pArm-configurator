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
import numpy as np

from typing import Union
from typing import Dict

from sympy import latex
from sympy import Matrix

from . import DHTable
from . import Symbol
from . import sin
from . import cos
from . import pi


class Manipulator:
    def __init__(self, params: DHTable, optimize: bool = True):
        self.params = params
        self.__transformation_matrices: Dict[str, Matrix] = {}
        self.__calc_matrices(optimize)

    def __calc_matrices(self, optimize: bool):
        for i, theta, d, a, alpha in self.params:
            self.__transformation_matrices[f"A{i - 1}{i}"] = \
                self._matrix(theta, d, a, alpha)
        for i in range(2, self.params.max + 1):
            self.__transformation_matrices[f"A0{i}"] = \
                self.__transformation_matrices[f"A0{i - 1}"] * \
                self.__transformation_matrices[f"A{i - 1}{i}"]
            if optimize:
                self.__transformation_matrices[f"A0{i}"].simplify()

    def apply(self, symbols: dict, transformation_matrix: str = None) -> np.array:
        if transformation_matrix is None:
            transformation_matrix = f"A0{self.params.max}"
        return self.__transformation_matrices[transformation_matrix].subs(symbols)

    def to_latrix(self, matrix_type: str, item: Union[str, Matrix]) -> str:
        if isinstance(item, str):
            matrix = self.__transformation_matrices[item]
        else:
            matrix = item
        if len(matrix.shape) > 2:
            raise ValueError("LaTeX can display at most two dimensions")
        if matrix_type not in ('b', 'p', 'v', 'V', ''):
            raise ValueError("Matrix type must be: [b, p, v, V] or nothing ('')")
        row_values = [r"%\usepackage{amsmath}", r"\begin{" + matrix_type + "matrix}"]
        row = ""
        for x, y in np.ndindex(matrix.shape):
            row += ' ' + latex(matrix[x, y])
            if y < (matrix.shape[1] - 1):
                row += ' & '
            else:
                row += r" \\"
                row_values += [row]
                row = ""
        row_values += [r"\end{" + matrix_type + "matrix}"]
        return '\n'.join(row_values)

    def __getitem__(self, item):
        return self.__transformation_matrices.get(item)

    @staticmethod
    def _matrix(theta: Union[Symbol, float],
                d: Union[Symbol, float],
                a: Union[Symbol, float],
                alpha: Union[Symbol, float]) -> Matrix:
        return Matrix([[cos(theta), - cos(alpha) * sin(theta), sin(alpha) * sin(theta),
                          a * cos(theta)],
                         [sin(theta), cos(alpha) * cos(theta), - sin(alpha) * cos(theta),
                          a * sin(theta)],
                         [0, sin(alpha), cos(alpha), d],
                         [0, 0, 0, 1]])
