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

from sympy import sin
from sympy import cos
from sympy import pi

from . import DHTable
from . import Symbol


class Manipulator:
    def __init__(self, params: DHTable):
        self.params = params
        self.__transformation_matrices = {}
        self.__calc_matrices()

    def __calc_matrices(self):
        for key, value in self.params.get():
            self.__transformation_matrices[f"A{key - 1}{key}"] =


def mtr_matrix(i: int, alpha: float, a: float, d: float) -> np.array:
    # theta = symbols(f"theta{i}")
    theta = None
    return np.array([[cos(theta), - cos(alpha) * sin(theta), sin(alpha) * sin(theta),
                      a * cos(theta)],
                     [sin(theta), cos(alpha) * cos(theta), - sin(alpha) * cos(theta),
                      a * sin(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])
