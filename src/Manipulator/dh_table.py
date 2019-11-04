#                             Manipulator
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
from sympy import Symbol


class DHTable:
    """
    Container class for the Denavit-Hartenberg table.
    Creates a data structure containing the necessary data for constructing
    the required matrix.
    """
    def __init__(self, table: dict = None, check: bool = True):
        """
        Creates a new instance for the class. If any argument is provided, then
        it checks if the dict is OK.
        :param table: a dictionary containing the DH table. It must have the following
        structure:
        {
            1: {
                'a': length,
                'd': distance,
                'alpha': angle with i + 1,
                'theta': arm angle (symbol)
            }, ...
        }
        :param check: skip the checking of the structure of the dictionary.
        """
        if table is None:
            table = dict()
            check = False
        if check:
            for key, value in table.items():
                assert isinstance(key, int)
                assert len(value.keys()) == 4
        self.__table = table
        self.__free_params = set()

    def set(self, i: int, theta: Symbol, d: Symbol, a: float, alpha: float):
        """
        Sets a new element inside de Denavit-Hartenberg table. If the element exists,
        then it is overwritten. In other case, it is created.
        Both "theta" and "d" can be Symbol (see: sympy.symbols), which means in test time
        they will be changed by the necessary value.
        :param i: the index position.
        :param theta: the parameter theta.
        :param d: the distance (elevation) between axes.
        :param a: the length of the segment.
        :param alpha: the angle between Zi and Zi+1 (radians).
        """
        self.__table[i] = {
            'a': a,
            'd': d,
            "alpha": alpha,
            "theta": theta
        }
        if type(theta) == Symbol:
            self.__free_params.add(theta)
        if type(d) == Symbol:
            self.__free_params.add(d)

    def get(self) -> dict:
        return self.__table

    def free_params(self) -> set:
        return self.__free_params
