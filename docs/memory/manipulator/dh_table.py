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
from typing import List
from typing import Union
from .symbols import Symbol


class DHTable:
    """
    Container class for the Denavit-Hartenberg table.
    Creates a data structure containing the necessary data for constructing
    the required matrix.
    """

    def __init__(self, table: List[dict] = None, check: bool = True):
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
            table = list()
            check = False
        if check:
            for value in table:
                assert len(value.keys()) == 4
        self.__table = table
        self.symbols = list()
        """
        List of symbols used in DH-Table
        """
        self.max = 0
        """
        How many items does the class have
        """
        self.Tx: float = 0.
        """
        Translation in 'X' axis
        """
        self.Ty: float = 0.
        """
        Translation in 'Y' axis
        """
        self.Tz: float = 0.
        """
        Translation in 'Z' axis
        """
        self.__lengths = [set() for _ in range(4)]

    @staticmethod
    def _check_errors(theta: Union[Symbol, float],
                      d: Union[Symbol, float],
                      a: Union[Symbol, float],
                      alpha: Union[Symbol, float]) -> bool:
        return (type(theta) is Symbol and (type(d) is Symbol or type(a) is Symbol or
                                           type(alpha) is Symbol)
                or type(d) is Symbol and (
                        type(theta) is Symbol or type(a) is Symbol or
                        type(alpha) is Symbol)
                or type(a) is Symbol and (type(d) is Symbol or type(theta) is
                                          Symbol or
                                          type(alpha) is Symbol)
                or type(alpha) is Symbol and (
                        type(d) is Symbol or type(a) is Symbol or
                        type(theta) is Symbol))

    def add(self,
            theta: Union[Symbol, float],
            d: Union[Symbol, float],
            a: Union[Symbol, float],
            alpha: Union[Symbol, float],
            check_attrs: bool = True) -> 'DHTable':
        """
        Add new params to the Denavit-Hartenberg table, in order. This method can safely
        be called by using the "Builder" structure (.add(...).add(...)...).
        By default, all params can be both "Symbol" or "float" but, if "check_attrs" is
        changed, only one can be a "Symbol". If not, it raises "AttributeError".

        :param theta: the parameter theta.
        :param d: the distance (elevation) between axes.
        :param a: the length of the segment.
        :param alpha: the angle between Zi and Zi+1 (radians).
        :param check_attrs: whether to perform a check or not - default: True
        :return: the class itself.
        :raises AttributeError when there is two or more params whose type is Symbol.
        Disable "check_attrs" for not throwing any exception.
        """
        if check_attrs:
            if self._check_errors(theta, d, a, alpha):
                raise AttributeError("Only one param can be a Symbol")

        self.__table.append({
            'a': a,
            'd': d,
            "alpha": alpha,
            "theta": theta
        })
        self.max += 1
        if type(theta) is Symbol:
            self.symbols.append(theta)
        if type(d) is Symbol:
            self.symbols.append(d)
        if type(a) is Symbol:
            self.symbols.append(a)
        if type(alpha) is Symbol:
            self.symbols.append(alpha)
        self.__lengths[0].add(len(str(theta)))
        self.__lengths[1].add(len(str(d)))
        self.__lengths[2].add(len(str(a)))
        self.__lengths[3].add(len(str(alpha)))
        return self

    def change(self, i: int, **kwargs):
        """
        Changes an existing param in the Denavit-Hartenber tables. 'i' (index) must
        exist.

        :param i: the table index (from 1 to n).
        :param kwargs: the keys to modify - [theta, d, a, alpha]
        :raises IndexError when the 'i' does not exist.
        """
        i -= 1
        for key, value in kwargs.items():
            if key not in self.__table[i].keys():
                raise KeyError(f"The key '{key}' is not a valid entry - it must be: "
                               f"[theta, d, a, alpha]")
            old_value = self.__table[i][key]
            self.__table[i][key] = value
            if type(value) is Symbol:
                if type(old_value) is Symbol:
                    self.symbols.insert(self.symbols.index(old_value), value)

    def remove(self, i: int) -> dict:
        """
        Removes an item from the Denavit-Hartenberg table.
        :param i: the item to remove (from 1 to n).
        :return: the removed item.
        :raises IndexError when the item does not exists.
        """
        i -= 1
        item = self.__table.pop(i)
        for key, value in item.items():
            if type(item[key]) is Symbol:
                self.symbols.remove(value)
        return item

    def get(self) -> List[dict]:
        """
        Obtains the table itself.
        :return: the Denavit-Hartenberg table.
        """
        return self.__table

    def __getitem__(self, item):
        assert isinstance(item, int)
        return self.__table[item]

    def __iter__(self):
        i = 0
        for element in self.__table:
            i += 1
            yield i, element["theta"], element['d'], element['a'], element["alpha"]

    def __str__(self):
        row_format = "{}{}{}{}{}".format("{:>4}",
                                         "{:>" + str(4 + max(self.__lengths[0])) + "}",
                                         "{:>" + str(4 + max(self.__lengths[1])) + "}",
                                         "{:>" + str(4 + max(self.__lengths[2])) + "}",
                                         "{:>" + str(4 + max(self.__lengths[3])) + "}")
        result = row_format.format('i', "t", "d", "a", "alpha") + "\n"
        i = 1
        for values in self.__table:
            result += row_format.format(str(i),
                                        str(values["theta"]),
                                        str(values['d']),
                                        str(values['a']),
                                        str(values["alpha"])) + "\n"
            i += 1
        return result
