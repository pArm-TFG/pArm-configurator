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
from . import Symbol
from . import pi
from . import Manipulator


def main():
    table = DHTable()
    table.add(theta=Symbol("t1"), d=0., a=5., alpha=(pi / 2))\
        .add(theta=Symbol("t2"), d=0., a=5, alpha=0, check_attrs=False)\
        .add(theta=0., d=Symbol("d2"), a=0, alpha=0)\
        .add(theta=Symbol("t4"), d=13.2, a=7., alpha=-pi / 2, check_attrs=False)
    # print(table.get())
    print(table)

    stt = time()
    m = Manipulator(params=table)
    stp = time()
    print(f"Elapsed time for calculating matrices: {stp - stt}s")
    print(m["A04"])
    stt = time()
    rs = m.apply(symbols={Symbol("t1"): 0, Symbol("t2"): 0, Symbol("d2"): pi,
                          Symbol("t4"): pi})
    stp = time()
    print(f"Elapsed time for applying matrix changes: {stp - stt}s")
    print(rs)
    print(m.to_latrix('p', "A04"))
