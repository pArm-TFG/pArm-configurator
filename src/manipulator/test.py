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
    table.add(theta=Symbol("theta_1"), d=0, a=106.1, alpha=-(pi / 2)) \
        .add(theta=Symbol("theta_2"), d=13.2, a=142., alpha=0) \
        .add(theta=Symbol("theta_3"), d=0, a=158.9, alpha=0) \
        .add(theta=Symbol("theta_4"), d=0, a=44.5, alpha=0)
    # print(table.get())
    print(table)

    # calc_times = set()
    # sub_times = set()
    # for i in range(10):
    #     stt = time()
    #     m = Manipulator(params=table)
    #     stp = time()
    #     print("Elapsed time for calculating matrices: {:.3f}s".format(stp - stt))
    #     calc_times.add(stp - stt)
    #     # print(m["A04"])
    #     stt = time()
    #     rs = m.point(symbols={Symbol("theta_1"): 0,
    #                           Symbol("theta_2"): 0,
    #                           Symbol("theta_3"): pi,
    #                           Symbol("theta_4"): pi})
    #     stp = time()
    #     print("Elapsed time for applying matrix changes: {:.3f}s".format(stp - stt))
    #     sub_times.add(stp - stt)
    # print("Average calc. time: {:.3f}s".format(sum(calc_times) / 10))
    # print("Average sub. time: {:.3f}s".format(sum(sub_times) / 10))
    m = Manipulator(params=table)
    m.set_phi(Symbol("theta_2") + Symbol("theta_3") + Symbol("theta_4"))
    sol = m.solve((1, 1, 1), pi / 2)
    print(sol)
    # print(rs)
    # print(m.to_latrix('p', rs))
    # print(m.to_latrix('p', "A04"))
