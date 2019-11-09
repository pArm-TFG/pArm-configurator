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
from numpy import ndindex
from sympy import Matrix
from sympy import latex


def to_latrix(matrix_type: str, matrix: Matrix) -> str:
    """
    With a given Matrix, obtain its representation as a LaTeX matrix.
    :param matrix_type: the type of the matrix. Possible values can be:
    ['b', 'p', 'v', 'V', '']. View
    https://en.wikibooks.org/wiki/LaTeX/Mathematics#Matrices_and_arrays
    for more information.
    :param matrix: the Matrix from which obtain the representation.
    :return: the LaTeX string representation.
    """
    if len(matrix.shape) > 2:
        raise ValueError("LaTeX can display at most two dimensions")
    if matrix_type not in ('b', 'p', 'v', 'V', ''):
        raise ValueError("Matrix type must be: [b, p, v, V] or nothing ('')")
    row_values = [r"%\usepackage{amsmath}", r"\begin{" + matrix_type + "matrix}"]
    row = ""
    for x, y in ndindex(matrix.shape):
        row += ' ' + latex(matrix[x, y])
        if y < (matrix.shape[1] - 1):
            row += ' & '
        else:
            row += r" \\"
            row_values += [row]
            row = ""
    row_values += [r"\end{" + matrix_type + "matrix}"]
    return '\n'.join(row_values)
