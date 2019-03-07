""" Module to add a matrix class to micropython

This class is very basic, and is designed simply to provide functions such as matrix multiplication,
addition and subtraction. It is not optimised, and still may be added to later, as many operations
have yet to be defined.

Author: Anders Appel
"""

""" Class to keep data and dimensions in one place"""
class matrix:
    def __init__(self, r, c, data=None):
        # r - number of rows of the matrix
        # c - number of columns
        # data - 2D list containing data to be stored in matrix
        self.rows = r
        self.columns = c

        if data == None:
            # If no data is given, initialise as a zero array
            self.data = [[0] * c for _ in range(r)]
        else:
            # If data is given, simply transfer it
            # NOTE: this assumes that the dimensions given are correct
            self.data = data

    """ Function to store data in the matrix"""
    def store(self, value, r, c):
        self.data[r][c] = value
        return self.data

    """ Function to get a value from the matrix"""
    def getVal(self, r, c):
        return self.data[r][c]

    """ Function to get the entire matrix"""
    def get(self):
        return self.data

    """ Transpose property - returns the transpose of the matrix"""
    @property
    def transpose(self):
        # Initialise transpose matrix
        T = matrix(self.columns, self.rows)

        # Cycle through and fill
        for r in range(self.rows):
            for c in range(self.columns):
                T.store(self.getVal(r, c), c, r)

        return T

    """ Function to multiply by a scalar"""
    def scalarMult(self, val):
        for r in  range(self.rows):
            for c in range(self.columns):
                self.data[r][c] = self.data[r][c] * val
        return self.data

    """ Definition of the result of adding two matrices"""
    def __add__(self, other):
        # Check that dimensions match
        if self.columns == other.columns and self.rows == other.rows:
            # Create the output matrix
            result = matrix(self.rows, self.columns)

            # Cycle through and calculate each addition
            for r in range(self.rows):
                for c in range(self.columns):
                    value = self.getVal(r, c) + other.getVal(r, c)
                    result.store(value, r, c)

            return result
        else:
            raise ValueError('Matrix dimensions do not agree')

    """ Definition of the result of subtracting two matrices"""
    def __sub__(self, other):
        # Check that dimensions match
        if self.columns == other.columns and self.rows == other.rows:
            # Create the output matrix
            result = matrix(self.rows, self.columns)

            # Cycle through and calculate each addition
            for r in range(self.rows):
                for c in range(self.columns):
                    value = self.getVal(r, c) - other.getVal(r, c)
                    result.store(value, r, c)

            return result
        else:
            raise ValueError('Matrix dimensions do not agree')

""" Function to multiply two matrix class objects"""
def matMult(A, B):
    # Check that dimensions match and return and error if they don't
    if A.columns == B.rows:
        # Initialise the output matrix
        C = matrix(A.rows, B.columns)

        # Cycle through the output and calculate each value
        for r in range(C.rows):
            for c in range(C.columns):
                value = 0
                for k in range(A.columns):
                    value += A.getVal(r, k) * B.getVal(k, c)
                C.store(value, r, c)

        # Return the calculated output
        return C
    else:
        raise ValueError('Matrix dimensions do not agree')

""" Function to calculate the inverse
NOTE: This function ONLY works for square matrices """
def matInv(A):
    # Check that it is a square matrix for good habit
    if A.columns == A.rows:
        # Decompose the matrix and get the combined LU matrix
        (LU, P) = LUDecompose(A)

        # Separate the LU matrix into the L matrix and U matrix
        N = A.rows
        L = matrix(4, 4)
        U = matrix(4, 4)
        for r in range(N):
            for c in range(N):
                if r < c:
                    U.store(LU.getVal(r, c), r, c)
                elif r == c:
                    U.store(LU.getVal(r, c), r, c)
                    L.store(1, r, c)
                else:
                    L.store(LU.getVal(r, c), r, c)

        # Create the output matrix to store the inverse
        B = matrix(N, N)

        # Solve for each column of the inverse matrix
        for k in range(N):
            # Create the correct column of the identity matrix
            i = matrix(N, 1)
            i.store(1, k, 0)

            # Solve equation 1 - Lz = i
            z = forwardSub(L, i)

            # Solve equation 2 - Ub = z (b is a column of the inverse matrix B)
            b = backSub(U, z)

            # Store the values in the corresponding column
            for l in range(N):
                B.store(b.getVal(l, 0), l, P[k])

        # Store the data in a new matrix and return
        return B
    else:
        raise ValueError('Matrix must be square')

""" Function to decompose a matrix into lower and upper triangular matrices"""
def LUDecompose(A):
    # The matrix to decompose
    # Check that it is a square matrix for good habit
    if A.columns == A.rows:
        # Get the size of the matrix
        N = A.rows
        tol = 0.0001 # Tolerance of the calculation (to determine if the matrix is singular)

        # Initialise the list P
        P = [0]*N
        for i in range(N):
            P[i] = i

        # Get the data from the matrix class
        data = A.get()

        for i in range(N):
            maxA = 0 # Max value in each column
            imax = i # Index of max value

            # Find the max value (absolute)
            for k in range(i, N):
                if abs(data[k][i]) > maxA:
                    maxA = abs(data[k][i])
                    imax = k

            if maxA < tol:
                raise ValueError('Matrix is degenerate')
                return

            if not imax == i:
                # Pivot P
                temp = P[i]
                P[i] = P[imax]
                P[imax] = temp

                # Swap the rows of data accordingly
                temp = data[i]
                data[i] = data[imax]
                data[imax] = temp

            # Calculate each entry
            for j in range(i+1, N):
                data[j][i] /= data[i][i]
                for k in range(i+1, N):
                    data[j][k] -= data[j][i] * data[i][k]

        # Store the data in a new matrix and return
        return matrix(A.rows, A.rows, data), P
    else:
        raise ValueError('Matrix must be square')

""" Function to solve the equation Ax = b using forward substitution
NOTE: A must be a square lower triangular matrix"""
def forwardSub(A, b):
    # A - n x n lower triangular matrix
    # b - n x 1 matrix
    # x - n x 1 solution
    if A.columns == A.rows:
        N = A.rows

        # Create the output matrix
        x = matrix(N, 1)

        # Cycle through each row and solve
        for i in range(N):
            s = 0
            for j in range(i):
                s += A.getVal(i, j) * x.getVal(j, 0)
            val = (b.getVal(i, 0) - s) / A.getVal(i, i)
            x.store(val, i, 0)
        return x
    else:
        raise ValueError('Matrix must be square')

""" Function to solve the equation Ax = b using back substitution
NOTE: A must be a square upper triangular matrix"""
def backSub(A, b):
    # A - n x n lower triangular matrix
    # b - n x 1 matrix
    # x - n x 1 solution
    if A.columns == A.rows:
        N = A.rows

        # Create the output matrix
        x = matrix(N, 1)

        # Cycle through each row and solve
        for i in range(N-1, -1, -1):
            s = 0
            for j in range(N-1, i, -1):
                s += A.getVal(i, j) * x.getVal(j, 0)
            val = (b.getVal(i, 0) - s) / A.getVal(i, i)
            x.store(val, i, 0)
        return x
    else:
        raise ValueError('Matrix must be square')

""" Function to create an identity matrix"""
def eye(n):
    # n - Dimension of the matrix
    # Create an empty matrix
    A = matrix(n, n)

    for k in range(n):
        A.store(1, k, k)

    return A
