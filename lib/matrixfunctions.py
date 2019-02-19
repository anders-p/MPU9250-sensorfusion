""" Library of various functions to perform matrix operations in micropython

They are designed to work using two dimensional lists, or lists of lists
Generally the dimensions need to be given in order to perform the operation, this
may be removed in later versions

Author: Anders Appel
"""

""" Class to keep data and dimensions in one place"""
class matrix:
    def __init__(self, r, c, data=None):
        # r - number of rows of the matrix
        # c - number of columns
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

    """ Definition of element-wise division of two matrices"""
    def __floordiv__(self, other):
        # Check that dimensions match
        if self.columns == other.columns and self.rows == other.rows:
            # Create the output matrix
            result = matrix(self.rows, self.columns)

            # Cycle through and calculate each addition
            for r in range(self.rows):
                for c in range(self.columns):
                    if not other.getVal(r, c) == 0:
                        value = self.getVal(r, c) / other.getVal(r, c)
                    else:
                        value = 0
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
NOTE: This function ONLY works for square matrices with values on the diagonal, similar to an identity matrix
This was designed for a specific application where only this was required, and speed was important"""
def matInv(A):
    # Check that it is a square matrix for good habit
    if A.columns == A.rows:
        # Get the data from the matrix class
        data = A.get()

        # Perform the operations
        # for k in range(A.rows):
        #     data[k][k] = 1 / data[k][k]
        for r in range(A.rows):
            for c in range(A.columns):
                if r == c:
                    data[r][c] = 1 / data[r][c]
                else:
                    data[r][c] = 0

        # Store the data in a new matrix and return
        return matrix(A.rows, A.rows, data)
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
