
def Det(matrix):
    isMatrix(matrix)
    if len(matrix[0]) != len(matrix):
        raise ArithmeticError("The determinant is undefined for a non-square matrix")
    #Check if matrix is a 2 x 2
    if len(matrix[0]) == 2:
        return (matrix[0][0]*matrix[1][1] -
                matrix[0][1]*matrix[1][0])
    
    else:
        determinant = 0
        for index in range(0,len(matrix[0])):
            if index % 2 == 0:
                newMatrix = FirstSubMatrix(0,index,matrix)
                determinant += matrix[0][index]*Det(newMatrix)
            else:
                newMatrix = FirstSubMatrix(0,index,matrix)
                determinant -= matrix[0][index]*Det(newMatrix)
                
        return determinant
    
def FirstSubMatrix(i,j,matrix):
    isMatrix(matrix)
    if len(matrix) == 2:
        return matrix
    #i,j starts at 0,0 like list indices not 1,1
    indices = [ind for ind in range(0,len(matrix))]
    indices.remove(j)
    m = list(matrix)
    m.remove(matrix[i])
    newMatrix = []
    for row in m:
        newRow = []
        for rowIndex in indices:
            newRow.append(row[rowIndex])
        newMatrix.append(newRow)
    return newMatrix

def Cofactor(matrix):
    isMatrix(matrix)
    if len(matrix) == 2:
        return [[matrix[1][1], -matrix[1][0]],[-matrix[0][1],matrix[0][0]]]
    cofactor = []
    for i in range(0,len(matrix)):
        row = []
        for j in range(0, len(matrix)):
            if (i+j)%2 == 0:
                row.append(Det(FirstSubMatrix(i,j,matrix)))
            else:
                row.append(Det(FirstSubMatrix(i,j,matrix))*(-1))
        cofactor.append(row)
    return cofactor

def Transpose(matrix):
    isMatrix(matrix)
    transpose = []
    for i in range(0, len(matrix[0])):
        row = []
        for j in range(0, len(matrix)):
            row.append(matrix[j][i])
        transpose.append(row)
    return transpose
        
def Inv(matrix):
    isMatrix(matrix)
    cofactor = Cofactor(matrix)
    det = Det(matrix)
    if det == 0:
        raise ArithmeticError('A matrix with a determinant of 0 does not have an inverse')
    inverse = []
    for row in Transpose(cofactor):
        newRow = []
        for j in range(0, len(row)):
            newRow.append(row[j]/det)
        inverse.append(newRow)
    return inverse

def Multiply(A,B):
    isMatrix(A)
    isMatrix(B)
    bTrans = Transpose(B)
    if len(A[0]) != len(bTrans[0]):
        raise ArithmeticError(
'Cannot multiply matrices where the number of colums in A does not equal the number of Rows in B')
    C = []
    for row in A:
        newRow = []
        for bRow in bTrans:
            value = 0;
            for index in range(0, len(row)):
                value += row[index]*bRow[index]
            newRow.append(value)
        C.append(newRow)
    return C

def isMatrix(matrix):
    try:
        if len(matrix) > 1 or len(matrix[0]) > 1:
            return True
        else:
            raise ValueError('Matrix must be at least 2x2')
        
    except ValueError as error:
        raise error
    except TypeError:
        raise TypeError('A matrix is input in the form of a list of lists [[a,b],[c,d]]')
            