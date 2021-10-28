# Python code to demonstrate calling the 
# function from another function
  
  
def SumofSquares(Array, n):
  
    # Initialize variable Sum to 0. It stores the 
    # Total sum of squares of the array of elements
    Sum = 0
    for i in range(n):
  
        # Square of Array[i] element is stored in SquaredValue
        SquaredValue = Square(Array[i])
  
        # Cummulative sum is stored in Sum variable
        Sum += SquaredValue
    return Sum

def Square(X):
    # computes the Square of the given number 
    # and return to the caller function
    return (X * X)
  
# Driver Function
Array = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
n = len(Array)
  
# Return value from the function 
# Sum of Squares is stored in Total
Total = SumofSquares(Array, n)
print("Sum of the Square of List of Numbers:", Total)