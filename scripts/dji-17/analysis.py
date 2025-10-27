import numpy as np

A_m = [[0.774427, -0.630701, 0.0497913],
       [0.686219, -0.722941, -0.0803753],
       [0.761352, -0.620098, -0.189265],
       [0.761374, -0.618448, -0.194506]]

A = np.array(A_m)

# 非方阵 → 做 A.T @ A，得到 3x3 方阵
A_square = A.T @ A

eigvals, eigvecs = np.linalg.eig(A_square)

print("特征值：", eigvals)
print("特征向量：\n", eigvecs)

