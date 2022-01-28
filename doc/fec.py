
import numpy as np
#from tqdm import tqdm

import matplotlib.pyplot as plt
#matplotlib inline


def hammingDistance(n1, n2):
    return bin(np.bitwise_xor(n1, n2)).count("1")

N = 16  # binary code of length N
D = 5  # with minimum distance D

hamm_list = []
tested = [0]  # exclude 0 in codes
indices = np.arange(2**N)


required_size = 256

while len(hamm_list) < required_size:

    if len(tested + hamm_list) == 2**N:
        print(f"could not find subset of {required_size} in this simulation. Exiting...")
        break

    test_candidate = np.random.choice(np.setxor1d(indices, tested + hamm_list))

    valid = True
    for num in hamm_list:
        if hammingDistance(test_candidate, num) < D:
            valid = False
            break

    if valid:
        hamm_list.append(test_candidate)
        print("hamm_list size", len(hamm_list), test_candidate)
    else:
        tested += [test_candidate]

[format(num, f"0{N}b") for num in hamm_list]



#
# N = 8  # binary code of length N
# D = 4  # with minimum distance D
#
# M = 2**N  # number of unique codes in general
#
# # construct hamming distance matrix
# A = np.zeros((M, M), dtype=int)
# for i in range(M):
#     for j in range(i+1, M):
#         A[i, j] = hammingDistance(i, j)
# A += A.T
#
# print(A)

# plt.imshow(A); plt.colorbar();
# plt.title("Hamming Distances!", fontsize=13);



# MIN_GROUP_SIZE = 10
#
#
# def recursivly_find_legit_numbers(nums, codes=set(), groups=list()):
#
#     if len(groups) > 0 and len(groups[-1]) < MIN_GROUP_SIZE:
#         """Found enough subgroups for initial number i"""
#         return set()
#
#     unchecked = nums.copy()
#
#     for num1 in nums:
#
#         unchecked -= {num1}
#         candidate = unchecked.copy()
#         codes.add(num1)
#         for num2 in unchecked:
#             if A[num1, num2] < D:
#                 "Distance isn't sufficient, remove this number from set"
#                 candidate -= {num2}
#
#         if len(candidate) > 0:
#             codes = recursivly_find_legit_numbers(candidate, codes, groups)
#         else:
#             groups.append(codes)
#             codes = set(list(codes)[:-1])
#
#     return set()
