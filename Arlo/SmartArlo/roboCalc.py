import numpy as np

def focal():
    bigX = 300 # mm højde på objekt
    Z = [500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800] # distance i mm for hvert billede
    smallX = [380,308,266,234,206,187,173,159,144,134,128,120,113,107] # størrelse af objekt på billede i pixels
    F = [0]*len(Z)
    # f = x*(Z/X)
    for i in range(len(Z)):
        F[i] = smallX[i]*(Z[i]/bigX)
        print(f"F[{i}] = {F[i]}")
    print(f"F = {F}")
    average = np.mean(F)
    print(f"Average F = {average}")
    std = np.std(F)
    print(f"std of F = {std}")
    return F

focal()
    