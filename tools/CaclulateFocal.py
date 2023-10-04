import numpy as np

bigX = 300
# Z in cm, 
val = np.array([
[500,380],[600,308],[700,266],[800,234],[900,206],[1000,187],[1100,173],[1200,159],[1300,144],[1400,134],[1500,128],[1600,120],[1700,113],[1800,107]
])



acc = 0

fs = []

for i in range(len(val)):
    z, x = val[i]
    value = x*(z/bigX)
    acc += value
    fs.append(value)

    print(acc/ i+1)


print(acc/len(val))
print(fs)

print(np.mean(np.array(fs)))
print(np.std(np.array(fs)))

