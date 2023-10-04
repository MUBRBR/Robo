import numpy as np
from sklearn import linear_model

# data er målte distancer og den hastighed distancen er kørt ved
data = np.array([[100,64],[200,164],[300,64]])

#labels er tid vi har målt at det tager at køre en distance
labels = np.array([1,1,3])

#Sklearn magic
reg = linear_model.LinearRegression()
reg.fit(data,labels)
print(reg.coef_)

#Hvis vi skal køre 200m, ved hastighed 64, hvor mange sekunder tager det?
print(np.array(reg.coef_)@np.array([200,64]))