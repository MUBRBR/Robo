import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures


# # Provided data
# X = np.array([
#     11.7, 10.74, 10.54, 9.8, 9.28, 8.58, 8.4, 7.42, 7.31, 6.8, 6.41, 5.7, 5.58, 5.06,
#     4.74, 4.27, 4.02, 3.73, 3.3, 3.07, 2.72, 2.49, 2.2, 1.97, 1.72, 1.52, 1.33, 1.14,
#     0.96, 0.79, 0.65, 0.53, 0.41, 0.31, 0.22, 0.15, 0.09
# ])

# t = np.array(range(400, 39, -10))

# # Reshape t to match X for model training
# t = t.reshape(-1, 1)

# # Create polynomial features
# poly_features = PolynomialFeatures(degree=2)
# t_poly = poly_features.fit_transform(t)

# # Create a polynomial regression model
# poly_model = LinearRegression()

# # Fit the model to the data
# poly_model.fit(t_poly, X)

# # Predict labels using the polynomial model
# predicted_labels = poly_model.predict(t_poly)



# # Plot the original data and the predicted labels
# plt.scatter(X, t, label='Original Data', color='blue')
# plt.plot(predicted_labels, t, label='Fitted model (Polynomial)', color='red')
# plt.xlabel('X')
# plt.ylabel('t')
# plt.legend()
# plt.show()


# def predict_t_values(X):

#     # Get the coefficients and y-intercept from the polynomial model
#     coef = [0.00000000e+00, -2.22966720e-03, 7.68889479e-05]
#     intercept = 0.0993244545876042

#     # Calculate the predicted t-values using the inverse transformation of the polynomial features
#     t_predicted = (-coef[1] + np.sqrt(coef[1]**2 - 4*coef[2]*(intercept - X))) / (2*coef[2])

#     return t_predicted

# predicted_x = np.array([predict_t_values(val) for val in X])
# # print(len(predicted_x))


# plt.axline((0, 0), (400, 400))
# plt.scatter(predicted_x, t, label='predict', color='blue')
# plt.xlabel('X')
# plt.ylabel('Predicted t')
# plt.legend()
# plt.show()

# x = 0.09
# y = 0.00000000e+00 + (-2.22966720e-03) * x + 7.68889479e-05 * x**2 + 0.0993244545876042
# print(y)




# Provided data
X = np.array([
    11.7, 10.74, 10.54, 9.8, 9.28, 8.58, 8.4, 7.42, 7.31, 6.8, 6.41, 5.7, 5.58, 5.06,
    4.74, 4.27, 4.02, 3.73, 3.3, 3.07, 2.72, 2.49, 2.2, 1.97, 1.72, 1.52, 1.33, 1.14,
    0.96, 0.79, 0.65, 0.53, 0.41, 0.31, 0.22, 0.15, 0.09
])

t = np.array(range(400, 39, -10))

plt.scatter(X, t)
# plt.show()

poly = PolynomialFeatures(degree=2, include_bias=False)
print(poly)
poly_features = poly.fit_transform(X.reshape(-1, 1))

poly_reg_model = LinearRegression()
poly_reg_model.fit(poly_features, t)

t_predicted = poly_reg_model.predict(poly_features)

plt.scatter(X, t)
plt.plot(X, t_predicted)
# plt.show()

def predict_t(x):
    return 51.93263227*x*-2.09414364*x**2+63.736734198956896

print(predict_t(11.7))
