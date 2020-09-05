import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("data.csv")

data1 = data[['x','v']]
data2 = data[['t','v']]
# print(data1.head())
# print(data2.head())

data1.plot(x = 'x', y = 'v')
data2.plot(x = 't', y = 'v')
plt.show()