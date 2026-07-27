import numpy as np
import matplotlib.pyplot as plt


x=np.array([125,304.6,473.4,637.5,815.6,984.3])
y=np.array([50,100,150,200,250,300])

slope,intercept = np.polyfit(x,y,1)

print(f"a={slope},b={intercept}")

plt.scatter(x,y)
plt.plot(x,slope*x+intercept)

plt.show()