from scipy.optimize import curve_fit
import numpy as np
import matplotlib.pyplot as plt


def aff_cst(t, a, b, T):
    return np.amax([(T - t)/np.abs(T - t) * a * t, (t - T)/np.abs((T - t) * b)], axis=0)


time = np.linspace(0, 20, 500)
plt.plot(time, np.arctan(time))


popt, pcov = curve_fit(aff_cst, time, np.arctan(time)) #, bounds=[[-np.inf, -np.inf, 1.0], [np.inf, np.inf, 3.0]])

plt.plot(time, aff_cst(time, *popt))

plt.show()