import numpy as np
from scipy.stats import norm, skew, kurtosis
import matplotlib.pyplot as plt

counts = np.array([1724,5718,14432,27180,45620,67618,87847,97498,107004,
                   97629,86594,68535,45971,28328,15355,5270,1643])
k = np.arange(len(counts)); N = counts.sum()

mu = (k*counts).sum()/N
sigma = np.sqrt(((k-mu)**2*counts).sum()/N)
expected = N*(norm.cdf((k+.5-mu)/sigma) - norm.cdf((k-.5-mu)/sigma))

print(f"mean={mu:.3f} sigma={sigma:.3f}")
print("max error / peak: %.1f%%" % (100*np.abs(counts-expected).max()/counts.max()))
samples = np.repeat(k, counts)
print("skew=%.3f  kurtosis=%.3f (Gaussian=3)" % (skew(samples), kurtosis(samples, fisher=False)))

fig, ax = plt.subplots(1, 2, figsize=(10,4))
for a, log in zip(ax, [False, True]):
    a.bar(k, counts, alpha=.6, label="measured")
    a.plot(k, expected, "r.-", label="Gaussian fit")
    if log: a.set_yscale("log")
    a.legend()
plt.show()