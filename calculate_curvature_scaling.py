import numpy as np

curvatures = np.asarray(np.linspace(-2.5, 2.5, 41))

print(curvatures)

max_curvature_max_speed = 1.5
min_curvature_max_speed = 5
max_curvature = 2.5
min_curvature = 0
f = lambda x : -abs(((1.5-3)/(2.5-0.1))*x)+3
g = lambda x : -((max_curvature_max_speed - min_curvature_max_speed) / (-((max_curvature)**2)))*x**2+5

print(g(curvatures))
