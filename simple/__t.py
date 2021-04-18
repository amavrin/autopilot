""" test """

import math
import matplotlib.pyplot as plt


R = 500
L = 500

X0 = 3000
Y0 = -1000
A0 = 0
rA0left = math.radians(A0) + math.pi/2
rA0right = math.radians(A0) - math.pi/2
Xr0left = X0 + math.cos(rA0left)*R
Yr0left = Y0 + math.sin(rA0left)*R
Xr0right = X0 + math.cos(rA0right)*R
Yr0right = Y0 + math.sin(rA0right)*R

X1 = 4000
Y1 = 0
A1 = 180

X1pre = X1 - math.cos(math.radians(A1))*L
Y1pre = Y1 - math.sin(math.radians(A1))*L

rA1left = math.radians(A1) + math.pi/2
rA1right = math.radians(A1) - math.pi/2
Xr1left = X1pre + math.cos(rA1left)*R
Yr1left = Y1pre + math.sin(rA1left)*R
Xr1right = X1pre + math.cos(rA1right)*R
Yr1right = Y1pre + math.sin(rA1right)*R

beta_left = math.atan2(Yr1left - Yr0left, Xr1left - Xr0left)
beta_right = math.atan2(Yr1right - Yr0right, Xr1right - Xr0right)

Xf0left = Xr0left + math.cos(beta_left - math.pi/2)*R
Yf0left = Yr0left + math.sin(beta_left - math.pi/2)*R
Xf1left = Xr1left + math.cos(beta_left - math.pi/2)*R
Yf1left = Yr1left + math.sin(beta_left - math.pi/2)*R

Xf0right = Xr0right + math.cos(beta_right + math.pi/2)*R
Yf0right = Yr0right + math.sin(beta_right + math.pi/2)*R
Xf1right = Xr1right + math.cos(beta_right + math.pi/2)*R
Yf1right = Yr1right + math.sin(beta_right + math.pi/2)*R

_, ax = plt.subplots()

ax.plot([X0, Xr0left, Xr0right, X1pre, X1, Xr1left, Xr1right],
         [Y0, Yr0left, Yr0right, Y1pre, Y1, Yr1left, Yr1right],
         'ro')
ax.plot([Xf0left, Xf1left], [Yf0left, Yf1left])
ax.plot([Xf0right, Xf1right], [Yf0right, Yf1right])

c00 = plt.Circle((Xr0left, Yr0left), R)
c01 = plt.Circle((Xr0right, Yr0right), R)
c10 = plt.Circle((Xr1left, Yr1left), R)
c11 = plt.Circle((Xr1right, Yr1right), R)

ax.add_patch(c00)
ax.add_patch(c01)
ax.add_patch(c10)
ax.add_patch(c11)

print(math.degrees(beta_left), math.degrees(beta_right))

plt.show()
