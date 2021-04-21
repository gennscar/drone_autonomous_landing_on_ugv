#!/usr/bin/env python3 

from common_modules import gauss_newton_trilateration
from common_modules import standard_trilateration

import numpy as np 

d0 = 4.623587856728445
d1 = 6.2645246208829475
d2 = 5.46944112933879
d3 = 5.57445401697534

gn_pos = [0.1,-0.1,0.15]
i = 1

while i < 1000:
    gn_pos = gauss_newton_trilateration.trilateration(gn_pos, d0, d1, d2, d3)

    i = i+1

print(gn_pos)

std_pos = standard_trilateration.trilateration(d0,d1,d2,d3)
print(std_pos)