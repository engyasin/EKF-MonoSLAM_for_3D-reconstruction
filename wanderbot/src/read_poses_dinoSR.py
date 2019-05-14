import numpy as np
Ks,Rs,Ts=[],[],[]
with open('dinoSR_par.txt') as f:
    for line in f:
        krt=[float(x)for x in line.split()[1:]]
        Ks.append(np.array(krt[:9]).reshape((3,3)))
        Rs.append(np.array(krt[9:18]).reshape((3,3)))
        Ts.append(np.array(krt[18:]).reshape((3,1)))

