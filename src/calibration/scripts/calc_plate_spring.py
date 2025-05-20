# calculate plate spring

import numpy as np
import matplotlib.pyplot as plt

def calcforward_simple():
    t = 0.5
    b = 25
    l = 48
    n = 2
    E = 186000 # [N/mm2]
    nyu = 0.3
    sigma_max = 791 # [N/mm2]
    
    I = b*t**3/12
    f = 1.05
    
    k = n * (E*b*t**3) / (4*l**3) * f
    P = 24
    delta = 9
    sigma = (6*P*l) / (b*t**2) / n
    
    if(sigma < sigma_max*0.7):
        print(f"sigma: OK")
        print(f"{sigma} < {sigma_max*0.7}")
    else:
        print(f"sigma NG")
        print(f"{sigma} > {sigma_max*0.7}")
    
    print(f"P: {P}")
    print(f"k: {k}")
    print(f"delta: {delta}")
    print(f"sigma: {sigma}")

def calcback_simple():
    P = 12
    delta = 4
    n = 2
    
    E = 186000 # [N/mm2]
    nyu = 0.3
    sigma_max = 651 # SUS304-CSP 3/4H[N/mm2]
    # sigma_max = 0.7*520 # SUS301, 304[N/mm2]
    rate_sigma = 0.8 # 最大たわみ時の応力
    # I = b*t**3/12
    f = 1.05
    b_list = [6,7,8,9,10,11,12,13,14,15,18,21,25]
    t_list = list(np.arange(0.2, 0.9, 0.1))
    # t_list = list(np.arange(0.2, 1.5, 0.1))
    t_list_b = []
    for b in b_list:
        # t_list_b = t_list
        if(b>=6 and b<=8): t_list_b = t_list[:4]
        elif(b>=9 and b<=13): t_list_b = t_list[1:5]
        elif(b>=14 and b<=18): t_list_b = t_list[3:6]
        elif(b==21 or b==25): t_list_b = t_list[4:]
        for t in t_list_b:
            l_calc = int(((E*delta*n)/(4*P)*b*t**3)**(1/3))
            for l in [l_calc, l_calc+1]:
                sigma = (6*P*l)/(b*t**2) / n
                k = n * (E*b*t**3) / (4*l**3) * f
                delta_designed = (P*l**3) / (3*E*b*t**3/12) / n
                if(sigma < rate_sigma*sigma_max):
                    print(f"k={k:.2f}, b={b:.1f}, t={t:.1f}, l={l}, disp:{delta_designed:.2f} - sigma:{sigma:.1f}")
                    # print(f"displacement: {delta_designed:.2f} [mm]")
                    # print(f"sigma: {sigma:.1f} < {rate_sigma*sigma_max:.1f}")

def calcback_one():
    P = 24
    delta = 9
    E = 186000
    nyu = 0.3
    sigma_max = 651 # SUS304-CSP 3/4H[N/mm2]
    
    b_list = [6,7,8,9,10,11,12,13,14,15,18,21,25]
    t_list = list(np.arange(0.2, 0.9, 0.1))
    q_list = list(range(10, 95, 5))

def calcback_two():
    P = 24
    delta = 9
    E = 186000
    nyu = 0.3
    sigma_max = 651 # SUS304-CSP 3/4H[N/mm2]
    
    b_list = [6,7,8,9,10,11,12,13,14,15,18,21,25]
    t_list = list(np.arange(0.2, 0.9, 0.1))
    q_list = list(range(10, 95, 5))
            

if __name__ == "__main__":
    # calcforward_simple()
    # calcback_simple()
    x = [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]
    y = [399.47, 331.09, 266.51, 210.82, 165.58, 130.11, 102.8, 81.9, 65.88, 53.54, 43.94]
    plt.figure()
    plt.plot(x, y, marker="o")
    plt.show()