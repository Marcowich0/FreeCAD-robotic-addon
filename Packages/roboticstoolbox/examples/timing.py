#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 10 14:22:36 2020

@author: corkep
"""


if __name__ == '__main__':

    import timeit



#     setup = '''
# import roboticstoolbox as rtb
# import numpy as np
# puma = rtb.models.DH.Puma560()

# q = puma.qz
# qd = puma.qz
# qdd = puma.qz
# ones = np.ones((6,))
# qd = ones
# qdd = ones
# '''
#     # N = 1000000
#     # t = timeit.timeit(stmt='puma.rne(q, qd, qdd)', setup=setup, number=N)
#     # print(f"rne(numeric):        {t/N*1e6:.3g} μs")

#     # N = 10000
#     # t = timeit.timeit(stmt='puma.rne_python(q, qd, qdd, debug=False)', setup=setup, number=N)
#     # print(f"rne_python(numeric):     {t/N*1e6:.3g} μs")

#     sym_setup = '''
# import roboticstoolbox as rtb
# from spatialmath.base import sym
# import numpy as np
# puma = rtb.models.DH.Puma560(symbolic=True)

# q = sym.symbol("q_:6")
# qd = sym.symbol("qd_:6")
# qdd = sym.symbol("qdd_:6")
# '''

#     # N = 100
#     t = timeit.timeit(stmt='puma.rne_python(q, qd, qdd, debug=False)', setup=sym_setup, number=N)
#     print(f"rne_python(symbolic):     {t/N*1e6:.3g} μs")



    setup = '''
from roboticstoolbox import ETS
'''
    N = 1000000
    t = timeit.timeit(stmt='ETS.tx(0.3)', setup=setup, number=N)
    print(f"ETS + trotx:             {t/N*1e6:.3g} μs")

    N = 1000000
    t = timeit.timeit(stmt='ETS.ty(0.3)', setup=setup, number=N)
    print(f"ETS + axis_func):        {t/N*1e6:.3g} μs")