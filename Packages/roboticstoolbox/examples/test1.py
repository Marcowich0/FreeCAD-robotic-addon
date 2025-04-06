from roboticstoolbox import *

puma = models.DH.Puma560()

q = jtraj(puma.qr, puma.qz, 50)
puma.plot(q.q, movie='bob.gif', backend='pyplot')