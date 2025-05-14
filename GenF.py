
import casadi as ca
import importlib
import sys
import os

PathFoo = sys.argv[1]
dim = int(sys.argv[2])
secondOrderDerivatives = bool(int(sys.argv[3]))

os.chdir(PathFoo)
sys.path.append(PathFoo)
import foo
importlib.reload(foo)

cg = ca.CodeGenerator('foo_jac')
arg = ca.SX.sym('arg', dim)
y, _, _ = foo.foo(arg)
F = ca.Function('F', [arg], [y])
cg.add(F)
cg.add(F.jacobian())

F.save("F_foo.casadi")

if secondOrderDerivatives:
    # Generate also forward, reverse, and forward-over-reverse to use an exact Hessian
    Fr = F.reverse(1)
    cg.add(Fr)
    for i in range(7):
    # 2^0 to 2^6
        cg.add(F.forward(2**i))
        cg.add(Fr.forward(2**i))

cg.generate()
