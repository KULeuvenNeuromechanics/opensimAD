function [] = generateFunction(nInputs, fooPath, pathOutputFile,...
    createSerialisedFunction, createSharedLibrary, secondOrderDerivatives)
% --------------------------------------------------------------------------
% generateFunction
%   Generates an expression graph of the function and its derivative
%   (foo_jac.c) based on the expression graph (foo.py).
%
%
% INPUT:
%   - nInputs -
%   * number of input arguments for the external function [double]
%
%   - fooPath -
%   * path to foo.py [char]
%
%   - secondOrderDerivatives -
%   * do you want to calculate 2nd derivatives of external function outputs 
%   w.r.t. inputs? [bool]
%
%   - generateCSource -
%   * generate source code (c), otherwise
%
% OUTPUT:
%   - (This function does not return output arguments) -
%
% Reference: 
%   Falisse A, Serrancolí G, et al. (2019) Algorithmic differentiation 
%   improves the computational efficiency of OpenSim-based trajectory 
%   optimization of human movement. PLoS ONE 14(10): e0217730. 
%   https://doi.org/10.1371/journal.pone.0217730
%
% Original author: Lars D'Hondt (based on code by Antoine Falisse)
% Original date: 8/May/2023
% --------------------------------------------------------------------------


[fooDir,fooName,~] = fileparts(fooPath);
workdir = pwd;
cd(fooDir)
foo = str2func(fooName);
cd(workdir)

import casadi.*

arg = SX.sym('arg', nInputs);
[y, ~, ~] = foo(arg);
F = Function('F', {arg}, {y});

if createSerialisedFunction
    F.save([pathOutputFile,'.casadi']);
end

if createSharedLibrary
    % Generate source code for shared library
    cg = CodeGenerator(fullfile(fooDir,[fooName,'.c']));
    cg.add(F);
    cg.add(F.jacobian());

    if secondOrderDerivatives
    % Include functions to evaluate forward, reverse, and 
    % forward-over-reverse to use an exact Hessian.
        Fr = F.reverse(1);
        cg.add(Fr);
        for i=0:6
            cg.add(F.forward(2^i));
            cg.add(Fr.forward(2^i));
        end
    end
    cg.generate();
end


end
