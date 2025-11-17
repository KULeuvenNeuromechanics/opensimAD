function [] = generateFunction(nInputs, fooPath, pathOutputFile)
% --------------------------------------------------------------------------
% generateFunction
%   Generates an expression graph of the function and its derivative
%   (foo_jac.c) based on the expression graph (foo.py).
%
%
% INPUT:
%   - nInputs -
%   * number of input arguments for the AD function [double]
%
%   - fooPath -
%   * path to foo.m [char]
%
%   - pathOutputFile -
%   * path where the generated file should be created, without file
%   extension. [char]
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

F.save([pathOutputFile,'.casadi']);


end
