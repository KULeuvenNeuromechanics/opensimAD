function [] = generateF(nInputs, fooPath, secondOrderDerivatives)
% --------------------------------------------------------------------------
% generateF
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
%
% Last edit by: 
% Last edit date: 
% --------------------------------------------------------------------------

% The bundled executable handles Python/CasADi without a MATLAB Python setup.
oldDir = pwd;
restoreDir = onCleanup(@() cd(oldDir));
[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
cd(fullfile(pathUtilities, 'GenF'));
command = ['GenF.exe "' fooPath '" ' num2str(nInputs) ' ' num2str(secondOrderDerivatives)];
runBuildCommand(command, 'CasADi code generation', true);
requireBuildFile(fullfile(fooPath, 'foo_jac.c'), 'CasADi code generation');
requireBuildFile(fullfile(fooPath, 'F_foo.casadi'), 'CasADi code generation');
end
