function [] = removeAllTempFiles(varargin)
% --------------------------------------------------------------------------
% removeAllTempFiles
%   Removes all temporary files that are created when running opensimAD.
%   These should be removed automatically if all goes well, but files can
%   persist if an error occurred.
%
%
% INPUT:
%   - (optional) outputFilename -
%   * remove only the temporary files from a specific build
%
%
% OUTPUT:
%   - (This function does not return output arguments) -
%
% Original author: Lars D'Hondt
% Original date: 15/May/2023
% --------------------------------------------------------------------------

[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathUtilities);


temp_dirs = ["intermediateFiles/AD-Recorder-source",...
    "intermediateFiles/AD-Recorder-build"...
    "intermediateFiles/AD-Function-source",...
    "intermediateFiles/InverseDynamics"];

for temp_dir=temp_dirs
    dir1 = dir(fullfile(pathMain, char(temp_dir)));

    for i=1:length(dir1)
        if ~strcmp(dir1(i).name,'.') && ~strcmp(dir1(i).name,'..') && dir1(i).isdir
            if ~isempty(varargin) && ~strcmp(dir1(i).name, varargin{1})
                continue
            end
            rmdir(fullfile(dir1(i).folder,dir1(i).name), 's');
        end
    end

end

end
