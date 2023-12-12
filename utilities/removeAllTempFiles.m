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


temp_dirs = ["buildExpressionGraph", "buildExternalFunction",...
    "installExternalFunction"];

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

lockFile = fullfile(pathMain, 'OpenSimAD-install', 'bin', 'lockFile.txt');
if isfile(lockFile)
    delete(lockFile)
end

path_bin_foo = fullfile(pathMain, 'OpenSimAD-install', 'bin', 'foo.py');
if isfile(path_bin_foo)
    delete(path_bin_foo)
end

end