function [path_installed] = downloadOpenSimADLibraries(path_install, version, overwrite)
% --------------------------------------------------------------------------
% downloadOpenSimADLibraries
%   Download precompiled binaries with the OpenSimAD libraries 
%
% INPUT:
%   - path_install -
%   * Where to put them
%
%   - version -
%   * Version tag of the release
%
% 
% Original author: Lars D'Hondt
% Original date: 07/November/2025
% --------------------------------------------------------------------------

arguments
    path_install = './opensimAD-install';
    version = 'v0.1.0';
    overwrite = false;
end

url_repo = 'https://github.com/Lars-DHondt-KUL/opensimAD-core/releases/download/';


if ispc
    filename = 'opensimad-install-windows.zip';
    dirname = ['windows-',version];
    path_installed = fullfile(path_install,dirname);
    if ~overwrite && isfolder(fullfile(path_installed,'bin')) && ...
            isfolder(fullfile(path_installed,'sdk'))
        return
    end
    url = [url_repo, version, '/', filename];
    zipfilename = websave(filename, url);
    
    unzip(zipfilename, path_installed);
    delete(zipfilename);

elseif ismac
    error("No binaries available")

elseif isunix
    filename = 'opensimad-install-linux.tar';
    dirname = ['linux-',version];
    path_installed = fullfile(path_install,dirname);
    if ~overwrite && isfolder(fullfile(path_installed,'lib')) && ...
            isfolder(fullfile(path_installed,'include'))
        return
    end
    url = [url_repo, version, '/', filename];
    zipfilename = websave(filename, url);
    untar(zipfilename, path_installed);
    delete(zipfilename);

end




end % end of function
