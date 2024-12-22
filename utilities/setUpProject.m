function setUpProject()
%setUpProject  Configure the environment for this project
%
%   Set up the environment for the current project. This function is set to
%   Run at Startup.

%   Copyright 2013-2015 The MathWorks, Inc.

% Use Simulink Project API to get the current project:
p = slproject.getCurrentProject;

projectRoot = p.RootFolder;
% Set the location of slprj to be the "work" folder of the current project:
myCacheFolder = fullfile(projectRoot, 'work');
if ~exist(myCacheFolder, 'dir')
    mkdir(myCacheFolder)
end
Simulink.fileGenControl('set', 'CacheFolder', myCacheFolder, ...
   'CodeGenFolder', myCacheFolder);

temp = fullfile(projectRoot, 'libraries');
addpath(temp)
temp = fullfile(projectRoot, 'libraries\minSnapTrajec_matlab-master');
addpath(temp)
temp = fullfile(projectRoot, 'controller');
addpath(temp)
temp = fullfile(projectRoot, 'nonlinearAirframe');
addpath(temp)
temp = fullfile(projectRoot, 'Data');
addpath(temp)
temp = fullfile(projectRoot, 'mainModels');
addpath(temp)
temp = fullfile(projectRoot, 'utilities');
addpath(temp)
temp = fullfile(projectRoot, 'tasks');
addpath(temp)