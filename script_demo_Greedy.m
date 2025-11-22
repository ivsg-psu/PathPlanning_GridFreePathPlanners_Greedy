%% script_demo_Greedy
% main demo file for the Greedy path planner

%% Introduction to and Purpose of the Code
% This is the explanation of the code that can be found by running
%       script_demo_Greedy.m
% This is a script to demonstrate the functions within the Greedy code
% library. This code repo is typically located at:
%   https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_Greedy
%
% If you have questions or comments, please contact Sean Brennan at
% sbrennan@psu.edu
%
% The purpose of the code is to demonstrate an implementation of the Greedy
% path planner

% REVISION HISTORY:
% 
% 2025_11_15 by Sean Brennan, sbrennan@psu.edu
% - Created this demo script of core functions
% 
% 2025_11_21 by Sean Brennan, sbrennan@psu.edu
% - Updating rev lists
% - confirmed greedyPlanner works at basic level
% (new release)
%
% 2025_11_22 by Sean Brennan, sbrennan@psu.edu
% - In script_test_fcn_Greedy_greedyPlanner
%   % * Added non-convex test case
%   % * Shows that the Greedy algorithm works and that old version fails
% (new release)

% TO-DO:
% 
% 2025_11_21 by Sean Brennan, sbrennan@psu.edu
% -  (add to do here)


%% Make sure we are running out of root directory
st = dbstack; 
thisFile = which(st(1).file);
[filepath,name,ext] = fileparts(thisFile);
cd(filepath);

%%% START OF STANDARD INSTALLER CODE %%%%%%%%%

%% Clear paths and folders, if needed
if 1==1
    clear flag_Greedy_Folders_Initialized
end
if 1==0
    fcn_INTERNAL_clearUtilitiesFromPathAndFolders;
end
if 1==0
    restoredefaultpath
end

%% Install dependencies
% Define a universal resource locator (URL) pointing to the repos of
% dependencies to install. Note that DebugTools is always installed
% automatically, first, even if not listed:
clear dependencyURLs dependencySubfolders
ith_repo = 0;

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_MapTools_MapGenClassLibrary';
dependencySubfolders{ith_repo} = {'Functions','testFixtures','GridMapGen'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_VGraph';
dependencySubfolders{ith_repo} = {'Functions','Data'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary';
dependencySubfolders{ith_repo} = {'Functions','Data'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_PathTools_GetUserInputPath';
dependencySubfolders{ith_repo} = {''};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/FieldDataCollection_VisualizingFieldData_PlotRoad';
dependencySubfolders{ith_repo} = {'Functions','Data'};

% ith_repo = ith_repo+1;
% dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_GeomClassLibrary';
% dependencySubfolders{ith_repo} = {'Functions','Data'};

% ith_repo = ith_repo+1;
% dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_MapT ools_MapGenClassLibrary';
% dependencySubfolders{ith_repo} = {'Functions','testFixtures','GridMapGen'};



%% Do we need to set up the work space?
if ~exist('flag_Greedy_Folders_Initialized','var')
    
    % Clear prior global variable flags
    clear global FLAG_*

    % Navigate to the Installer directory
    currentFolder = pwd;
    cd('Installer');
    % Create a function handle
    func_handle = @fcn_DebugTools_autoInstallRepos;

    % Return to the original directory
    cd(currentFolder);

    % Call the function to do the install
    func_handle(dependencyURLs, dependencySubfolders, (0), (-1));

    % Add this function's folders to the path
    this_project_folders = {...
        'Functions','Data'};
    fcn_DebugTools_addSubdirectoriesToPath(pwd,this_project_folders)

    flag_Greedy_Folders_Initialized = 1;
end

%%% END OF STANDARD INSTALLER CODE %%%%%%%%%

%%
folderToDoReplacement = pwd;
fcn_DebugTools_replaceStringInDirectory(folderToDoReplacement, cat(2,'_L','APS_'), '_GREEDY_', ('Greedy'), (-1));

%% Set environment flags for input checking in Laps library
% These are values to set if we want to check inputs or do debugging
setenv('MATLABFLAG_GREEDY_FLAG_CHECK_INPUTS','1');
setenv('MATLABFLAG_GREEDY_FLAG_DO_DEBUG','0');

%% Set environment flags that define the ENU origin
% This sets the "center" of the ENU coordinate system for all plotting
% functions
% Location for Test Track base station
setenv('MATLABFLAG_PLOTROAD_REFERENCE_LATITUDE','40.86368573');
setenv('MATLABFLAG_PLOTROAD_REFERENCE_LONGITUDE','-77.83592832');
setenv('MATLABFLAG_PLOTROAD_REFERENCE_ALTITUDE','344.189');


%% Set environment flags for plotting
% These are values to set if we are forcing image alignment via Lat and Lon
% shifting, when doing geoplot. This is added because the geoplot images
% are very, very slightly off at the test track, which is confusing when
% plotting data
setenv('MATLABFLAG_PLOTROAD_ALIGNMATLABLLAPLOTTINGIMAGES_LAT','-0.0000008');
setenv('MATLABFLAG_PLOTROAD_ALIGNMATLABLLAPLOTTINGIMAGES_LON','0.0000054');

%% Start of Demo Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   _____ _             _            __   _____                          _____          _
%  / ____| |           | |          / _| |  __ \                        / ____|        | |
% | (___ | |_ __ _ _ __| |_    ___ | |_  | |  | | ___ _ __ ___   ___   | |     ___   __| | ___
%  \___ \| __/ _` | '__| __|  / _ \|  _| | |  | |/ _ \ '_ ` _ \ / _ \  | |    / _ \ / _` |/ _ \
%  ____) | || (_| | |  | |_  | (_) | |   | |__| |  __/ | | | | | (_) | | |___| (_) | (_| |  __/
% |_____/ \__\__,_|_|   \__|  \___/|_|   |_____/ \___|_| |_| |_|\___/   \_____\___/ \__,_|\___|
%
%
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Start%20of%20Demo%20Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Welcome to the demo code for the Greedy library!')

%% Core functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                 ______                _   _
%  / ____|               |  ____|              | | (_)
% | |     ___  _ __ ___  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
% | |    / _ \| '__/ _ \ |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
% | |___| (_) | | |  __/ | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  \_____\___/|_|  \___| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Core+Functions&x=none&v=4&h=4&w=80&we=false
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
fprintf(1,'Figure: 1XXXXXX: CORE functions\n');

%% CORE function: fcn_Greedy_greedyPlanner
% This is the first demo case in the script: use the greedy planner to find
% a path through an obstacle field

functionName = 'fcn_Greedy_greedyPlanner';

figNum = 10001;
titleString = sprintf('CORE function: %s',functionName);
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% start and finish (x,y) coordinates
start_xy = [0.0, 0.5];
finish_xy = [1, 0.5];

% generate map
% generate Voronoi tiling from Halton points
fullyTiledPolytopes = fcn_MapGen_generatePolysFromSeedGeneratorNames('haltonset', [1 ,30],[],[],-1);

% remove the edge polytope that extend past the high and low points    
trimmedPolytopes = fcn_MapGen_polytopesDeleteByAABB( fullyTiledPolytopes, [0 0 1 1], (-1));

% shink the polytopes so that they are no longer tiled
% shink the polytopes so that they are no longer fully tiled
des_radius = 0.05; % desired average maximum radius
sigma_radius = 0.002; % desired standard deviation in maximum radii
min_rad = 0.0001; % minimum possible maximum radius for any obstacle
shrink_seed = 1111; % seed used for randomizing the shrinking process
des_cost = 0; % polytope traversal cost

rng(shrink_seed) % set the random number generator with the shrink seed    
polytopes = fcn_MapGen_polytopesShrinkToRadius(trimmedPolytopes,des_radius,sigma_radius,min_rad, -1);
polytopes = fcn_MapGen_polytopesSetCosts(polytopes, des_cost, (-1));

% info needed for further work
% gather data on all the points
[pointsWithData, startPointData, finishPointData] = fcn_VGraph_polytopesGenerateAllPtsTable(polytopes,start_xy,finish_xy,-1);

% Generate visibility graph
% finishes = [all_pts; start; finish];
% starts = [all_pts; start; finish];

% Fill in vGraph
vGraph = fcn_VGraph_clearAndBlockedPointsGlobal(polytopes, pointsWithData, pointsWithData, [], -1);

% Plan path through field using OLD greedy planner
[cost_OLD, route_OLD] = fcn_Greedy_greedyPlanner_OLD(vGraph, pointsWithData, startPointData, finishPointData, (polytopes), (figNum*100));


% Fill in cGraph
cGraph_heuristic = fcn_VGraph_costCalculate(vGraph, pointsWithData, 'distance from finish', (-1));
cGraph_movement  = fcn_VGraph_costCalculate(vGraph, pointsWithData, 'movement distance', (-1));
cGraph = cGraph_movement + cGraph_heuristic;

% Plan path through field using greedy planner
[cost, route] = fcn_Greedy_greedyPlanner(cGraph, pointsWithData, (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(cost));
assert(isnumeric(route));

% Check variable sizes
assert(size(cost,1)==1); 
assert(size(cost,2)==1); 
assert(size(route,1)>=2); 
assert(size(route,2)==2); 

% Check variable values
assert(isequal(route(1,:), startPointData(1,1:2)));
assert(isequal(route(end,:), finishPointData(1,1:2)));

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

% Save results
fullPathFileName = fullfile(pwd,'Images',cat(2,functionName,'.png'));
saveas(gcf, fullPathFileName);
fullPathFileName = fullfile(pwd,'Images',cat(2,functionName,'.fig'));
saveas(gcf, fullPathFileName);

%% Helper functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  _    _      _                   ______                _   _
% | |  | |    | |                 |  ____|              | | (_)
% | |__| | ___| |_ __   ___ _ __  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
% |  __  |/ _ \ | '_ \ / _ \ '__| |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
% | |  | |  __/ | |_) |  __/ |    | |  | |_| | | | | (__| |_| | (_) | | | \__ \
% |_|  |_|\___|_| .__/ \___|_|    |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%               | |
%               |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Helper+Functions&x=none&v=4&h=4&w=80&we=false
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
fprintf(1,'Figure: 2XXXXXX: HELPER functions\n');

%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

%% function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
% Clear out the variables
clear global flag* FLAG*
clear flag*
clear path

% Clear out any path directories under Utilities
path_dirs = regexp(path,'[;]','split');
utilities_dir = fullfile(pwd,filesep,'Utilities');
for ith_dir = 1:length(path_dirs)
    utility_flag = strfind(path_dirs{ith_dir},utilities_dir);
    if ~isempty(utility_flag)
        rmpath(path_dirs{ith_dir});
    end
end

% Delete the Utilities folder, to be extra clean!
if  exist(utilities_dir,'dir')
    [status,message,message_ID] = rmdir(utilities_dir,'s');
    if 0==status
        error('Unable remove directory: %s \nReason message: %s \nand message_ID: %s\n',utilities_dir, message,message_ID);
    end
end

end % Ends fcn_INTERNAL_clearUtilitiesFromPathAndFolders

