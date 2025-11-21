% script_test_fcn_Greedy_greedyPlanner

% a basic test of the greedy planner algorithm implementation

% REVISION HISTORY:
%
% As: script_test_fcn_algorithm_greedy_planner
% 
% 2025_07_08 - K. Hayes, kxh1031@psu.edu
% - Replaced fcn_general_calculation_euclidean_point_to_point_distance
%    with vector sum method 
%
% As: script_test_fcn_BoundedAStar_greedyPlanner
% 
% 2025_08_07 - K. Hayes
% - Copied script_test_fcn_algorithm_greedy_planner into new script file
%   % to follow library naming conventions
% 
% 2025_11_02 by Sean Brennan, sbrennan@psu.edu
% - Changed fcn_BoundedAStar_polytopesGenerateAllPtsTable 
%   % to fcn_Visibility_polytopesGenerateAllPtsTable
%   % WARNING: inputs/outputs to this changed slightly. Function needs to 
%   % be rechecked
% 
% 2025_11_14 by Sean Brennan, sbrennan@psu.edu
% - Changed fcn_Visibility_clearAndBlockedPointsGlobal 
%   % to fcn_VGraph_clearAndBlockedPointsGlobal
% - Changed fcn_Visibility_polytopesGenerateAllPtsTable 
%   % to fcn_VGraph_polytopesGenerateAllPtsTable
%
% As: script_test_fcn_Greedy_greedyPlanner
% 
% 2025_11_14 by Sean Brennan, sbrennan@psu.edu
% - Moved script into the Greedy repo
%
% 2025_11_17 by Sean Brennan, sbrennan@psu.edu
% - Moved script into the Greedy repo

% TO-DO
% 2025_11_21 by Sean Brennan, sbrennan@psu.edu
% -  (add to do here)


%% Set up the workspace
close all

%% Code demos start here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   _____                              ____   __    _____          _
%  |  __ \                            / __ \ / _|  / ____|        | |
%  | |  | | ___ _ __ ___   ___  ___  | |  | | |_  | |     ___   __| | ___
%  | |  | |/ _ \ '_ ` _ \ / _ \/ __| | |  | |  _| | |    / _ \ / _` |/ _ \
%  | |__| |  __/ | | | | | (_) \__ \ | |__| | |   | |___| (_) | (_| |  __/
%  |_____/ \___|_| |_| |_|\___/|___/  \____/|_|    \_____\___/ \__,_|\___|
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Demos%20Of%20Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 1

close all;
fprintf(1,'Figure: 1XXXXXX: DEMO cases\n');

%% DEMO case: plan path through field with greedy planner
figNum = 10001;
titleString = sprintf('DEMO case: plan path through field with greedy planner');
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


%% Test cases start here. These are very simple, usually trivial
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  _______ ______  _____ _______ _____
% |__   __|  ____|/ ____|__   __/ ____|
%    | |  | |__  | (___    | | | (___
%    | |  |  __|  \___ \   | |  \___ \
%    | |  | |____ ____) |  | |  ____) |
%    |_|  |______|_____/   |_| |_____/
%
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=TESTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 2

close all;
fprintf(1,'Figure: 2XXXXXX: TEST mode cases\n');

%% TEST case: zero gap between polytopes
figNum = 20001;
titleString = sprintf('TEST case: zero gap between polytopes');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

%% Fast Mode Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ______        _     __  __           _        _______        _
% |  ____|      | |   |  \/  |         | |      |__   __|      | |
% | |__ __ _ ___| |_  | \  / | ___   __| | ___     | | ___  ___| |_ ___
% |  __/ _` / __| __| | |\/| |/ _ \ / _` |/ _ \    | |/ _ \/ __| __/ __|
% | | | (_| \__ \ |_  | |  | | (_) | (_| |  __/    | |  __/\__ \ |_\__ \
% |_|  \__,_|___/\__| |_|  |_|\___/ \__,_|\___|    |_|\___||___/\__|___/
%
%
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Fast%20Mode%20Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 8

close all;
fprintf(1,'Figure: 8XXXXXX: FAST mode cases\n');

%% Basic example - NO FIGURE
figNum = 80001;
fprintf(1,'Figure: %.0f: FAST mode, empty figNum\n',figNum);
figure(figNum); close(figNum);

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

% Fill in cGraph
cGraph_heuristic = fcn_VGraph_costCalculate(vGraph, pointsWithData, 'distance from finish', (-1));
cGraph_movement  = fcn_VGraph_costCalculate(vGraph, pointsWithData, 'movement distance', (-1));
cGraph = cGraph_movement + cGraph_heuristic;

% Plan path through field using greedy planner
[cost, route] = fcn_Greedy_greedyPlanner(cGraph, pointsWithData, ([]));

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


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

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

% Fill in cGraph
cGraph_heuristic = fcn_VGraph_costCalculate(vGraph, pointsWithData, 'distance from finish', (-1));
cGraph_movement  = fcn_VGraph_costCalculate(vGraph, pointsWithData, 'movement distance', (-1));
cGraph = cGraph_movement + cGraph_heuristic;

% Plan path through field using greedy planner
[cost, route] = fcn_Greedy_greedyPlanner(cGraph, pointsWithData, (-1));

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

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

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

% Fill in cGraph
cGraph_heuristic = fcn_VGraph_costCalculate(vGraph, pointsWithData, 'distance from finish', (-1));
cGraph_movement  = fcn_VGraph_costCalculate(vGraph, pointsWithData, 'movement distance', (-1));
cGraph = cGraph_movement + cGraph_heuristic;


Niterations = 10;

% Do calculation without pre-calculation
tic;
for ith_test = 1:Niterations
    % Call the function
    [cost, route] = fcn_Greedy_greedyPlanner(cGraph, pointsWithData, ([]));
end
slow_method = toc;

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call the function
    [cost, route] = fcn_Greedy_greedyPlanner(cGraph, pointsWithData, (-1));
end
fast_method = toc;

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));

% Plot results as bar chart
figure(373737);
clf;
hold on;

X = categorical({'Normal mode','Fast mode'});
X = reordercats(X,{'Normal mode','Fast mode'}); % Forces bars to appear in this exact order, not alphabetized
Y = [slow_method fast_method ]*1000/Niterations;
bar(X,Y)
ylabel('Execution time (Milliseconds)')


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));

%% BUG cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ____  _    _  _____
% |  _ \| |  | |/ ____|
% | |_) | |  | | |  __    ___ __ _ ___  ___  ___
% |  _ <| |  | | | |_ |  / __/ _` / __|/ _ \/ __|
% | |_) | |__| | |__| | | (_| (_| \__ \  __/\__ \
% |____/ \____/ \_____|  \___\__,_|___/\___||___/
%
% See: http://patorjk.com/software/taag/#p=display&v=0&f=Big&t=BUG%20cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All bug case figures start with the number 9

% close all;

%% BUG

%% Fail conditions
if 1==0

end


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