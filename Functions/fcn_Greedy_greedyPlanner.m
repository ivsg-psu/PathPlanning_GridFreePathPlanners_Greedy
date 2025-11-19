function [cost, route] = fcn_Greedy_greedyPlanner(cGraph, pointsWithData, varargin)
% fcn_Greedy_greedyPlanner
%
% uses 'greedy' planner methods to plan a path through an environment
%
% FORMAT:
% [cost, route] = fcn_Greedy_greedyPlanner(cGraph, pointsWithData, (figNum))
%
%
% INPUTS:
%
%     cGraph: the cost graph as an nxn matrix where n is the number of
%     points (nodes) in the map. This is the same size as the visibility
%     graph, but instead of 1 values indicating visibility, a cost value
%     is given for each element. For static maps, the visibility graph
%     will not change, but costs may change depending on user-defined cost
%     criteria.
%
%     pointsWithData: n-by-5 matrix of all the possible to/from points for
%     visibility calculations including the vertex points on each obstacle,
%     and if the user specifies, the start and/or end points. If the
%     start/end points are omitted, the value of p is the same as the
%     number of points within the polytope field, numPolytopeVertices.
%     Otherwise, p is 1 or 2 larger depending on whether start/end is
%     given. The information in the 5 columns is as follows:
%         x-coordinate
%         y-coordinate
%         point id number
%         obstacle id number (-1 for start/end points)
%         beginning/ending indication (1 if the point is a beginning or
%         start point, 2 if ending point or finish point, and 0 otherwise)
%         Ex: [x y point_id obs_id beg_end]
%
%   (optional inputs)
%
%   figNum: a figure number to plot results. If set to -1, skips any
%   input checking or debugging, no figures will be generated, and sets
%   up code to maximize speed. As well, if given, this forces the
%   variable types to be displayed as output and as well makes the input
%   check process verbose
%
% OUTPUTS:
%
%   cost: the total cost of the route planned, where cost is equivalent to
%   distance
%
%   route: the list of points in the planned route
%
% DEPENDENCIES:
%
% none
%
% EXAMPLES:
%
% See the script: script_test_fcn_Greedy_greedyPlanner
% for a full test suite.
%
% This function was written in January 2024 by Steve Harnett
% Questions or comments? contact sjharnett@psu.edu


% Revision history:
% As: fcn_algorithm_greedy_planner
% January 2024 by Steve Harnett
% -- first write of function
% February 2024 by Steve Harnett
% -- function updated to make a right left distinction using cross products
%
% As: fcn_BoundedAStar_greedyPlanner
% 2025_07_17 by K. Hayes, kxh1031@psu.edu
% -- function copied to new script from
%    fcn_algorithm_greedy_planner.m to follow library
%    conventions
%
% 2025_08_06 - K. Hayes
% -- updated fcn header and formatting
%
% 2025_08_18 - K. Hayes
% -- added debug plotting capabilities
%
% As: fcn_Greedy_greedyPlanner
% 2025_11_14 by S. Brennan, sbrennan@psu.edu
% - copied code into Greedy repo and changed name
%   % * From: fcn_BoundedAStar_greedyPlanner
%   % * To: fcn_Greedy_greedyPlanner
%
% 2025_11_16 to 2025_11_19 by S. Brennan, sbrennan@psu.edu
% - complete rewrite for clarity


% TO-DO:
% -- fill in to-do items here.

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the figNum variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 3; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS");
    MATLABFLAG_MAPGEN_FLAG_DO_DEBUG = getenv("MATLABFLAG_MAPGEN_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS);
    end
end

flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_figNum = 999978; 
else
    debug_figNum = []; 
end

%% check input arguments?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if 0==flag_max_speed
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(2,MAX_NARGIN);

        % Check the pointsWithData input, make sure it has 5 columns
        fcn_DebugTools_checkInputsToFunctions(...
            pointsWithData, '5column_of_numbers');

    end
end

% % Does user want to specify the polytopes input?
% polytopes = []; % Default is empty
% if 5 <= nargin
%     temp = varargin{1};
%     if ~isempty(temp)
%         polytopes = temp;
%     end
% end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin)
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        figNum = temp;
        flag_do_plots = 1;
    end
end


%% Main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%See: http://patorjk.com/software/taag/#p=display&f=Big&t=Main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§


% Plot the polytopes
if flag_do_debug
    figure(debug_figNum);
    clf;
    hold on;

    legend('Interpreter','none','Location','best');

    % axes_limits = [0 1 0 1]; % x and y axes limits
    % axis_style = 'square'; % plot axes style
    plotFormat.Color = 'Blue'; % edge line plotting
    plotFormat.LineStyle = '-';
    plotFormat.LineWidth = 2; % linewidth of the edge
    fillFormat = [1 0 0 1 0.4];

    polytopes = fcn_VGraph_helperFillPolytopesFromPointData(pointsWithData,-1);

    % FORMAT: fcn_MapGen_plotPolytopes(polytopes,figNum,line_spec,line_width,axes_limits,axis_style);
    h_polytopes = fcn_MapGen_plotPolytopes(polytopes,(plotFormat),(fillFormat),(debug_figNum));
    set(h_polytopes,'DisplayName','polytopes')

    box on
    axis([-0.1 1.1 -0.1 1.1]);
    xlabel('x [m]');
    ylabel('y [m]');

    h_openSet    = plot(nan, nan,'r.','MarkerSize',30,'DisplayName','Query point');
    h_q          = plot(nan, nan,'go','MarkerSize',10,'DisplayName','Current q');
    h_allPoints  = plot(pointsWithData(:,1), pointsWithData(:,2),'.','MarkerSize',10,'Color',0.8*[1 1 1], 'DisplayName','All points');
    h_successors = plot(nan, nan,'b.','MarkerSize',30,'DisplayName','Successors to open set');
    legend('Interpreter','none','Location','best');

    % label point ids for debugging. The last two points are start and
    % finish, so do not need to be plotted and labeled.
    addNudge = 0.01;
    text(pointsWithData(:,1)+addNudge,pointsWithData(:,2)+addNudge,string(pointsWithData(:,3)));


end

numNodes = size(cGraph,1); % number of nodes in the cost graph

% Grab start and end point, which are the 2nd to last and last point in the
% pointsWithData list
startPointData  = pointsWithData(end-1,:);
finishPointData = pointsWithData(end,:);
startPointIndex  = startPointData(1,3);
finishPointIndex = finishPointData(1,3);

% Make sure cost graph does not have self-visibility, e.g. the diagonals
% have infinite cost
cGraphModified = cGraph;
diagonalMatrix = eye(numNodes);
cGraphModified(diagonalMatrix==1) = inf;

% Initialize the open list: put the starting node on the open list
% Open set is the set of all nodes that have been visited thus far. This
% starts out as NaN values except for the start point.
openSet = nan(numNodes,1);
openSet(startPointIndex) = startPointIndex; % only store the ID not the whole point
openSetCosts = inf(numNodes,1);
openSetCosts(startPointIndex,1) = 0; % Force the search to start at startPointIndex

% 2.  Initialize the closed list. This is the list of all the points
% already evaluated. This is updated AFTER the connections for the current
% open set index have been exhausted.
closed_set = nan(1,numNodes);

% Initialize route outputs
route = [];
qHistory = nan(numNodes,5);
openHistory = cell(numNodes,1);
openParents = cell(numNodes,1);


% 3.  Loop through connection depth. For a vGraph with N points, the path
% will never be longer than N-1 points since this would connect EVERY point
% together as part of the path
for ith_depth = 1:numNodes

    % Find which of the openSet have been visited
    openIndices = find(~isnan(openSet));
    openHistory{ith_depth,1} = openIndices;


    % Update the plot
    if flag_do_debug
        figure(debug_figNum);
        set(h_openSet,'Xdata',pointsWithData(openIndices,1),'Ydata',pointsWithData(openIndices,2));
    end


    %%%%%%%%%%%%%%%
    % a) find the openSet node with the least cost. Call it
    % "q". This is the "greedy" part of the algorithm - always choosing
    % minimum cost.
    [~, idx_of_q] = min(openSetCosts);

    qWithData = pointsWithData(idx_of_q,:);

    % Update the plot
    if flag_do_debug
        figure(debug_figNum);
        set(h_q,'Xdata',pointsWithData(idx_of_q,1),'Ydata',pointsWithData(idx_of_q,2));
    end

    qHistory(ith_depth,:) = qWithData;

    %%%%%%%%%%%%%%%
    % b) pop q off the open list by setting cost to go to q to infinity
    openSet(idx_of_q) = NaN;
    % Set costs for those that were explored to infinity. These are the
    % "to" costs, so that no future searches will choose to return to
    % indices already visited
    cGraphModified(:,idx_of_q) = inf;
    % cGraphModified(idx_of_q,:) = inf;

    %%%%%%%%%%%%%%%
    % c) use the cost graph to generate open set successors
    % qs_costRow = cGraphModified(idx_of_q,:);
    % successorIndices = find(~isinf(qs_costRow));
    % qParents{end+1} = idx_of_q; %#ok<AGROW>

    % The cost for the current open set selection are the "rows" of the cost
    % matrix. These represent the costs to leave all visited nodes
    nextConnectionCosts = cGraphModified(openIndices,:); 

    % Keep only those that are not infinite
    indicesToSearch = find(~isinf(nextConnectionCosts));
    [fromIndex, toIndex] = find(~isinf(nextConnectionCosts)); 
    actualFromIndex = openIndices(fromIndex);

    openParents{ith_depth,1} = [actualFromIndex, toIndex];

    % Update the openSet info
    openSet(indicesToSearch) = indicesToSearch;
    openSetCosts(indicesToSearch) = nextConnectionCosts(indicesToSearch);

    % Update the plot
    if flag_do_debug
        figure(debug_figNum);
        set(h_successors,'Xdata',pointsWithData(indicesToSearch,1),'Ydata',pointsWithData(indicesToSearch,2));
    end



    % Check exit conditions
    if all(isnan(openSet))
        flagFinishWasFound = 0;
        break
    end
    if any(openSet==finishPointIndex)
        flagFinishWasFound = 1;
    end
end % Ends while loop

if flagFinishWasFound
    disp('Found finish');
end
%
%
% % d) for each successor
% for i = 1:length(successorIndices)
%     successor = pointsWithData(successorIndices(i),:);
%
%     % i) if successor is the goal, stop search
%     if successor(3) == finishPointData(3)
%         % Could move this out of the for-loop to post-calculate
%         %%%%%%%%
%         cost = open_set_gs(idx_of_q) + heuristicCost(idx_of_q);
%         parent = idx_of_q;
%         route = [qWithData; finishPointData];
%         qHistory(end,:) = [];
%         possible_parents = intersect(openParents{end}, qHistory(:,3));
%
%         while parent ~= startPointData(3)
%             parent_gs = open_set_gs(possible_parents);
%             [~, idx_of_parent] = min(parent_gs);
%             parent = possible_parents(idx_of_parent);
%             parent_position_in_history = find(parent == qHistory(:,3));
%             parent_point = qHistory(parent_position_in_history,:);
%             route = [parent_point;route]; %#ok<AGROW>
%             qHistory(parent_position_in_history:end,:) = [];
%             % parent = possible_parents(idx_of_parent);
%             % parent_position_in_history = find(parent == q_history(:,3));
%             possible_parents = intersect(openParents{parent_position_in_history}, qHistory(:,3));
%         end
%         route = [startPointData; route]; %#ok<AGROW>
%         %%%%%%%%
%         break
%
%     else
%         % ii) else, compute both g and h for successor
%         successor_g = open_set_gs(idx_of_q) + sqrt((successor(1) - qWithData(1)).^2 + ((successor(2) - qWithData(2)).^2));
%         successor_h = sqrt((successor(1) - finishPointData(1)).^2 + ((successor(2) - finishPointData(2)).^2));
%         successor_f = successor_g + successor_h;
%         openSet(successor(3)) = successor(3);
%         open_set_gs(successor(3)) = successor_g;
%         nextConnectionCosts(successor(3)) = successor_f;
%
%     end
%
%
%     % e) push q on the closed list
%     closed_set(idx_of_q) = idx_of_q;
%
% end
%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_do_plots
    figure(figNum)
    hold on

    % Plot polytopes
    plotFormat.Color = 'blue';
    plotFormat.LineWidth = 2;
    h = fcn_MapGen_plotPolytopes(polytopes,plotFormat,[1 0 0 0 0.5],figNum);
    set(h, 'HandleVisibility', 'off');

    % Plot path through field
    plot(route(:,1),route(:,2),'k-','linewidth',2, 'DisplayName', 'Route')
    plot(startPointData(1), startPointData(2), 'gx','linewidth',2, 'DisplayName', 'Start')
    plot(finishPointData(1), finishPointData(2), 'rx','linewidth',2, 'DisplayName', 'Finish')

    % Plot neighboring points
    % plot(appex_x,appex_y,'o','linewidth',2)

end


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