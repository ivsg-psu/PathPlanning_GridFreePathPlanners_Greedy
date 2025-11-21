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


% REVISION HISTORY:
%
% As: fcn_algorithm_greedy_planner
%
% January 2024 by Steve Harnett
% - First write of function
%
% February 2024 by Steve Harnett
% - Function updated to make a right left distinction using cross products
%
% As: fcn_BoundedAStar_greedyPlanner
%
% 2025_07_17 by K. Hayes, kxh1031@psu.edu
% - Function copied to new script from
%   % fcn_algorithm_greedy_planner.m to follow library
%   % conventions
%
% 2025_08_06 - K. Hayes
% - Updated fcn header and formatting
%
% 2025_08_18 - K. Hayes
% - Added debug plotting capabilities
%
% As: fcn_Greedy_greedyPlanner
%
% 2025_11_14 by Sean Brennan, sbrennan@psu.edu
% - Copied code into Greedy repo and changed name
%   % * From: fcn_BoundedAStar_greedyPlanner
%   % * To: fcn_Greedy_greedyPlanner
%
% 2025_11_16 to 2025_11_19 by Sean Brennan, sbrennan@psu.edu
% - Complete rewrite for clarity


% TO-DO
% 2025_11_21 by Sean Brennan, sbrennan@psu.edu
% -  (add to do here)

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
    MATLABFLAG_GREEDY_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_GREEDY_FLAG_CHECK_INPUTS");
    MATLABFLAG_GREEDY_FLAG_DO_DEBUG = getenv("MATLABFLAG_GREEDY_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_GREEDY_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_GREEDY_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_GREEDY_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_GREEDY_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

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

    h_openSet    = plot(nan, nan,'r.','MarkerSize',30,'DisplayName','Open set');
    h_qHistory   = plot(nan, nan,'g-','MarkerSize',10,'LineWidth', 3, 'DisplayName','Current q');
    h_allPoints  = plot(pointsWithData(:,1), pointsWithData(:,2),'.','MarkerSize',10,'Color',0.8*[1 1 1], 'DisplayName','All points'); %#ok<NASGU>
    h_successors = plot(nan, nan,'b-','MarkerSize',30,'DisplayName','Successors to open set');
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


    % Update the plot of the openSet
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
    qHistory(ith_depth,:) = qWithData;

    % Update the plot
    if flag_do_debug
        figure(debug_figNum);
        set(h_qHistory,'Xdata',qHistory(:,1),'Ydata',qHistory(:,2));
    end



    %%%%%%%%%%%%%%%
    % b) pop q off the open list by setting cost to go to q to infinity
    openSet(idx_of_q) = NaN;

    % Set the openSetCosts for this query node, q, to inf so it's not
    % called again.
    openSetCosts(idx_of_q,1) = inf;

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
    % USE THIS FOR DJKSTRAS --> nextConnectionCosts = cGraphModified(openIndices,:);
    nextConnectionCosts = cGraphModified(idx_of_q,:);

    % Keep only those that are not infinite
    indicesToSearch = find(~isinf(nextConnectionCosts));
    [~, toIndex] = find(~isinf(nextConnectionCosts));
    % USE THIS FOR DJKSTRAS --> actualFromIndex = openIndices(fromIndex);
    actualFromIndex = ones(length(toIndex),1)*idx_of_q;

    openParents{ith_depth,1} = [actualFromIndex, toIndex'];

    % Update the openSet info
    openSet(indicesToSearch) = indicesToSearch;
    openSetCosts(indicesToSearch) = nextConnectionCosts(indicesToSearch);

    % Update the plot
    if flag_do_debug
        figure(debug_figNum);
        dataToPlot = nan(length(indicesToSearch)*3,2);
        for ith_plot = 1:length(indicesToSearch)
            rowsToFill = (ith_plot-1)*3;
            dataToPlot(rowsToFill+1:rowsToFill+3,:) = [pointsWithData(idx_of_q,1:2); pointsWithData(indicesToSearch(ith_plot),1:2); nan(1,2)];
        end
        set(h_successors,'Xdata',dataToPlot(:,1),'Ydata',dataToPlot(:,2));
    end



    % Check exit conditions
    if all(isnan(openSet))
        flagFinishWasFound = 0;
        break
    end
    if any(openSet==finishPointIndex)
        flagFinishWasFound = 1;
        break;
    end
end % Ends while loop

cost = [];
if flagFinishWasFound

    qWithData = pointsWithData(finishPointIndex,:);
    qHistory(ith_depth+1,:) = qWithData;

    % Update the plot
    if flag_do_debug
        figure(debug_figNum);
        set(h_qHistory,'Xdata',qHistory(:,1),'Ydata',qHistory(:,2));
    end

    % Trace steps backwards to start point
    lastQ = finishPointIndex;
    Nlinks = ith_depth;
    backwardsRouteIndex = lastQ;
    for ith_back = Nlinks:-1:1
        routeIndex(ith_back,1) = lastQ;
        thisParentSet = openParents{ith_back};
        sourceIndex = find(thisParentSet(:,2)==lastQ, 1);
        if ~isempty(sourceIndex)
            lastQ = thisParentSet(sourceIndex,1);
            backwardsRouteIndex = [backwardsRouteIndex; lastQ]; %#ok<AGROW>
        end
    end
    routeIndex = flipud(backwardsRouteIndex);
    route = pointsWithData(routeIndex,1:2);

    % Calculate route length by finding the delta change in XY, then doing
    % sum of distances
    deltaRoute = diff(route,1,1);
    deltaDistances = sum(deltaRoute.^2,2).^0.5;
    cost = sum(deltaDistances);

end

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

    legend('Interpreter','none','Location','best');
    box on

    xlabel('x [m]');
    ylabel('y [m]');

    % Plot polytopes
    plotFormat.Color = 'blue';
    plotFormat.LineWidth = 2;

    % Call the function
    polytopes = fcn_VGraph_helperFillPolytopesFromPointData(pointsWithData, (-1));

    h = fcn_MapGen_plotPolytopes(polytopes,plotFormat,[1 0 0 0 0.5],figNum);
    set(h, 'HandleVisibility', 'off');

    % Plot path through field
    plot(startPointData(1), startPointData(2), 'gx','linewidth',2, 'DisplayName', 'Start')
    plot(finishPointData(1), finishPointData(2), 'rx','linewidth',2, 'DisplayName', 'Finish')

    % Plot the results
    plot(route(:,1),route(:,2),'.-','Color', [0 0 1], 'MarkerSize', 30, 'LineWidth', 3, 'DisplayName', 'Route');


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