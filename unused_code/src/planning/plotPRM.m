function plotPRM(PRM, walls, boundary)
% PLOTPRM Plots PRM with obstacles
%
% Inputs:
%   PRM             - struct with fields V, E, startIdx, goalIdx
%   obstacleMatrix  - each row: [x1 y1 x2 y2 ...]
%   boundary        - [xmin ymin xmax ymax]

    hold on;

    % ---------------------------
    % Plot walls
    % ---------------------------
    for i = 1:length(walls)
        plot([walls(i,1) walls(i,3)], [walls(i,2) walls(i,4)], 'Color', 'k');
    end

    % ---------------------------
    % Plot edges
    % ---------------------------
    for i = 1:size(PRM.E,1)
        p1 = PRM.V(PRM.E(i,1),:);
        p2 = PRM.V(PRM.E(i,2),:);

        plot([p1(1), p2(1)], [p1(2), p2(2)], 'b');
    end

    % ---------------------------
    % Plot vertices
    % ---------------------------
    plot(PRM.V(:,1), PRM.V(:,2), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r');

    % ---------------------------
    % Plot start and goal
    % ---------------------------
    if isfield(PRM, 'startIdx')
        s = PRM.V(PRM.startIdx,:);
        plot(s(1), s(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    end

    if isfield(PRM, 'goalIdx')
        g = PRM.V(PRM.goalIdx,:);
        plot(g(1), g(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);
    end

    % ---------------------------
    % Axis formatting
    % ---------------------------
    if nargin > 2 && ~isempty(boundary)
        xmin = boundary(1);
        ymin = boundary(2);
        xmax = boundary(3);
        ymax = boundary(4);

        xlim([xmin xmax]);
        ylim([ymin ymax]);
    end

    axis equal;
    title('PRM');

    hold off;
end

