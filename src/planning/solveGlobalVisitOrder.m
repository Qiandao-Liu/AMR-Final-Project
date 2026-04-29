function [visitNodeOrder, visitGoalOrder, totalCost] = solveGlobalVisitOrder(costMatrix, startNodeIdx, remainingGoalNodeIdx)
% SOLVEGLOBALVISITORDER Solve the exact shortest visit order using
% Held-Karp DP on a small goal set.

remainingGoalNodeIdx = remainingGoalNodeIdx(:)';
numGoals = numel(remainingGoalNodeIdx);

if numGoals == 0
    visitNodeOrder = zeros(1, 0);
    visitGoalOrder = zeros(1, 0);
    totalCost = 0;
    return;
end

goalCost = costMatrix(startNodeIdx, remainingGoalNodeIdx);
if numGoals == 1
    visitNodeOrder = remainingGoalNodeIdx;
    visitGoalOrder = 1;
    totalCost = goalCost(1);
    return;
end

numMasks = 2 ^ numGoals;
dp = inf(numMasks, numGoals);
parent = zeros(numMasks, numGoals);

for j = 1:numGoals
    mask = bitset(0, j, 1) + 1;
    dp(mask, j) = goalCost(j);
end

for mask0 = 1:(numMasks - 1)
    mask = mask0 + 1;
    for j = 1:numGoals
        if bitget(mask0, j) == 0 || ~isfinite(dp(mask, j))
            continue;
        end
        for k = 1:numGoals
            if bitget(mask0, k) ~= 0
                continue;
            end
            nextMask0 = bitset(mask0, k, 1);
            nextMask = nextMask0 + 1;
            edgeCost = costMatrix(remainingGoalNodeIdx(j), remainingGoalNodeIdx(k));
            candidateCost = dp(mask, j) + edgeCost;
            if candidateCost < dp(nextMask, k)
                dp(nextMask, k) = candidateCost;
                parent(nextMask, k) = j;
            end
        end
    end
end

fullMask = numMasks;
[totalCost, bestLast] = min(dp(fullMask, :));
if ~isfinite(totalCost)
    visitNodeOrder = zeros(1, 0);
    visitGoalOrder = zeros(1, 0);
    return;
end

visitGoalOrder = zeros(1, numGoals);
mask0 = numMasks - 1;
last = bestLast;
for pos = numGoals:-1:1
    visitGoalOrder(pos) = last;
    prev = parent(mask0 + 1, last);
    mask0 = bitset(mask0, last, 0);
    last = prev;
end

visitNodeOrder = remainingGoalNodeIdx(visitGoalOrder);
end
