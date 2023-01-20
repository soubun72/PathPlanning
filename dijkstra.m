function [distanceToNode, parentOfNode, nodeTrajectory] = dijkstra(nbNodes, visibilityGraph);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function [ToNode, parentOfNode, nodeTrajectory] = dijkstra(nbNodes, visibilityGraph)
%
% Task: Perform the Dijkstra algorithm on a given visibility graph
%
% Inputs:
%	-nbNodes: number of nodes of the graph excluding the starting and goal points
%	-visibilityGraph: a matrix containing the distance between connected nodes 
%		(NaN refers to not connected nodes)
%		The matrix has a size of (nbNodes+2)x(nbNodes+2)
%
% Outputs:
%	- distanceToNode: distance between the current node and its parent
%	- parentOfNode: index of the parent node for each node
%	- nodeTrajectory: best trajectory
%
% Guillaume Gibert (guillaume.gibert@ecam.fr)
% 17/03/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

constantLargeDitance=10000;

visitedNodes = zeros(1, nbNodes+2);
distanceToNode = constantLargeDitance*ones(1, nbNodes+2);
distanceToNode(1) = 0;
parentOfNode = zeros(1, nbNodes+2);

fprintf('##Starting Dijkstra''s algorithm...\n')

while (sum(visitedNodes(:)==0))
	thresholdDistance = constantLargeDitance+1;
	for l_node=1:nbNodes+2
		%l_node
		if (visitedNodes(l_node)==0 &&  distanceToNode(l_node) < thresholdDistance)
			minIndex = l_node;
			thresholdDistance = distanceToNode(l_node);
		end
	end
	
	fprintf('-->Visiting N%d\n', minIndex-1)
	
	visitedNodes(minIndex) = 1;
	for l_node=1:nbNodes+2
		%l_node
		if (l_node~=minIndex && ~isnan(visibilityGraph(minIndex, l_node)))
			distance = distanceToNode(minIndex) + visibilityGraph(minIndex,l_node);
			if (distance < distanceToNode(l_node))
				distanceToNode(l_node) = distance;
				parentOfNode(l_node) = minIndex;
			end
		end
	end
end
fprintf('##Dijkstra''s algorithm is done!\n')
fprintf('##Results\n')
fprintf('Minimal distance to target: %d\n', distanceToNode(nbNodes+2))
nodeIndex = nbNodes+2;
nodeTrajectory = [];
while(nodeIndex~=1)
	nodeIndex = parentOfNode(nodeIndex);
	nodeTrajectory = [nodeTrajectory nodeIndex];
end
fprintf('S-->');
for l_node=2:length(nodeTrajectory)
	fprintf('N%d-->', nodeTrajectory(length(nodeTrajectory)-(l_node-1))-1);
end
fprintf('G\n');
fprintf('########\n');

