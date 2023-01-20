function [nbNodes, visibilityGraph] = createVisibilityGraph(connectionMatrix, points2D)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function [nbNodes, visibilityGraph] = createVisibilityGraph(connectionMatrix, points2D)
%
% Task: Create a visibility graph from a connection matrix and a set of 2D points
%
% Inputs:
%	-connectionMatrix: matrix of connection if cell is equal to 1 there is an edge between the corresponding points, cell is 0 otherwise
%	-points2D: coordinates of the vertices of the graph
%
% Outputs:
%	-nbNodes: the number of nodes of this graph
%	-visibilityGraph: a matrix containing the distance between connected nodes 
%		(NaN refers to not connected nodes)
%		The matrix has a size of (nbNodes+2)x(nbNodes+2)
%
% Guillaume Gibert (guillaume.gibert@ecam.fr)
% 19/03/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nbNodes = size(points2D,1)-2;
visibilityGraph = NaN(nbNodes+2, nbNodes+2);

for l_row=1:size(connectionMatrix,1)
	for l_col=1:size(connectionMatrix,2)
		if (connectionMatrix(l_row, l_col) == 1)
			% computes the distance between the 2 points
			distance = sqrt( (points2D(l_row,1)-points2D(l_col,1))^2 + (points2D(l_row,2)-points2D(l_col,2))^2);
			visibilityGraph(l_row, l_col) =distance;
		end
	end
end


