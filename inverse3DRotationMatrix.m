function invRotationMatrix = inverse3DRotationMatrix(rotationMatrix)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function invRotationMatrix = inverse3DRotationMatrix(rotationMatrix)
% Task: Inverse a 3D rotation matrix 
%
% Inputs:
%	- rotationMatrix: the rotation matrix to inverse
%
% Output: 
%	-invRotationMatrix: the inverse of the rotation matrix
%
%
% author: Guillaume Gibert, guillaume.gibert@ecam.fr
% date: 25/01/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% checks if the input rotation matrix has the right size
if (size(rotationMatrix, 1) != 3 || size(rotationMatrix, 2) != 3)
	fprintf('[ERROR] (inverseRotationMatrix) -> the size of the input rotation matrix is not 3x3!\n');
end

invRotationMatrix = rotationMatrix';