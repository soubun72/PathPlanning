function invTransformationMatrix = inverse3DTransformationMatrix(transformMatrix)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function invTransformMatrix = inverse3DTransformationMatrix(transformMatrix)
% Task: Inverse a 3D transformation matrix 
%
% Inputs:
%	- transformMatrix: the transform matrix to inverse
%
% Output: 
%	- invTransformationMatrix: the inverse of the transformation matrix
%
%
% author: Guillaume Gibert, guillaume.gibert@ecam.fr
% date: 25/01/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% checks if the input transform matrix has the right size
if (size(transformMatrix, 1) != 4 || size(transformMatrix, 2) != 4)
	fprintf('[ERROR] (inverseTransformationMatrix) -> the size of the input transform matrix is not 4x4!\n');
end

% retrieves the rotation matrix
rotationMatrix = transformMatrix(1:3, 1:3);

%retrieves the translation matrix
translationMatrix = transformMatrix(1:3, 4);

% inverses the rotation matrix
invRotationMatrix = inverse3DRotationMatrix(rotationMatrix);

% inverses the translation matrix
invTranslationMatrix = -invRotationMatrix * translationMatrix;

% create the inverse of the transform matrix
% ( R^-1 | -R^-1t )
%  ---  -----  -----  --
% ( 0 | 1)
% with R: the rotation matrix (3x3)
% and t: the translation matrix (3x1)
invTransformationMatrix = [invRotationMatrix invTranslationMatrix; 0 0 0 1];


