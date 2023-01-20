function transformationMatrix = create3DTransformationMatrix(roll, pitch, yaw, order, tX, tY, tZ)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function transformationMatrix = create3DTransformationMatrix(roll, pitch, yaw, order, tX, tY, tZ)
% Task: Create the 3D transformation matrix corresponding to roll, ptich, yaw angles and a 3D translation
%
% Inputs:
%	- roll: the value of the roll angle in degrees
%	- pitch: the value of the pitch angle in degrees
%	- yaw: the value of the yaw angle in degrees
%	- order: the order of rotation if 1 ZYX, if 0 XYZ
%	- tX = the value of the translation along x in mm
%	- tY = the value of the translation along y in mm
%	- tZ = the value of the translation along z in mm
%
% Output: 
%	- transformationMatrix: the transformation matrix corresponding to  roll, ptich, yaw angles and a 3D translation
%
%
% author: Guillaume Gibert, guillaume.gibert@ecam.fr
% date: 25/01/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% determine the rotation matrix (3 x 3)
rotationMatrix = create3DRotationMatrix(roll, pitch, yaw, order);

% create the translation part (3 x 1)
translationMatrix = [tX; tY; tZ];

% create the homogeneous coordinate part
homogeneousCoord = [0 0 0 1];

% create the transformation matrix which shape is
% ( R | t )
%  ---  --
% ( 0 | 1)
% with R: the rotation matrix (3x3)
% and t: the translation matrix (3x1)
transformationMatrix = [ rotationMatrix translationMatrix; homogeneousCoord];

