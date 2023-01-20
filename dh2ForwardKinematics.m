function jTee = dh2ForwardKinematics(theta, d, a, alpha, jointNumber)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function  wTee = dh2ForwardKinematics(theta, d, a, alpha, jointNumber)
% Task: Determine the 3D transformation matrix corresponding to a set of Denavit-Hartenberg parameters
%
% Inputs:
%	- theta: an array of theta parameters (rotation around z in degrees)
%	- d: an array of d parameters (translation along z in mm)
%	- a: an array of a parameters (translation along x in mm)
%	- alpha: an array of alpha parameters (rotation around x in degrees)
%	- jointNumber: joint number you want to start with (>=1 && <=size(theta,1)) 
%
% Output: 
%	-jTee: the transformation matrix from the {World} reference frame to the {end-effector} reference frame
%
%
% author: Guillaume Gibert, guillaume.gibert@ecam.fr
% date: 29/01/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% checks if the arrays have the same size
if (size(theta, 1) ~= size(d,1) || size(theta,1) ~= size(a, 1) || size(theta,1) ~= size(alpha, 1))
	disp('[ERROR](dh2ForwardKinematics)-> sizes of input arrays do not match!')
	return;
end

% creates the output matrix as an identity one
jTee = eye(4);

% checks if jointNumber is in the good range  [1 size(theta,1)]
if (jointNumber < 1 || jointNumber > size(theta, 1))
	disp('[ERROR](dh2ForwardKinematics)-> jointNumber is out of range!')
	return;
end

% loops over all the joints and create the transformation matrix as follow:
% for joint i: Trot(theta(i), z) Ttrans(d(i), z) Ttrans (a(i), x) Trot(alpha(i), x)
for l_joint=jointNumber:size(theta, 1)
	% determine the transformation matrices for theta, d, a and alpha values of each joint
	thetaTransformMatrix = create3DTransformationMatrix(0, 0, theta(l_joint), 1, 0, 0, 0); % Rz
	dTransformMatrix =  create3DTransformationMatrix(0, 0, 0, 1, 0, 0, d(l_joint)); % Tz
	aTransformMatrix = create3DTransformationMatrix(0, 0, 0, 1, a(l_joint), 0, 0); % Tx
	alphaTransformMatrix = create3DTransformationMatrix(alpha(l_joint), 0, 0, 1, 0, 0, 0); % Rx
	
	jTee = jTee * thetaTransformMatrix * dTransformMatrix * aTransformMatrix *alphaTransformMatrix;
end
