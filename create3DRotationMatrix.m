function rotationMatrix = create3DRotationMatrix(roll, pitch , yaw, order)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function  rotationMatrix = create3DRotationMatrix(roll, pitch , yaw)
% Task: Compute the 3D rotation matrix from the values of roll, pitch, yaw angles
%
% Inputs:
%	- roll: the value of the roll angle in degrees
%	- pitch: the value of the pitch angle in degrees
%	- yaw: the value of the yaw angle in degrees
%	- order: if equal to 1, ZYX; if equal to 0, XYZ
%
% Output: 
%	- rotMatrix: the rotation matrix corresponding to the roll, pitch, yaw angles
%
%
% author: Guillaume Gibert, guillaume.gibert@ecam.fr
% date: 25/01/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% convert the input angles from degrees to radians
rollAngleInRadians = roll / 180.0 * pi;
pitchAngleInRadians = pitch / 180.0 * pi;
yawAngleInRadians = yaw / 180.0 * pi;

% ZYX or XYZ direction
switch order
	case 0
		thetaX = yawAngleInRadians;
		thetaY = pitchAngleInRadians;
		thetaZ = rollAngleInRadians;
	case 1
		thetaX = rollAngleInRadians;
		thetaY = pitchAngleInRadians;
		thetaZ = yawAngleInRadians;
	otherwise
		disp('[ERROR](create3DRotationMatrix)-> order value is neither 0 or 1!')
end

Rz = [cos(thetaZ) -sin(thetaZ) 0;
	sin(thetaZ) cos(thetaZ) 0;
	0	0	1];

Ry = [cos(thetaY) 0 sin(thetaY);
	0 1 0;
	-sin(thetaY) 0 cos(thetaY)];
	
Rx = [1 0 0;
	0 cos(thetaX) -sin(thetaX);
	0 sin(thetaX) cos(thetaX)];


% ZYX or XYZ direction
switch order
	case 0
		rotationMatrix = Rx * Ry * Rz
	case 1
		rotationMatrix = Rz * Ry * Rx;
	otherwise
		disp('[ERROR](create3DRotationMatrix)-> order value is neither 0 or 1!')
end


