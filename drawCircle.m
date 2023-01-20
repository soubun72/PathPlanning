function h = drawCircle(x,y,r)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function h = drawCircle(x,y,r)
% Task: Draw a circle providing its center and radius
%
% Inputs:
%	- x: the x-coordinate of the circle center (in m)
%	- y: the y-coordinate of the circle center (in m)
%	- r: the radius of the circle center (in m)
%
% Outputs: 
%	- h: a reference to the plot figure
%	
%
% author: Guillaume Gibert, guillaume.gibert@ecam.fr
% date: 14/09/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% holds the previous drawing
hold on;

% generates samples in the range [0, 2pi]
th = 0:pi/50:2*pi;

% computes (x,y) samples along the circle perimeter
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;

% plots the samples
h = plot(xunit, yunit, 'r');
