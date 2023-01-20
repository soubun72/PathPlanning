function [Point, Joint, link] = buildPRM(startPoint, endPoint, maxPoint)
Number_of_point = 1;
L1 = 2;
L2 = 1;
d = [0;0];
alpha = [0;0];
a = [L1;L2];
Point = [];
Joint = [];
link = [];
while Number_of_point <= maxPoint
    q1 = 360*rand();
    q2 = 360*rand();
    theta = [q1;q2];
    
    %do this for the first point
    if Number_of_point == 1
        theta = inverseKinematics(d,a,alpha,startPoint);
        q1 = theta(1,1);
        q2 = theta(2,1);
    end
    
    %do this for the last point
    if Number_of_point == maxPoint
        theta = inverseKinematics(d,a,alpha,endPoint);
        q1 = theta(1,1);
        q2 = theta(2,1);
    end
    
    %calculate end effector position
    bTee = dh2ForwardKinematics(theta,d,a,alpha,1);
    X = bTee(1,4);
    Y = bTee(2,4);
    
    %check if the newly generated point is valid or not
    if ~(Y >= L1 || Y <= -L1 || ((X >= -L2 && X <= L2) && (Y >= -L2 && Y <= L2)))
        Point = [Point; X Y];
        Joint = [Joint; q1 q2];
        Number_of_point = Number_of_point + 1;
    end
end

for i = 1:maxPoint
    for j = i+1:maxPoint
        point1 = Point(i,:);
        point2 = Point(j,:);
        
        %link the point if no intersection
        if isNotIntersect(point1,point2)
            link = [link; i j];
        end
    end
end

%plot the cartesian space
subplot(1,3,1)
title("Cartesian space")
xlim([-3,3]);
ylim([-3,3]);
hold on

for i = 1:size(link,1)
    Point1 = Point(link(i,1),:);
    Point2 = Point(link(i,2),:);
    
    plot([Point1(1,1), Point2(1,1)], [Point1(1,2), Point2(1,2)]);
end

h = line([-1,-1,1,1,-1],[-1,1,1,-1,-1]);
set(h,'Color','red','LineWidth',2)
h = line([-3,3],[2,2])
set(h,'Color','red','LineWidth',2)
h = line([-3,3],[-2,-2])
set(h,'Color','red','LineWidth',2)

hold off

%plot the joint space
subplot(1,3,2)
title("Joint space");
hold on
for i = 1:size(link,1)
    Joint1 = Joint(link(i,1),:);
    Joint2 = Joint(link(i,2),:);
    plot([Joint1(1,1), Joint2(1,1)], [Joint1(1,2), Joint2(1,2)]);
end
hold off
save map link Point Joint
