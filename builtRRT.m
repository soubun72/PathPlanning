%function builtRRT
clear all
close all
sP = [2,0];
gP = [-2,0];

L1 = 2;
L2 = 1;
d = [0;0];
alpha = [0;0];
a = [L1;L2];
Point = [];
Joint = [];
Link = [];

Point(1,:) = sP;
nPoint = 1;
notDone = 1;

%check if the line between end point and start point intersect or not

if isNotIntersect(sP,gP)
    notDone = 0;
    line([sP(1),gP(1)],[sP(2),gP(2)]);
end

%draw obstacle
h = line([-1,-1,1,1,-1],[-1,1,1,-1,-1]);
set(h,'Color','red','LineWidth',2)
h = line([-3,3],[2,2])
set(h,'Color','red','LineWidth',2)
h = line([-3,3],[-2,-2])
set(h,'Color','red','LineWidth',2)
hold on

while notDone
    q1 = 360*rand();
    q2 = 360*rand();
    theta = [q1;q2];
    
    %get end effector position
    bTee = dh2ForwardKinematics(theta,d,a,alpha,1);
    X = bTee(1,4);
    Y = bTee(2,4);
    
    %check if the point is valid or not
    if ~(Y >= L1 || Y <= -L1 || ((X >= -L2 && X <= L2) && (Y >= -L2 && Y <= L2)))
        Point = [Point; X Y];
        Joint = [Joint; q1 q2];
        nPoint = nPoint + 1;
        
        %find the closest and non intersecting point
        for i = 1:nPoint-1
            distance(i) = pdist2(Point(i,:),Point(nPoint,:),'euclidean');
        end
        %sort the distance from the new point to all previous generated
        %point
        [distance,nodeIndex] = sort(distance);
        %nodeIndex is the number of link which is closer to the new point
        for i = 1:nPoint-1
            %check if the closest point is intersect or not
            PointI = Point(nodeIndex(i),:)
            xI = PointI(1,1);
            yI = PointI(1,2);
            PointN = Point(nPoint,:)
            xN = PointN(1,1);
            yN = PointN(1,2);
            if isNotIntersect(PointI,PointN)
                Link = [Link; nodeIndex(i), nPoint];
                plot([xI,xN], [yI,yN]);
                break;
            elseif i == nPoint -1
                nPoint = nPoint - 1;
                Point(end,:) = [];
                Joint(end,:) = [];
                IntersectAll = 1;
                break;
            end
        end
        
        if nPoint>=3 && isNotIntersect(gP, PointN) && ~IntersectAll
            notDone = 0;
            line([PointN(1),gP(1)], [PointN(2),gP(2)]);
        end
        IntersectAll = 0;
        if nPoint == 10000
            notDone = 0;
        end
    end
end
    