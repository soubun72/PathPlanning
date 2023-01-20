function planPathPRM

clear all

%start point
sP = [2,0];

%goal point
gP = [-2,0];

%number of point
nPoint = 50;

[Point, Joint, Link] = buildPRM(sP,gP,nPoint)

connectionMatrix = zeros(nPoint);

for i = 1:size(Link,1)
    connectionMatrix(Link(i,1),Link(i,2)) = 1;
    connectionMatrix(Link(i,2),Link(i,1)) = 1;
end

%plan path

[nbNodes, visibilityGraph] = createVisibilityGraph(connectionMatrix, Point)
[distanceToNode, parentOfNode, nodeTrajectory] = dijkstra(nbNodes, visibilityGraph)


%starting point
TrajectoryX = [sP(1)];
TrajectoryY= [sP(2)];
numberOfTrajectory = size(nodeTrajectory,2)-1;
for i = numberOfTrajectory:-1:1
    %connect to latest point
    trajectPoint(i,:) = Point(nodeTrajectory(i),:);
    TrajectoryX = [TrajectoryX, trajectPoint(i,1)];
    TrajectoryY = [TrajectoryY, trajectPoint(i,2)];
end
%connect last point to previous point
TrajectoryX = [TrajectoryX, gP(1)];
TrajectoryY = [TrajectoryY, gP(2)];

%plot the path
subplot(1,3,3)
xlim([-3,3]);
ylim([-3,3]);
line(TrajectoryX, TrajectoryY)
h = line([-1,-1,1,1,-1],[-1,1,1,-1,-1]);
set(h,'Color','red','LineWidth',2)
h = line([-3,3],[2,2])
set(h,'Color','red','LineWidth',2)
h = line([-3,3],[-2,-2])
set(h,'Color','red','LineWidth',2)

d = 0;
for i = 1:size(TrajectoryX,2)-1
    d = d + sqrt((TrajectoryX(i)-TrajectoryX(i+1))^2+(TrajectoryY(i)-TrajectoryY(i+1))^2);
end
%travel distance
print(d)
