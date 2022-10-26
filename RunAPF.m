clear all
close all
clc

% 
% rng(7,"twister");
omap3D =  occupancyMap3D(1); %keep this as 1 to work
mapWidth = 30; %meters
mapLength = 30;
numberOfObstacles = 45;
obstacleNumber = 1;

while obstacleNumber <= numberOfObstacles
    width = randi([1 5],1);                                       
    lengthh = randi([1 5],1);                % can be changed as necessary to create different occupancy maps.
    height = randi([1 5],1);
    xPosition = randi([0 mapWidth-width],1);
    yPosition = randi([0 mapLength-lengthh],1);
    
    [xObstacle,yObstacle,zObstacle] = meshgrid(xPosition:xPosition+width,yPosition:yPosition+lengthh,0:height);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    
    checkIntersection = false;
    for i = 1:size(xyzObstacles,1)
        if checkOccupancy(omap3D,xyzObstacles(i,:)) == 1
            checkIntersection = true;
            break
        end
    end
    if checkIntersection
        continue
    end
    
    setOccupancy(omap3D,xyzObstacles,1)
    
    obstacleNumber = obstacleNumber + 1;
end
% ground as obstacle
[xGround,yGround,zGround] = meshgrid(0:mapWidth,0:mapLength,0);
xyzGround = [xGround(:) yGround(:) zGround(:)];
setOccupancy(omap3D,xyzGround,1)


rho0 = 0.3; %distance of influence, adaptative given velocity and freespace?
ka = 1;
krep = 1;
goal.pos=[20 20 6];
box = collisionBox(0.4,0.4,0.1);  % drone as a box XYZ 
currentpos=[0.5 0.5 2];
box.Pose = trvec2tform(currentpos);

movement=[0 0 0];
Xhistory=0;
Yhistory=0;
Zhistory=0;
fh = figure();
fh.WindowState = 'maximized';
pause(1)
while norm(goal.pos-currentpos)>0.1


bpOpts = occupancyMap3DCollisionOptions(ReturnDistance=true);
[collisionStatus,details] = checkMapCollision(omap3D,box,bpOpts);
rho = details.DistanceInfo.Distance;
h=0.01;
trvec = tform2trvec(box.Pose);

box.Pose = trvec2tform([trvec(1) trvec(2) trvec(3)]);
newX = trvec2tform([trvec(1)+h trvec(2) trvec(3)]);
boxCopyx = collisionBox(box.X,box.Y,box.Z);  
boxCopyx.Pose=box.Pose;
boxCopyx.Pose=newX;
[collisionStatus,detailsX] = checkMapCollision(omap3D,boxCopyx,bpOpts);
rhoX = detailsX.DistanceInfo.Distance;
newY = trvec2tform([trvec(1) trvec(2)+h trvec(3)]);
boxCopyy = collisionBox(box.X,box.Y,box.Z);  
boxCopyy.Pose=box.Pose;
boxCopyy.Pose=newY;
[collisionStatus,detailsY] = checkMapCollision(omap3D,boxCopyy,bpOpts);
rhoY = detailsY.DistanceInfo.Distance;
newZ = trvec2tform([trvec(1) trvec(2) trvec(3)+h]);
boxCopyz = collisionBox(box.X,box.Y,box.Z);  
boxCopyz.Pose=box.Pose;
boxCopyz.Pose=newZ;
[collisionStatus,detailsZ] = checkMapCollision(omap3D,boxCopyz,bpOpts);
rhoZ = detailsZ.DistanceInfo.Distance;
% forward difference for gradient
gradrho = [(rhoX-rho)/h (rhoY-rho)/h (rhoZ-rho)/h];



bpWitnessptsSphere = details.DistanceInfo.WitnessPoints;


show(omap3D)
hold on
show(box)
plot3(bpWitnessptsSphere(1,:),bpWitnessptsSphere(2,:),bpWitnessptsSphere(3,:),LineWidth=2,Color='r')

state.pos=[trvec(1) trvec(2) trvec(3)];

Fa = -ka*(state.pos-goal.pos);
if rho>rho0
    Fr = [0 0 0];
else
    Fr = krep*(((1/rho)-(1/rho0))*(1/rho)^2)*gradrho;
end
% centerobstacle = bpWitnessptsSphere(:,1)';
% Rspin = cross((goal.pos-state.pos),(centerobstacle-state.pos));

%plot3([box.X box.X+10*Fr(1)],[box.Y box.Y+10*Fr(2)],[box.Z box.Z+10*Fr(3)],LineWidth=2,Color='k')
[X,Y,Z] = sphere;
r = 0.2;
X = X * r;
Y = Y * r;
Z = Z * r;
surf(X+goal.pos(1),Y+goal.pos(2),Z+goal.pos(3),'FaceColor','k','EdgeColor','none')

quiver3(trvec(1),trvec(2),trvec(3),Fr(1)/norm(Fr),Fr(2)/norm(Fr),Fr(3)/norm(Fr),LineWidth=3,Color='k')
quiver3(trvec(1),trvec(2),trvec(3),Fa(1)/norm(Fa),Fa(2)/norm(Fa),Fa(3)/norm(Fa),LineWidth=3,Color='b')
quiver3(trvec(1),trvec(2),trvec(3),(Fr(1)+Fa(1))/norm(Fa+Fr),(Fr(2)+Fa(2))/norm(Fa+Fr),(Fr(3)+Fa(3))/norm(Fa+Fr),LineWidth=3,Color='y')
% quiver3(trvec(1),trvec(2),trvec(3),Rspin(1)/norm(Rspin),Rspin(2)/norm(Rspin),Rspin(3)/norm(Rspin),LineWidth=3,Color='g')


axis equal
plot3(Xhistory,Yhistory,Zhistory,'r')
view(-15.6658,30.0343)

legend({'Obstacle','Obstacle','Nearest Obstacle','Goal','Repulsive Force','Attractive Force','Total Force','Path'})

hold off

movement = (Fr+Fa);
movement = 0.1*movement/norm(movement);
trvec = trvec+movement;
currentpos = [trvec(1) trvec(2) trvec(3)];
box.Pose=trvec2tform([trvec(1) trvec(2) trvec(3)]);
Xhistory=[Xhistory trvec(1)];
Yhistory=[Yhistory trvec(2)];
Zhistory=[Zhistory trvec(3)];
pause(0.001)

end