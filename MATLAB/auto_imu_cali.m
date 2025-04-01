clc
clear
posSFromP = [0 0.01 0.02];
orientSfromP = quaternion([3,0,0],'eulerd','ZYX','frame')

posPlat = [0 0 0];
orientPlat = quaternion(1, 0, 0, 0);
velPlat = [0 0 0];
accPlat = [0 0 0];
angvelPlat = [pi 0 0];

[posIMU, orientIMU, velIMU, accIMU, angvelIMU] = transformMotion( ...
    posSFromP, orientSfromP, ...
    posPlat, orientPlat, velPlat, accPlat, angvelPlat);

rotMMis = eul2rotm([pi/40 pi/40 0]);

isFirst = 1;
diffwxs = zeros(100,1);
diffwys = zeros(100,1);
diffvxs = zeros(100,1);
diffvys = zeros(100,1);

for i=1:100
    
    angvelIMU = (rotMMis * angvelPlat') + (-0.01+0.02.*rand(3,1));
    velIMU_ = velIMU'+ (-0.01+0.02.*rand(3,1));
    
    A = matW(pi+0.01.*rand(1));
    B = [angvelIMU(2) angvelIMU(3) velIMU_(2) velIMU_(3)]';
    
    if(isFirst)
        [X,S,V] = IMU_RTLS(A,B,0,0,1);
        diffwxs(i) = X(1)-(pi/40);
        diffwys(i) = X(2)-(pi/40);
        diffvxs(i) = X(3)-0.1;
        diffvys(i) = X(4)-0.2;
        isFirst = 0;
    else
        [X,S,V] = IMU_RTLS(A,B,S,V,0);
        diffwxs(i) = X(1)-(pi/40);
        diffwys(i) = X(2)-(pi/40);
        diffvxs(i) = X(3)+0.01;
        diffvys(i) = X(4)+0.02;
    end
end
yyaxis left
plot(diffwxs,'LineWidth',1.5)
hold on
plot(diffwys,'LineWidth',1.5)
yyaxis right
plot(diffvxs,'LineWidth',1.5)
plot(diffvys,'LineWidth',1.5)
hold off

yyaxis left
ylim([-3e-3 3e-3])
ylabel('Error in angle (rad)')
yyaxis right
ylim([-3e-3 3e-3])
ylabel('Error in offset (m)')

legend('\alpha','\beta','y','z')

xlabel('Number of recursions')