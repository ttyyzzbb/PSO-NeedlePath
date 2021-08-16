function circleObstacles = setObstacles3D
%设置障碍物圆心和半径
%返回值ciecleObstacle=zeros(M,3), (:,1) 为x坐标  (:,2)为y坐标， (:,3)为z坐标, (:,4)为圆半径
%
%scene1

% M = 5;%障碍物个数
% circleObstacles = zeros(M,4);
% 
% circleObstacles(1,:) = [-10,10,20,8];
% circleObstacles(2,:) = [10,-10,30,15];
% circleObstacles(3,:) = [10,10,50,8];
% circleObstacles(4,:) = [10,-10,70,10];
% circleObstacles(5,:) = [-20,10,100,8];

% M = 11;
% circleObstacles = zeros(M,4); 
% circleObstacles(1,:) = [-14, -3, -20, 11];
% circleObstacles(2,:) = [-14, 4, -20, 11];
% circleObstacles(3,:) = [-14, 13, -20, 11];
% circleObstacles(4,:) = [-14, 22, -20, 11];
% circleObstacles(5,:) = [-14, -10, -31, 9];
% circleObstacles(6,:) = [-14, -21, -29, 9];
% circleObstacles(7,:) = [-11, 6, -35, 9];
% circleObstacles(8,:) = [-11, -3, -33, 9];
% circleObstacles(9,:) = [-15, -60, -42, 7];
% circleObstacles(10,:) = [-25, -57, -42, 7];
% circleObstacles(11,:) = [-1, -59, -42, 7];

M = 8;
circleObstacles = zeros(M,4); 
circleObstacles(1,:) = [-14, -3, 70, 11];
%circleObstacles(2,:) = [-14, 4, -20, 11];
%circleObstacles(3,:) = [-14, 13, -20, 11];
circleObstacles(2,:) = [-14, 22, 70, 11];
circleObstacles(3,:) = [-14, -10, 60, 9];
%circleObstacles(6,:) = [-14, -21, -29, 9];
circleObstacles(4,:) = [-11, 6, 55, 9];
circleObstacles(5,:) = [-11, -3, 57, 9];
%circleObstacles(9,:) = [-15, -60, -42, 7];
%circleObstacles(6,:) = [-25, -35, 48, 7];
%circleObstacles(11,:) = [-1, -59, -42, 7];
circleObstacles(6,:) = [-16, -53, 65, 7];
circleObstacles(7,:) = [-16, -53, 74, 7];
circleObstacles(8,:) = [-13, -20, 45, 5];

%scene2
% M = 6;%障碍物个数
% circleObstacles = zeros(M,4); 
% circleObstacles(1,:) = [-10,-2,25,5];
% circleObstacles(2,:) = [5,-8,40,8];
% circleObstacles(3,:) = [-2,10,10,7];
% circleObstacles(4,:) = [12,2,40,6];
% circleObstacles(5,:) = [6,15,60,5];
% circleObstacles(6,:) = [2,10,90,10];

% circleObstacles(2,:) = [-6,10,20,5];
% circleObstacles(3,:) = [-2,10,20,5];
% circleObstacles(4,:) = [2,10,20,5];
% circleObstacles(5,:) = [6,10,20,5];
% circleObstacles(6,:) = [10,10,20,5];


%replan
% M = 1;
% circleObstacles = zeros(M,4);
% circleObstacles(1,:) = [0,5,80,9]; %1   8是半径
% circleObstacles(1,:) = [0,5,60,10]; %move 2


end