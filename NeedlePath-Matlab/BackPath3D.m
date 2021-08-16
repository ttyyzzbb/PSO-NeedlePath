function [Point_pic]=BackPath3D(P0, Ptarget, arcsLength, radius)
%arcsLength
len = size(arcsLength);
len = len(1) + len(2) -1;%一维数组长度，这么做可处理横向量和列向量
len = len/2;

%初始切向量
p0c = [0 0 1];
%初始圆心位置
O0 = [0 radius 0];


circleObstacles = setObstacles3D;
obstacles = circleObstacles;%设置障碍物球心和半径
olen = size(obstacles);% 1 4
olen = olen(1);% 1 代表有几个障碍物

%考虑旋转点切线与起点到目标向量的夹角，夹角越大，路径越偏离目标位置
%切向量与目标向量夹角，cos\gamma = corss(1,2)/|2|?????????????????????????????????????
%1-cos\gamma越大，夹角越大
p0pt = Ptarget - P0;%起点到终点的向量
p0pt = p0pt/norm(p0pt); %norm求得向量的模，此处也就是求单位向量

len = size(arcsLength);
len = len(1) + len(2) -1;%一维数组长度，这么做可处理横向量和列向量
len = len/2;

Point_pic = zeros(len+1, 9); % 存储停车转向点和方向向量和圆心
Point_pic(1, :) = [P0, p0c, O0];

thetas = zeros(1, len);%此时是1行20列的零矩阵
points = zeros(len+1,3);%此时是21行3列的零矩阵
points(1,:) = P0;%此时是[0 0 0]，第一行是出发点的坐标

pic = p0c;%初始切向量
pi2c = pic;
Oi=O0;%初始圆心坐标
Oi2=O0;%用于存储将Oi旋转后的值

isStop = 0;
nonZeroLength = 2*len;

for i=1:len
%     figure(4)
%     plot3(Oi(1),Oi(2),Oi(3),'r.')
%     hold on
    thetas(i) = arcsLength(2*i)/radius;%弧长/半径=弧度
    alpha = arcsLength(2*i-1);%从arcLength行矩阵里读取旋转角度
    %计算旋转后的圆心位置
    sa=sin(alpha);%计算后的值存在sa里方便后续使用
    ca=cos(alpha);
    %将圆心转换为向量
    Oi=Oi-points(i,:);%初始圆心坐标的向量形式
    %对向量进行旋转
    Oi2(1) = Oi(3)*(pic(2)*sa-pic(1)*pic(3)*(ca-1)) -  Oi(2)*(pic(3)*sa+pic(1)*pic(2)*(ca-1)) + Oi(1)*(ca-pic(1)^2*(ca-1));
    Oi2(2) = Oi(1)*(pic(3)*sa-pic(1)*pic(2)*(ca-1)) -  Oi(3)*(pic(1)*sa+pic(2)*pic(3)*(ca-1)) + Oi(2)*(ca-pic(2)^2*(ca-1));
    Oi2(3) = Oi(2)*(pic(1)*sa-pic(2)*pic(3)*(ca-1)) -  Oi(1)*(pic(2)*sa+pic(1)*pic(3)*(ca-1)) + Oi(3)*(ca-pic(3)^2*(ca-1));
    %从向量中获取旋转后的圆心
    Oi2 = Oi2 +  points(i,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %绘路径
    figure(1)
    for angle=0:0.01:thetas(i)
        P=points(i,:)*cos(angle)+(1-cos(angle))*Oi2+radius*pic*sin(angle);
        plot3(P(1),P(2),P(3),'b.')
        hold on
        if points(i+1, 3) > Ptarget(3)%超过设定的终点z轴坐标则停止
            isStop = 1;
            nonZeroLength = 2*i;
            break
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %计算穿刺后的端点位置
    st = sin(thetas(i));
    ct = cos(thetas(i));
    points(i+1,:) = ct*points(i,:) + (1-ct)*Oi2 + radius*st*pic;
    
    %计算端点处切向量
    pi2c=Oi2*(1-ct)+points(i+1,:)*ct-points(i,:);
    %向量单位化
    pi2c = pi2c/norm(pi2c);
    
    Point_pic(i+1, :) = [points(i+1,:), pi2c, Oi2];
    
    %更新临时变量
    pic = pi2c;
    Oi=Oi2; 
end
%大循环结束

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%画障碍物
for i=1:olen %olen代表有几个障碍物
    [x,y,z]=ellipsoid(obstacles(i,1),obstacles(i,2),obstacles(i,3),obstacles(i,4),obstacles(i,4),obstacles(i,4));
    surf(x,y,z)
    hold on
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%标记起点
plot3(P0(1),P0(2),P0(3),'r*')
hold on
%标记靶点
plot3(Ptarget(1),Ptarget(2),Ptarget(3),'g*')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
title('路径规划图')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
axis equal
axis([-50,50,-70,50,-10,120])