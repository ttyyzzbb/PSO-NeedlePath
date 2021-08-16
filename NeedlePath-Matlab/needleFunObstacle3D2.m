function [pathLength, tipError, deviationValue, obstacleValue, nonZeroLength] =needleFunObstacle3D2(P0, Ptarget, arcsLength, radius)
%三维规划 适应值函数

%初始在y-z平面内
%初始切向量
p0c = [0 0 1];
%初始圆心位置
O0 = [0 radius 0];

%障碍物设定
obstacles = setObstacles3D();%[0,5,40,8]
olen = size(obstacles);% 1 4
olen = olen(1);% 1 代表有几个障碍物？
InObstacleValue = 10^6;
obstacleValue=0;

%考虑旋转点切线与起点到目标向量的夹角，夹角越大，路径越偏离目标位置
%切向量与目标向量夹角，cos\gamma = corss(1,2)/|2|
%1-cos\gamma越大，夹角越大
p0pt = Ptarget - P0;%起点到终点的向量
p0pt = p0pt/norm(p0pt); %norm求得向量的模，此处也就是求单位向量

deviationValue = 0; %偏差值
KDV = 20;%系数

len = size(arcsLength);
len = len(1) + len(2) -1;%一维数组长度，这么做可处理横向量和列向量
len = len/2;

thetas = zeros(1, len);%此时是1行20列的零矩阵
points = zeros(len+1,3);%此时是21行3列的零矩阵
points(1,:) = P0;%此时是[0 0 0]，第一行是出发点的坐标

pic = p0c;%初始切向量
pi2c = pic;
Oi=O0;%初始圆心坐标
Oi2=O0;%用于存储将Oi旋转后的值

isStop = 0;
nonZeroLength = 2*len;

for i=1:len %每一段圆弧判断与障碍的距离
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
    %计算穿刺后的端点位置
    st = sin(thetas(i));
    ct = cos(thetas(i));
    points(i+1,:) = ct*points(i,:) + (1-ct)*Oi2 + radius*st*pic;
    if points(i+1, 3) > Ptarget(3)%超过设定的终点z轴坐标则停止
        isStop = 1;
        nonZeroLength = 2*i;
    end
    %计算端点处切向量
    pi2c=Oi2*(1-ct)+points(i+1,:)*ct-points(i,:);
    %向量单位化
    pi2c = pi2c/norm(pi2c);
    
    %计算偏离值
    dotpp = dot(pi2c, p0pt);%dot是求点积 x1*x2 + y1*y2 + z1*z2  p0pt是起点到终点的向量
    if dotpp > 0
        deviationValue = deviationValue + KDV * (1/dotpp);
    else
        deviationValue = deviationValue + 10^6;
    end
    
    %计算障碍物约束
    for j=1:olen
       if obstacles(j,4) > 0 %障碍物的半径是否大于0
           %向量，i点到障碍物圆心
           po=obstacles(j,1:3)-points(i,:);
           %向量，i+1点到障碍物圆心
           p1o=obstacles(j,1:3)-points(i+1,:);
           
           %计算i和i+1点是否和障碍物相交，即到圆心的距离<r
           npo = norm(po);
           np1o = norm(p1o);
           if npo<obstacles(j,4) || np1o<obstacles(j,4)%和障碍物半径作比较
               obstacleValue = obstacleValue + InObstacleValue;
               isStop = 1;
               break;
           end
           if npo < np1o
                minL = npo;
           else
                minL = np1o;
           end
           %计算障碍物是否在弧内部，pp1与po  p1p与p1的内积>0，否则直接跳过,保留一定余量
           if dot(pic,po) >0 && dot(pi2c,p1o) < 0
               %弧线圆心到障碍物圆心的向量
               vecO = obstacles(j,1:3) - Oi2;
               vecOnorm = norm(vecO);
               %
               vecN = cross(pic, pi2c);
               alpha = asin(abs(dot(vecO, vecN)/vecOnorm/norm(vecN)));
               %障碍物圆心到弧线的最短距离
               minL = sqrt(vecOnorm^2+radius^2-2*vecOnorm*radius*cos(alpha)) -  obstacles(j,4);
               if minL < 0
                   obstacleValue = obstacleValue + InObstacleValue;
                   isStop = 1;
                   break;
               end
           end
           
           obstacleValue = obstacleValue + 1/minL;%将距离的倒数作为适应值函数，越靠近障碍物，适应值函数越大
       end
    end
    if isStop == 1
        break;
    end
    
    %更新临时变量
    pic = pi2c;
    Oi=Oi2;    
end

%计算适应值
fx=Ptarget(1)-points(len+1,1);
fy=Ptarget(2)-points(len+1,2);
fz=Ptarget(3)-points(len+1,3);


pathLength = sum(thetas)*radius;
tipError = (fx^2+fy^2+fz^2);
deviationValue;
obstacleValue;


end