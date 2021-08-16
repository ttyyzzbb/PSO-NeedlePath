function [pathLength, tipError, deviationValue, obstacleValue, nonZeroLength] =needleFunObstacle3D2(P0, Ptarget, arcsLength, radius)
%��ά�滮 ��Ӧֵ����

%��ʼ��y-zƽ����
%��ʼ������
p0c = [0 0 1];
%��ʼԲ��λ��
O0 = [0 radius 0];

%�ϰ����趨
obstacles = setObstacles3D();%[0,5,40,8]
olen = size(obstacles);% 1 4
olen = olen(1);% 1 �����м����ϰ��
InObstacleValue = 10^6;
obstacleValue=0;

%������ת����������㵽Ŀ�������ļнǣ��н�Խ��·��Խƫ��Ŀ��λ��
%��������Ŀ�������нǣ�cos\gamma = corss(1,2)/|2|
%1-cos\gammaԽ�󣬼н�Խ��
p0pt = Ptarget - P0;%��㵽�յ������
p0pt = p0pt/norm(p0pt); %norm���������ģ���˴�Ҳ������λ����

deviationValue = 0; %ƫ��ֵ
KDV = 20;%ϵ��

len = size(arcsLength);
len = len(1) + len(2) -1;%һά���鳤�ȣ���ô���ɴ����������������
len = len/2;

thetas = zeros(1, len);%��ʱ��1��20�е������
points = zeros(len+1,3);%��ʱ��21��3�е������
points(1,:) = P0;%��ʱ��[0 0 0]����һ���ǳ����������

pic = p0c;%��ʼ������
pi2c = pic;
Oi=O0;%��ʼԲ������
Oi2=O0;%���ڴ洢��Oi��ת���ֵ

isStop = 0;
nonZeroLength = 2*len;

for i=1:len %ÿһ��Բ���ж����ϰ��ľ���
    thetas(i) = arcsLength(2*i)/radius;%����/�뾶=����
    alpha = arcsLength(2*i-1);%��arcLength�о������ȡ��ת�Ƕ�
    %������ת���Բ��λ��
    sa=sin(alpha);%������ֵ����sa�﷽�����ʹ��
    ca=cos(alpha);
    %��Բ��ת��Ϊ����
    Oi=Oi-points(i,:);%��ʼԲ�������������ʽ
    %������������ת
    Oi2(1) = Oi(3)*(pic(2)*sa-pic(1)*pic(3)*(ca-1)) -  Oi(2)*(pic(3)*sa+pic(1)*pic(2)*(ca-1)) + Oi(1)*(ca-pic(1)^2*(ca-1));
    Oi2(2) = Oi(1)*(pic(3)*sa-pic(1)*pic(2)*(ca-1)) -  Oi(3)*(pic(1)*sa+pic(2)*pic(3)*(ca-1)) + Oi(2)*(ca-pic(2)^2*(ca-1));
    Oi2(3) = Oi(2)*(pic(1)*sa-pic(2)*pic(3)*(ca-1)) -  Oi(1)*(pic(2)*sa+pic(1)*pic(3)*(ca-1)) + Oi(3)*(ca-pic(3)^2*(ca-1));
    %�������л�ȡ��ת���Բ��
    Oi2 = Oi2 +  points(i,:);
    %���㴩�̺�Ķ˵�λ��
    st = sin(thetas(i));
    ct = cos(thetas(i));
    points(i+1,:) = ct*points(i,:) + (1-ct)*Oi2 + radius*st*pic;
    if points(i+1, 3) > Ptarget(3)%�����趨���յ�z��������ֹͣ
        isStop = 1;
        nonZeroLength = 2*i;
    end
    %����˵㴦������
    pi2c=Oi2*(1-ct)+points(i+1,:)*ct-points(i,:);
    %������λ��
    pi2c = pi2c/norm(pi2c);
    
    %����ƫ��ֵ
    dotpp = dot(pi2c, p0pt);%dot������ x1*x2 + y1*y2 + z1*z2  p0pt����㵽�յ������
    if dotpp > 0
        deviationValue = deviationValue + KDV * (1/dotpp);
    else
        deviationValue = deviationValue + 10^6;
    end
    
    %�����ϰ���Լ��
    for j=1:olen
       if obstacles(j,4) > 0 %�ϰ���İ뾶�Ƿ����0
           %������i�㵽�ϰ���Բ��
           po=obstacles(j,1:3)-points(i,:);
           %������i+1�㵽�ϰ���Բ��
           p1o=obstacles(j,1:3)-points(i+1,:);
           
           %����i��i+1���Ƿ���ϰ����ཻ������Բ�ĵľ���<r
           npo = norm(po);
           np1o = norm(p1o);
           if npo<obstacles(j,4) || np1o<obstacles(j,4)%���ϰ���뾶���Ƚ�
               obstacleValue = obstacleValue + InObstacleValue;
               isStop = 1;
               break;
           end
           if npo < np1o
                minL = npo;
           else
                minL = np1o;
           end
           %�����ϰ����Ƿ��ڻ��ڲ���pp1��po  p1p��p1���ڻ�>0������ֱ������,����һ������
           if dot(pic,po) >0 && dot(pi2c,p1o) < 0
               %����Բ�ĵ��ϰ���Բ�ĵ�����
               vecO = obstacles(j,1:3) - Oi2;
               vecOnorm = norm(vecO);
               %
               vecN = cross(pic, pi2c);
               alpha = asin(abs(dot(vecO, vecN)/vecOnorm/norm(vecN)));
               %�ϰ���Բ�ĵ����ߵ���̾���
               minL = sqrt(vecOnorm^2+radius^2-2*vecOnorm*radius*cos(alpha)) -  obstacles(j,4);
               if minL < 0
                   obstacleValue = obstacleValue + InObstacleValue;
                   isStop = 1;
                   break;
               end
           end
           
           obstacleValue = obstacleValue + 1/minL;%������ĵ�����Ϊ��Ӧֵ������Խ�����ϰ����Ӧֵ����Խ��
       end
    end
    if isStop == 1
        break;
    end
    
    %������ʱ����
    pic = pi2c;
    Oi=Oi2;    
end

%������Ӧֵ
fx=Ptarget(1)-points(len+1,1);
fy=Ptarget(2)-points(len+1,2);
fz=Ptarget(3)-points(len+1,3);


pathLength = sum(thetas)*radius;
tipError = (fx^2+fy^2+fz^2);
deviationValue;
obstacleValue;


end