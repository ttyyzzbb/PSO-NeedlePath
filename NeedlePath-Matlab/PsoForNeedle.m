function [arcsLength, Pgv,sump]=PsoForNeedle(arcsLengthUpperBound, arcsLengthLowerBound, evaluationFun, P0, Ptarget, radius)
%����Ⱥ�Ż��㷨
%�Ż�����Ŀ��λ�õ����ų�ʼ�ǶȺͺ������Ļ���
%arcsnum�����ĸ���+1, 1Ϊ��ʼ�Ƕ�
%arcsLengthUpperBound,arcsLenthLowerBound��ʼ�ǶȺͻ��������½�
%evaluationFun���㷨�����ۺ���
global wmax wk N vmin vmax Plength len M

len = size(arcsLengthUpperBound);% ֵΪ1 20  Ҳ����1��20��
len = len(1) + len(2) - 1;%һά���鳤�ȣ���ô���ɴ���������������� 1+20-1=20 lenҲ�ʹ���·���Ķ������ȿ������г���
%������ν��������еĳ���Ҳ����Ż�Ŀ�꣬����һ�������ʹ�����ת����Ҳ������

%����Ⱥ�㷨
N=2000;%��������
M = 40;%��Ⱥ���� 
Z=zeros(M, len); % ��Ⱥ����   M��len�е������
V=zeros(M, len); % �����ٶ�
P=zeros(M, len); % ���嵽������״̬(ÿ�����ӵ�����״̬)
Pv=zeros(M, 4); % ���嵽�����״̬ʱ�����Ӧֵ(ÿ�����ӵ�������Ӧֵ)
Pg=zeros(1, len); % ��Ⱥ���������״̬(��������)
Pgv = zeros(1, 4); % ����״̬����Ӧֵ(Ҳ����ȫ��������Ӧֵ)
Plength = zeros(M, 1); %һ����
Plength(:) = len;
sump_temp = zeros(1, N); % �洢ÿһ��������Ӧֵ
PgvLengthbuff = zeros(1, N);
PgvErrorbuff = zeros(1, N);

%�洢���������е�����ֵ�����ڻ�ͼ������Ҫ��ͼ��ʱ��ע�͵������Ч��
% plen = zeros(1, N);
% ptipErr = zeros(1, N);
% pdv = zeros(1, N);

Pstopv = norm(Ptarget-P0) * 1.2;  %��ֹ�������е���ȷ����

%����Ⱥ�ٶȸ��¹�ʽ����
c1=1;
% r1=rand(1);%����һ�����ֵ
c2=2;
% r2=rand(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�������ⳤ�ȿ�������
% for i=1:controlLen/2
%     arcsLengthLowerBound(1,2*i-1)=-pi;
%     arcsLengthLowerBound(1,2*i)=0;
%     arcsLengthUpperBound(1,2*i-1)=pi;
%     arcsLengthUpperBound(1,2*i)=pi*radius/2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vmax = (arcsLengthUpperBound - arcsLengthLowerBound)/10;%��������ٶ�
vmin = -vmax;%������С�ٶ�
wmax=1.5;%��ʼ��1.5
wmin=0.5;%��ʼ��0.5
wk=(wmax-wmin)/N;%Ȩ��ϵ������ϵ��
w=0.6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��ʼ������Ⱥ�㷨
%��ʼ������Ⱥ״̬
for i=1:len     % M����Ⱥ��С
    Z(:,i) = rand(M, 1)*(arcsLengthUpperBound(i) - arcsLengthLowerBound(i)) + arcsLengthLowerBound(i);
    %ÿ������ǰ���ε�ת�Ǻͻ����Ƕ����ģ�Ҳ������������ǰ�����ߵ�·������һ���ģ��ӵ��Ķο�ʼ�����ӵ�ת����0~2*pi��������0~1/4Բ��
    
end
D=Z;
%��ʼ������Ⱥ�ٶ�
for i=1:len
    V(:,i) = V(:,i)*(vmax(i) - vmin(i)) + vmin(i);%ÿ���������ÿ���������в�ͬ��Ѱ���ٶȣ�����ǰ����������������䣬�ٶ�Ҳ������
end

%��ʼ��Ⱥ���Ա���
global counter fit P_lunpan; % ��ĳ�������ڸ��¹����еõ��˸��õ���Ӧֵ����counter��0������counter��1
counter = zeros(1, M); % ���¼�����
fit = zeros(1, M); % �������ӱ�ѡ��ĸ�����
P_lunpan = zeros(1, M); % �������̶ı�ѡ��ĸ���

%���״̬
Pv(:) = 10^10;
Pgv(:) = 10^10;
[P, Pv, Pg, Pgv,sump, sumPgv] = refreshBestStates(V, Z, M, evaluationFun, P0, Ptarget, radius, P, Pv, Pg, Pgv);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��ʼ�����Ż�����Ⱥ�㷨
time=[];%�洢��������
for i = 1:N
    %��ֹ����
%     disp(['run times:' num2str(i) ',' 'Pgv:' num2str(Pgv)]);
    
%     if Pgv(2) < 0.1 && Pgv(1) < Pstopv
%         break;
%     end
    
    %���¸������ź�ȫ������
    [P, Pv, Pg, Pgv,sump, sumPgv] = refreshBestStates(V, Z, M, evaluationFun, P0, Ptarget, radius, P, Pv, Pg, Pgv);
    sump_temp(1, i) = sumPgv;
    
    %�����ٶ�
    w = wmax - wk*i;%����Ӧ����Ȩ��
    for k = 1:M
        for j = 1:Plength(k)
            r1 = rand(1);
            r2 = rand(1);
            V(k, j) = w * V(k,j) + c1*r1*(P(k, j) - Z(k, j)) + c2*r2*(Pg(j) - Z(k, j));
            if V(k, j) < vmin(j) %��ʵ���ٶȶ��ڽ����Ӱ��ϴ󣬲��ܺ��Զ��ٶ�����
                V(k,j) = vmin(j);
            end
            if V(k, j) > vmax(j)
                V(k,j) = vmax(j);
            end
        end
    end
    
    %����״̬
    for k = 1:M
        for j = 1:Plength(k)
            Z(k, j) = Z(k, j) + V(k, j);
            %״̬�����߽�
            if Z(k, j) < arcsLengthLowerBound(j)
                Z(k,j) = arcsLengthLowerBound(j);
                V(k, j) = -V(k, j);%�ٶȷ���
            end
            if Z(k, j) > arcsLengthUpperBound(j)
                Z(k,j) = arcsLengthUpperBound(j);
                V(k, j) = -V(k, j);%�ٶȷ���
            end
        end
    end
    
    
    PgvLengthbuff(i)=Pgv(1); %�洢ÿ��·����������ֵ
    PgvErrorbuff(i)=Pgv(2); %�洢ÿ��·��ƫ��(���λ��ƫ��)����ֵ
    time(i)=i;%�洢��������
    %��¼���������е�����ֵ
    plen(i) = Pgv(1);
    ptipErr(i) = Pgv(2);
    pdv(i) = Pgv(3);
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %     %����·����������ͼ
%     figure(2)
%     plot(1:N, PgvLengthbuff,'b')
% %     title('·����������ͼ')
%     xlabel('Number of iterations')
%     ylabel('Path length/mm')
% 
%     %����·��ƫ��(���λ��ƫ��)����ͼ
%     figure(3)
%     plot(1:N, PgvErrorbuff,'b')
% %     title('·��ƫ������ͼ')
%     xlabel('Number of iterations')
%     ylabel('Path error/mm')
% 
%     figure(4)
%     plot(1:N, sump_temp, 'b')
% %     title('��Ӧ������ͼ')
%     xlabel('Number of iterations')
%     ylabel('Fitness')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    arcsLength = Pg;
    i=i-1;
%     figure;
%     plot(1:i, plen(1:i));
%     axis([0 i 0 250])
%     figure
%     plot(1:i, ptipErr(1:i));
%     axis([0 i 0 250])
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function [P, Pv, Pg, Pgv,sump, sumPgv] = refreshBestStates(V, Z, M, evaluationFun, P0, Ptarget, radius, P, Pv, Pg, Pgv)
    global counter fit P_lunpan N len arcsLengthLowerBound arcsLengthUpperBound;
    % �ӵ�ǰ״̬�������ŵ�״̬
    particalLength = size(Z);%���ӳ���(������ʽ��ʾ)50 20

    particalLength = particalLength(2); %20

%     Klen = 20*Pgv(1)/norm(Ptarget - P0); %ԭ��
%     Kerror = 30; %ԭʼֵΪ30
    Klen = Pgv(1)/norm(Ptarget - P0); %ԭ��
    Kerror = 10; %ԭʼֵΪ30  �������Ȼ������
    Kdv = 10; %��ʱδʹ��
    sumPgv = Pgv(1) * Klen +  Pgv(2) * Kerror +  Pgv(3) * Klen +  Pgv(4);

    for i = 1:M %�������ӵ���ѭ��
        [pathLen, tipError, deviationValue, obstacle, len] = evaluationFun(P0, Ptarget, Z(i,:), radius);
        if len < particalLength
            Z(i,len+1:particalLength) = 0;
        end

        sump = pathLen * Klen + tipError * Kerror + deviationValue * Klen + obstacle; % ��һ�����ӵ���Ӧֵ
        sumPvi = Pv(i,1) * Klen +  Pv(i,2) * Kerror +  Pv(i,3) * Klen +  Pv(i,4); % ���������������״̬����Ӧֵ
        fit(i) = 1 / (1 + sumPvi); % �������̶�ѡ��
        if sump < sumPvi
            Pv(i, :) = [pathLen tipError deviationValue obstacle];
            P(i,:) = Z(i,:);
            counter(i) = 0;
        end
        if sump > sumPvi || sump == sumPvi
            counter(i) = counter(i) + 1;
        end
        if sump < sumPgv
            Pgv(:) = [pathLen tipError deviationValue obstacle];
            Pg = Z(i,:);
        end
        % ���counter����ֵ�����˼��޾���������
        if counter(i) > 0.5 * N * len
            for t = 1:len
                Z(i, t) = rand(1, 1)*(arcsLengthUpperBound(t) - arcsLengthLowerBound(t)) + arcsLengthLowerBound(t);    
            end
        end
    end
    
    
    % ���̶ĸ��ʼ���
    for u = 1:M
        P_lunpan(u) = fit(u) / sum(fit);
    end
    
    % ���̶�ѡ��
    for lunpan_num = 1:M
        % num = 1; % num�����̶�
        m = length(P_lunpan);
        r = rand();
        sumP_lunpan = 0;
        j = ceil(m*rand); %����1~m֮����������
        while sumP_lunpan < r
            sumP_lunpan = sumP_lunpan + P_lunpan(mod(j-1,m)+1);
            j = j+1;
        end
        %Select(i) = mod(j-1,m)+1-1;
        Select = mod(j-2,m)+1; % ѡ��ĳ������

        % ���̶�ѡ�����±�ѡ��������
        Z = LunpanUpdata(V, Z, P, Pg, Select);
        [pathLen, tipError, deviationValue, obstacle, len] = evaluationFun(P0, Ptarget, Z(Select,:), radius);
        if len < particalLength
            Z(Select,len+1:particalLength) = 0;
        end
        sump = pathLen * Klen + tipError * Kerror + deviationValue * Klen + obstacle; % ��һ�����ӵ���Ӧֵ
        sumPvi = Pv(Select,1) * Klen +  Pv(Select,2) * Kerror +  Pv(Select,3) * Klen +  Pv(Select,4); % ���������������״̬����Ӧֵ
        if sump < sumPvi
                Pv(Select, :) = [pathLen tipError deviationValue obstacle];
                P(Select,:) = Z(Select,:);
                counter(Select) = 0;
        end
        if sump > sumPvi || sump == sumPvi
            counter(Select) = counter(Select) + 1;
        end
        if sump < sumPgv
            Pgv(:) = [pathLen tipError deviationValue obstacle];
            Pg = Z(Select,:);
        end
    end
        
end

%% ���̶�ѡ�е����Ӹ���
function [Z]= LunpanUpdata(V, Z, P, Pg, Select)
    global wmax wk N vmin vmax Plength arcsLengthLowerBound arcsLengthUpperBound;
    c1 = 1;
    c2 = 2;
    %�����ٶ�
    w = wmax - wk*(2/N); % ����Ӧ����Ȩ��
    for j = 1:Plength(Select)
        r1 = rand(1);
        r2 = rand(1);
        V(Select, j) = w * V(Select,j) + c1*r1*(P(Select, j) - Z(Select, j)) + c2*r2*(Pg(j) - Z(Select, j));
        if V(Select, j) < vmin(j) % ��ʵ���ٶȶ��ڽ����Ӱ��ϴ󣬲��ܺ��Զ��ٶ�����
            V(Select,j) = vmin(j);
        end
        if V(Select, j) > vmax(j)
            V(Select,j) = vmax(j);
        end
    end
    
    %����״̬
    for j = 1:Plength(Select)
        Z(Select, j) = Z(Select, j) + V(Select, j);
        %״̬�����߽�
        if Z(Select, j) < arcsLengthLowerBound(j)
            Z(Select,j) = arcsLengthLowerBound(j);
            V(Select, j) = -V(Select, j);%�ٶȷ���
        end
        if Z(Select, j) > arcsLengthUpperBound(j)
            Z(Select,j) = arcsLengthUpperBound(j);
            V(Select, j) = -V(Select, j);%�ٶȷ���
        end
    end

end



