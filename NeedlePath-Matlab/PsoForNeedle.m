function [arcsLength, Pgv,sump]=PsoForNeedle(arcsLengthUpperBound, arcsLengthLowerBound, evaluationFun, P0, Ptarget, radius)
%粒子群优化算法
%优化到达目标位置的最优初始角度和后续弧的弧长
%arcsnum：弧的个数+1, 1为初始角度
%arcsLengthUpperBound,arcsLenthLowerBound初始角度和弧长的上下界
%evaluationFun，算法的评价函数
global wmax wk N vmin vmax Plength len M

len = size(arcsLengthUpperBound);% 值为1 20  也就是1行20列
len = len(1) + len(2) - 1;%一维数组长度，这么做可处理横向量和列向量 1+20-1=20 len也就代表路径的段数，既控制序列长度
%考虑如何将控制序列的长度也变成优化目标，既在一定情况下使针尖旋转次数也尽量少

%粒子群算法
N=2000;%迭代次数
M = 40;%种群数量 
Z=zeros(M, len); % 种群个体   M行len列的零矩阵
V=zeros(M, len); % 个体速度
P=zeros(M, len); % 个体到达的最好状态(每个粒子的最优状态)
Pv=zeros(M, 4); % 个体到达最好状态时候的适应值(每个粒子的最优适应值)
Pg=zeros(1, len); % 种群到达的最优状态(最优粒子)
Pgv = zeros(1, 4); % 最优状态的适应值(也就是全局最优适应值)
Plength = zeros(M, 1); %一列零
Plength(:) = len;
sump_temp = zeros(1, N); % 存储每一代最优适应值
PgvLengthbuff = zeros(1, N);
PgvErrorbuff = zeros(1, N);

%存储迭代过程中的最优值，用于绘图，不需要绘图的时候注释掉，提高效率
% plen = zeros(1, N);
% ptipErr = zeros(1, N);
% pdv = zeros(1, N);

Pstopv = norm(Ptarget-P0) * 1.2;  %终止条件，有点难确定，

%粒子群速度更新公式参数
c1=1;
% r1=rand(1);%产生一个随机值
c2=2;
% r2=rand(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%定义任意长度控制序列
% for i=1:controlLen/2
%     arcsLengthLowerBound(1,2*i-1)=-pi;
%     arcsLengthLowerBound(1,2*i)=0;
%     arcsLengthUpperBound(1,2*i-1)=pi;
%     arcsLengthUpperBound(1,2*i)=pi*radius/2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vmax = (arcsLengthUpperBound - arcsLengthLowerBound)/10;%粒子最大速度
vmin = -vmax;%粒子最小速度
wmax=1.5;%初始是1.5
wmin=0.5;%初始是0.5
wk=(wmax-wmin)/N;%权重系数迭代系数
w=0.6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%初始化粒子群算法
%初始化粒子群状态
for i=1:len     % M是种群大小
    Z(:,i) = rand(M, 1)*(arcsLengthUpperBound(i) - arcsLengthLowerBound(i)) + arcsLengthLowerBound(i);
    %每个粒子前三段的转角和弧长是定死的，也就是所有粒子前三段走的路径都是一样的，从第四段开始，粒子的转角是0~2*pi，弧长是0~1/4圆弧
    
end
D=Z;
%初始化粒子群速度
for i=1:len
    V(:,i) = V(:,i)*(vmax(i) - vmin(i)) + vmin(i);%每组决策量的每个变量都有不同的寻优速度，但是前三组决策量定死不变，速度也就是零
end

%初始蜂群策略变量
global counter fit P_lunpan; % 若某个粒子在更新过程中得到了更好的适应值，则counter置0，否则counter加1
counter = zeros(1, M); % 更新计数器
fit = zeros(1, M); % 计算粒子被选择的概率用
P_lunpan = zeros(1, M); % 粒子轮盘赌被选择的概率

%最好状态
Pv(:) = 10^10;
Pgv(:) = 10^10;
[P, Pv, Pg, Pgv,sump, sumPgv] = refreshBestStates(V, Z, M, evaluationFun, P0, Ptarget, radius, P, Pv, Pg, Pgv);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%开始迭代优化粒子群算法
time=[];%存储迭代次数
for i = 1:N
    %终止条件
%     disp(['run times:' num2str(i) ',' 'Pgv:' num2str(Pgv)]);
    
%     if Pgv(2) < 0.1 && Pgv(1) < Pstopv
%         break;
%     end
    
    %更新个体最优和全局最优
    [P, Pv, Pg, Pgv,sump, sumPgv] = refreshBestStates(V, Z, M, evaluationFun, P0, Ptarget, radius, P, Pv, Pg, Pgv);
    sump_temp(1, i) = sumPgv;
    
    %更新速度
    w = wmax - wk*i;%自适应惯性权重
    for k = 1:M
        for j = 1:Plength(k)
            r1 = rand(1);
            r2 = rand(1);
            V(k, j) = w * V(k,j) + c1*r1*(P(k, j) - Z(k, j)) + c2*r2*(Pg(j) - Z(k, j));
            if V(k, j) < vmin(j) %此实例速度对于结果的影响较大，不能忽略对速度限制
                V(k,j) = vmin(j);
            end
            if V(k, j) > vmax(j)
                V(k,j) = vmax(j);
            end
        end
    end
    
    %更新状态
    for k = 1:M
        for j = 1:Plength(k)
            Z(k, j) = Z(k, j) + V(k, j);
            %状态超出边界
            if Z(k, j) < arcsLengthLowerBound(j)
                Z(k,j) = arcsLengthLowerBound(j);
                V(k, j) = -V(k, j);%速度反向
            end
            if Z(k, j) > arcsLengthUpperBound(j)
                Z(k,j) = arcsLengthUpperBound(j);
                V(k, j) = -V(k, j);%速度反向
            end
        end
    end
    
    
    PgvLengthbuff(i)=Pgv(1); %存储每次路径长度收敛值
    PgvErrorbuff(i)=Pgv(2); %存储每次路径偏差(针尖位置偏差)收敛值
    time(i)=i;%存储迭代次数
    %记录迭代过程中的最优值
    plen(i) = Pgv(1);
    ptipErr(i) = Pgv(2);
    pdv(i) = Pgv(3);
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %     %绘制路径长度收敛图
%     figure(2)
%     plot(1:N, PgvLengthbuff,'b')
% %     title('路径长度收敛图')
%     xlabel('Number of iterations')
%     ylabel('Path length/mm')
% 
%     %绘制路径偏差(针尖位置偏差)收敛图
%     figure(3)
%     plot(1:N, PgvErrorbuff,'b')
% %     title('路径偏差收敛图')
%     xlabel('Number of iterations')
%     ylabel('Path error/mm')
% 
%     figure(4)
%     plot(1:N, sump_temp, 'b')
% %     title('适应度收敛图')
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
    % 从当前状态计算最优的状态
    particalLength = size(Z);%粒子长度(矩阵形式表示)50 20

    particalLength = particalLength(2); %20

%     Klen = 20*Pgv(1)/norm(Ptarget - P0); %原理？
%     Kerror = 30; %原始值为30
    Klen = Pgv(1)/norm(Ptarget - P0); %原理？
    Kerror = 10; %原始值为30  这样精度还提高了
    Kdv = 10; %暂时未使用
    sumPgv = Pgv(1) * Klen +  Pgv(2) * Kerror +  Pgv(3) * Klen +  Pgv(4);

    for i = 1:M %所有粒子迭代循环
        [pathLen, tipError, deviationValue, obstacle, len] = evaluationFun(P0, Ptarget, Z(i,:), radius);
        if len < particalLength
            Z(i,len+1:particalLength) = 0;
        end

        sump = pathLen * Klen + tipError * Kerror + deviationValue * Klen + obstacle; % 这一代粒子的适应值
        sumPvi = Pv(i,1) * Klen +  Pv(i,2) * Kerror +  Pv(i,3) * Klen +  Pv(i,4); % 这个粒子曾经最优状态的适应值
        fit(i) = 1 / (1 + sumPvi); % 用于轮盘赌选择
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
        % 检查counter的数值，到了极限就重置粒子
        if counter(i) > 0.5 * N * len
            for t = 1:len
                Z(i, t) = rand(1, 1)*(arcsLengthUpperBound(t) - arcsLengthLowerBound(t)) + arcsLengthLowerBound(t);    
            end
        end
    end
    
    
    % 轮盘赌概率计算
    for u = 1:M
        P_lunpan(u) = fit(u) / sum(fit);
    end
    
    % 轮盘赌选择
    for lunpan_num = 1:M
        % num = 1; % num次轮盘赌
        m = length(P_lunpan);
        r = rand();
        sumP_lunpan = 0;
        j = ceil(m*rand); %产生1~m之间的随机整数
        while sumP_lunpan < r
            sumP_lunpan = sumP_lunpan + P_lunpan(mod(j-1,m)+1);
            j = j+1;
        end
        %Select(i) = mod(j-1,m)+1-1;
        Select = mod(j-2,m)+1; % 选出某个粒子

        % 轮盘赌选择后更新被选到的粒子
        Z = LunpanUpdata(V, Z, P, Pg, Select);
        [pathLen, tipError, deviationValue, obstacle, len] = evaluationFun(P0, Ptarget, Z(Select,:), radius);
        if len < particalLength
            Z(Select,len+1:particalLength) = 0;
        end
        sump = pathLen * Klen + tipError * Kerror + deviationValue * Klen + obstacle; % 这一代粒子的适应值
        sumPvi = Pv(Select,1) * Klen +  Pv(Select,2) * Kerror +  Pv(Select,3) * Klen +  Pv(Select,4); % 这个粒子曾经最优状态的适应值
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

%% 轮盘赌选中的粒子更新
function [Z]= LunpanUpdata(V, Z, P, Pg, Select)
    global wmax wk N vmin vmax Plength arcsLengthLowerBound arcsLengthUpperBound;
    c1 = 1;
    c2 = 2;
    %更新速度
    w = wmax - wk*(2/N); % 自适应惯性权重
    for j = 1:Plength(Select)
        r1 = rand(1);
        r2 = rand(1);
        V(Select, j) = w * V(Select,j) + c1*r1*(P(Select, j) - Z(Select, j)) + c2*r2*(Pg(j) - Z(Select, j));
        if V(Select, j) < vmin(j) % 此实例速度对于结果的影响较大，不能忽略对速度限制
            V(Select,j) = vmin(j);
        end
        if V(Select, j) > vmax(j)
            V(Select,j) = vmax(j);
        end
    end
    
    %更新状态
    for j = 1:Plength(Select)
        Z(Select, j) = Z(Select, j) + V(Select, j);
        %状态超出边界
        if Z(Select, j) < arcsLengthLowerBound(j)
            Z(Select,j) = arcsLengthLowerBound(j);
            V(Select, j) = -V(Select, j);%速度反向
        end
        if Z(Select, j) > arcsLengthUpperBound(j)
            Z(Select,j) = arcsLengthUpperBound(j);
            V(Select, j) = -V(Select, j);%速度反向
        end
    end

end



