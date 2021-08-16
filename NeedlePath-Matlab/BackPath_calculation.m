
% 回退路径规划
function [arcsLength, BackPath_length, BackPath_error] = BackPath_calculation(P0, pic, Ptarget, O0)

    radius = 50;

    %控制序列  三维优化，旋转角度，弧长，旋转角度，弧长……
    %全程自由
    global arcsLengthLowerBound arcsLengthUpperBound
    %arcsLengthLowerBound = [-pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0];
    %arcsLengthUpperBound = [pi, pi*radius/2, pi, pi*radius/2, pi, pi*radius/2, pi, pi*radius/2, pi, pi*radius/2, pi, pi*radius/2, pi, pi*radius/2, pi, pi*radius/2, pi, pi*radius/2, pi, pi*radius/2];

    %tic;%启动秒表计时器

    [arcsLength, Pgv,sump]= BackPathPsoForNeedle(arcsLengthUpperBound, arcsLengthLowerBound, @BackPathNeedleFunObstacle3D2, P0, pic, O0, Ptarget, radius); 
    %toc %显示已用时间

    %Path3D(P0, Ptarget, arcsLength, radius);

    %arcsLength
    %Pgv
    BackPath_length = Pgv(1);
    BackPath_error = Pgv(2);
    % TIME

end
