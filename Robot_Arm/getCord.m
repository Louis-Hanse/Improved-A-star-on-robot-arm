%%  获取机械臂上4个关键位置的坐标
%{
    使用的puma560最后三个关节在同一个点上，所以无需计算后两个点的坐标
    Input:  robotArm:   机械臂的SerialLink
            q：         需要求的各个关节的角度
    retVal: 每个节点的坐标
%}
function cordList = getCord(robotArm,q)
    if nargin == 1
        q = robotArm.getpos();
    end
    cordList = zeros(3,4);
    for i = 1:4
        cordList(1:3,i:i) = robotArm.A(1:i,q).t;
    end
end