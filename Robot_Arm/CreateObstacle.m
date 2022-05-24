%{
    Function:   创建障碍物
    Input:      surf:   用于绘制的面信息，可用delete删除
                type:   障碍物类型，目前有'ball'/'wall'两种
                cord:   坐标，球形为球心，面无意义填0
                arg:    参数，球形为半径，面为ax+by+cz-d=0中的[a b c d]
    RetVal:     障碍物                
%}
function Obstacle = CreateObstacle(style,cord,args)

    color = [102,204,255];%球体的默认颜色,平面没搞明白咋改
    testMode = false;

    if isstruct(args)
        arg = args.arg;
        testMode = args.testMode;
    else
        arg = args;
    end

    if strcmp(style,'sphere') || style == 1 %球形障碍,arg代表半径
        
        if ~testMode
            Obstacle.surf = CreateSphere(arg,cord,color);
        else
            Obstacle.surf = CreateSphere(arg,cord,color,'test');
        end
        Obstacle.type = 'sphere';
        Obstacle.cord = cord;
        Obstacle.arg  = arg;

    elseif strcmp(style,'wall') || style == 2 %墙壁(平面)障碍,arg为长宽高,cord为原点
        Obstacle.surf = drawSurface(arg);
        Obstacle.type = 'wall';
        Obstacle.cord = 0;
        Obstacle.arg  = arg;
    else %别的需要再写
    end
end

%{

    Function:   绘制长方体
    Input:      originPoint：长方体的原点,行向量，如[0，0，0];
                cuboidSize：长方体的长宽高,行向量，如[10，20，30];
    RetVal:     绘制出的surf对象

function surface = PlotCuboid(originPoint,cuboidSize)
    %根据原点和尺寸，计算长方体的8个的顶点
    vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
    vertex=originPoint+vertexIndex.*cuboidSize;
    %定义6个平面分别对应的顶点
    facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];
    %定义8个顶点的颜色，绘制的平面颜色根据顶点的颜色进行插补
    color=[1;2;3;4;5;6;7;8];
    % patch 对图像进行绘制。
    surface = patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.8);
end

%}