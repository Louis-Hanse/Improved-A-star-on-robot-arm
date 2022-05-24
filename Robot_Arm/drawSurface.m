function retSurf = drawSurface(args,mode,range)
    %   ax+by+cz=d,args = [a b c d]
    if nargin <= 2
        range = [-1 1];
    end
    if nargin == 1
        mode = 'eq';
    end

    minVal = range(1);
    maxVal = range(2);
    gap = (maxVal - minVal)/50;

    if strcmp(mode,'eq')
        if      all(args(1:3) == [1 0 0])
            mode = 'vertical_X';
        elseif  all(args(1:3) == [0 1 0])
            mode = 'vertical_Y';
        elseif  all(args(1:3) == [0 0 1])
            mode = 'vertical_Z';
        else 
            mode = 'eq';
        end
    end

    %%  输入为方程
    if strcmp(mode,'eq')
        %   x/z*X + y/z*Y - d/z = -Z
        x = args(1); y = args(2); z = args(3); d = args(4); 
        arrX = minVal:gap:maxVal;
        arrY = arrX;
        [X,Y] = meshgrid(arrX,arrY);
        Z = -1*(x/z*X + y/z*Y - d/z);
    %%  输入为垂直于某个轴的平面方程
    elseif strcmp(mode,'vertical_X')
        % x=d,args = [1 0 0 d]
        arrZ = minVal:gap:maxVal;
        arrY = arrZ;
        [Z,Y] = meshgrid(arrZ,arrY);
        X = args(4)*ones(size(Z));
    elseif strcmp(mode,'vertical_Y')
        % y=d,args = [0 1 0 d]
        arrX = minVal:gap:maxVal;
        arrZ = arrX;
        [X,Z] = meshgrid(arrX,arrZ);
        Y = args(4)*ones(size(X));
    elseif strcmp(mode,'vertical_Z')
        % z=d,args = [0 0 1 d]
        arrX = minVal:gap:maxVal;
        arrY = arrX;
        [X,Y] = meshgrid(arrX,arrY);
        Z = args(4)*ones(size(X));
    %%  输入为节点
    elseif strcmp(mode,'node')
        disp('WIP');
    else
        disp('Error')
    end

    c = ones(size(Z));
    retSurf = surf(X,Y,Z,c);

end