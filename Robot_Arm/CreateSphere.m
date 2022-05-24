%{
    Function:   创建球体
    Input:      r：     半径
                cord：  球心坐标
                color:  颜色，默认为[24, 90, 189]
    RetVal:     球体的面矩阵
%}
function ball = CreateSphere(r,cord,color,mode)
    if nargin == 2
        color = [24, 90, 189];%默认颜色
    elseif nargin == 3
        mode = 'normal';
    end
    if isnan(color)
        color = [24, 90, 189];%默认颜色
    end

    x = cord(1);
    y = cord(2);
    z = cord(3);
    if strcmp(mode,'test')
        surfNum = 12;
    else
        surfNum = 36;
    end
    [bx,by,bz] = sphere(surfNum);

    c=zeros(size(bx));%获得大小相同的矩阵
    for i=1:1:length(c(1,:))
        for j=1:1:length(c(:,1))
            c(i,j,1)=color(1)/255;
            c(i,j,2)=color(2)/255;
            c(i,j,3)=color(3)/255;
        end
    end

    if strcmp(mode,'test')
        rC = r;
        r = 0.02;
    end
    r = r*0.88;
    ball = surf(bx*r+x,by*r+y,bz*r+z,c,'EdgeColor','none');
    if strcmp(mode,'test')
        ballT = surf(bx*rC+x,by*rC+y,bz*rC+z,c);
        set(ballT,'FaceAlpha',0.1);
    end
end