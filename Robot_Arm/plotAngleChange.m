%plotAngleChange
function plotAngleChange(pathList)
    Angle1 = [];
    Angle2 = [];
    Angle3 = [];
    for i = pathList
        Angle1 = [Angle1 i{1}.pose(1)/pi*180];
        Angle2 = [Angle2 i{1}.pose(2)/pi*180];
        Angle3 = [Angle3 i{1}.pose(3)/pi*180];
    end
    sizeA = max(size(Angle1));
    hold on
    plot(1:sizeA, Angle1, 'r-', 'linewidth', 7);
    plot(1:sizeA, Angle2, 'k-', 'linewidth', 7);
    plot(1:sizeA, Angle3, 'b-', 'linewidth', 7);
end