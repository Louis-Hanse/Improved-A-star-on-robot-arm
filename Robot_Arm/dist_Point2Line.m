%%  计算点到直线距离
function output = dist_Point2Line(args)
    line.eq = args.line(2,:) - args.line(1,:);
    line.cord = args.line;
    cord = args.cord;
    
    if ~isfield(args,'mode')
        mode = 0;
    else
        mode = args.mode;
    end

    t = (line.eq*(cord-line.cord(1,:))') / sum(line.eq.^2);
    c = t*line.eq + line.cord(1,:);
    output.dist = norm(cord - c);

    if t<=0 || t>=1
        output.isOutOfLine = true;
    else
        output.isOutOfLine = false;
    end

    if mode
        limMat = [args.line;cord;c];
        lim = [
            min(limMat(:,1))-1 max(limMat(:,1))+1;
            min(limMat(:,2))-1 max(limMat(:,2))+1;
            min(limMat(:,3))-1 max(limMat(:,3))+1;
            ];
        lineT = [
            (lim(1,:) - line.cord(1,1))/line.eq(1) (lim(2,:) - line.cord(1,2))/line.eq(2) (lim(3,:) - line.cord(1,3))/line.eq(3)
        ];
        while true
            sizeT = size(lineT);
            sizeT = sizeT(2);
            flag = 0;
            for i = 1:sizeT
                if isnan(lineT(i)) || isinf(lineT(i))
                    lineT(i) = [];
                    flag = 1;
                    break;
                end
            end
            if ~flag
                break;
            end
        end
        lineC = [
            min(min(lineT))*line.eq + line.cord(1,:);
            max(max(lineT))*line.eq + line.cord(1,:);
        ];
        figure(2);
        clf(2);
        axis([lim(1,:) lim(2,:) lim(3,:)])
        hold on
        plot3(cord(1),cord(2),cord(3),'ro','MarkerSize',10);
        plot3(c(1),c(2),c(3),'bo','MarkerSize',10);
        plot3(lineC(:,1),lineC(:,2),lineC(:,3),'b','lineWidth',2);
        cLine = [c;cord];
        plot3(cLine(:,1),cLine(:,2),cLine(:,3),'r--','lineWidth',2);
        view(3);
        hold off
    end
end
