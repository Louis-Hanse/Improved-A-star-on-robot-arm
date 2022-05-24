function linkT()

    mdl_puma560;
    TList = zeros(4,4,6);
    TListCache = zeros(4,4,6);
    res = zeros(4,4,6);

    for loop = 1:100
        thetaList = zeros(1,6);
        for i = 1:6
            thetaList(i) = rand()*(p560.qlim(i,2)-p560.qlim(i,1)) + p560.qlim(i,1);
        end
        for i = 1:6
            TList(:,:,i) = p560.A(i,thetaList);
            res(:,:,i) = TList(:,:,i) == TListCache(:,:,i);
            disp(i)
            disp(TList(:,:,i))
            %disp(TListCache(:,:,i))
            %disp(~res(:,:,i))
            disp('*****************')
        end
        TListCache = TList;
        disp('AAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
        pause
    end