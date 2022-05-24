from numpy import size

totalList = []
for i in range(3):
    for j in range(3):
        for k in range(3):
            a = i-1
            b = j-1
            c = k-1
            if a*b*c == 0:
                if a==0 and b==0 and c==0:
                    continue
                else:
                    totalList.append([a,b,c])
            else:
                continue
print(totalList)
print(size(totalList)/3)
count = 0
for i in totalList:
    for j in i:
        if j != -1:
            print(' ',end='')
        print(j,end=' ')
    print(';',end=' ')
    count += 1
    if count == 6:
        count = 0
        print('')

'''
-1 -1  0; -1  0 -1; -1  0  0; -1  0  1; -1  1  0;  0 -1 -1;
 0 -1  0;  0 -1  1;  0  0 -1;  0  0  1;  0  1 -1;  0  1  0;
 0  1  1;  1 -1  0;  1  0 -1;  1  0  0;  1  0  1;  1  1  0;
'''