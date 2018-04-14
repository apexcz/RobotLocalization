function [ bo ] = isIntersected(A, B, C, D )
%% IsInterseted
% two segments, AB and CD
% return 1 if they have a intersection else return 0
AC = C - A;
AD = D - A;
BC = C - B;
BD = D - B;
CA = A - C;
CB = B - C;
DA = A - D;
DB = B - D;

reqA = vectorProduct(AC, AD) * vectorProduct(BC, BD);
reqB = vectorProduct(CA, CB) * vectorProduct(DA, DB);

bo = and(reqA <= 0, reqB <= 0);

end

