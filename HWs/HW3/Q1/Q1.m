clc; clear all

% theta
syms th1 th2 th3;

% DH-table
DH = [[th1 0.2 0.4 0];
    [th2 0.25 0.3 0];
    [th3 0.15 0.15 0]];

len = length(DH);

for i=1:len-1
    if i==1
        T_final = createDH_T(DH(1, 1), DH(1, 2), DH(1, 3), DH(1, 4));
    else
        T_final = T_final * createDH_T(DH(i, 1), DH(i, 2), DH(i, 3), DH(i, 4));
    end
end

T_final = simplify(T_final)