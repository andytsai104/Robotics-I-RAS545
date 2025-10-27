clc; clear all

% theta
th1 = 0;
th2 = 0;
th3 = 0;

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

fprintf('The position of E.E.(Px, Py, Pz): (%.2f, %.2f, %.2f)', T_final(1, end), T_final(2, end), T_final(3, end));
T_final