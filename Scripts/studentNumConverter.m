function [A, B, C, D, E, F, G, H] = studentNumConverter(studentNum)

fprintf('Student Number = %d\n', studentNum);

studentNumString = num2str(fix(studentNum));

if numel(studentNumString) ~= 8
    disp('Warning: Student number should be eight digits.')
end

A = str2num(studentNumString(1)) + 10;
B = str2num(studentNumString(2)) + 10;
C = str2num(studentNumString(3)) + 10;
D = str2num(studentNumString(4)) + 10;
E = str2num(studentNumString(5)) + 10;
F = str2num(studentNumString(6)) + 10;
G = str2num(studentNumString(7)) + 10;
H = str2num(studentNumString(8)) + 10;

end