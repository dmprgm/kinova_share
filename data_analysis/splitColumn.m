function array = splitColumn(col)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
array = [];
len_col = length(col);
for i=1:len_col
    cell = col(i);
    string = cell{1};
    corrected = string(2:end-2);
    numeric = sscanf(corrected,'%f');
    num_corrected = numeric(1:end);
    array = [array, num_corrected];
end
end