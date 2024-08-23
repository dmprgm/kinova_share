function output = normal(var1, var2, var3)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
len = length(var1);
storage = [];
for i=1:len
    result = sqrt(var1(i)^2 + var2(i)^2 + var3(i)^2);
    storage = [storage; result];
end
output = storage;
end