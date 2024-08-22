function solution = acceleration(time,velocity)
%Solves for acceleration from time and velocity columns
len = length(time);
storage = [0];
for i= 2:len
    t2 = time(i);
    t1 = time(i-1);
    
    v2 = velocity(i);
    v1 = velocity(i-1);
    accel = (v2-v1) / (t2-t1);
    storage = [storage; accel];
end
solution = storage;
end