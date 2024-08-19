%Data Processing
T = readtable('joint_states_data.csv');
%T.Time;
%T.Position;
vel = T.Velocity;
pos = T.Position;
len_pos = length(pos);
tracked_joints = [];
for i=1:len_pos
    cell = pos(i);
    string = cell{1};
    corrected = value(2:end-2);
    numeric = sscanf(corrected,'%f');
    num_corrected = numeric(1:end);
    tracked_joints = [tracked_joints, num_corrected];
end


kinova = loadrobot("kinovaGen3");
kinova.DataFormat = 'col';
%Do some processing per step of data csv file 
config = randomConfiguration(kinova);

showdetails(kinova)

%Average Time
%time_allotted = 0.1*length(T.Time)

%Get the Jacobian of config
jacobian = geometricJacobian(kinova, config,'EndEffector_Link')

%EE Properties (Location, Speed, Acceleration)
% jacobian*
homo = getTransform(kinova, config, 'EndEffector_Link');

%End Effector Position
trvec = tform2trvec(homo);

velocity = jacobian*q_star;


%Center of Mass
cog = centerOfMass(kinova, config);
show(kinova,config, Visuals='on');
hold on
plot3(cog(1), cog(2), cog(3), '-o', 'MarkerSize', 10);
