%Data Processing
T = readtable('joint_states_data.csv');
%T.Time;
%T.Position;
vel = T.Velocity;
pos = T.Position;
time = T.Time;
time = time - time(1);

%Fix columns for pos/vel/efforts
pos_joints = splitColumn(T.Position);
vel_joints = splitColumn(T.Velocity);
%eff_joints = splitColumn(T.Efforts);

kinova = loadrobot("kinovaGen3");
kinova.DataFormat = 'col';
%Do some processing per step of data csv file 
config = randomConfiguration(kinova);

showdetails(kinova)

%Average Time
%time_allotted = 0.1*length(T.Time)

%Get the Jacobian of config
jacobian = geometricJacobian(kinova, config,'EndEffector_Link');

%EE Properties (Location, Speed, Acceleration)
% jacobian*
homo = getTransform(kinova, config, 'EndEffector_Link');

%End Effector Position
trvec = tform2trvec(homo);

%velocity = jacobian*q_star;





%Center of Mass
%
%show(kinova,config, Visuals='on');
%hold on
%plot3(cog(1), cog(2), cog(3), '-o', 'MarkerSize', 10);

pdata = table(time);


store_speed = [];
store_position = [];
store_cog = [];
%For Every Configuration In the Set
len = length(pos_joints);
for i= 1:len
    config = pos_joints(2:8,i);
    %Calculate position
    homo = getTransform(kinova, config, 'EndEffector_Link');
    position = tform2trvec(homo);
    store_position = [store_position; position];
    
    %Calculate Speed
    jacobian = geometricJacobian(kinova, config,'EndEffector_Link');
    speed = (jacobian*vel_joints(2:8, i))';
    store_speed = [store_speed; speed];
    
    %Determine COG
    cog = centerOfMass(kinova, config)';
    store_cog = [store_cog; cog];
end

%Cartesian positions
pdata.Xposition = store_position(:,1);
pdata.Yposition = store_position(:,2);
pdata.Zposition = store_position(:,3);

%Linear Velocities
pdata.speedX = store_speed(:,1);
pdata.speedY = store_speed(:,2);
pdata.speedZ = store_speed(:,3);

%Angular Velcoities
pdata.twistX = store_speed(:,4);
pdata.twistY = store_speed(:,5);
pdata.twistZ = store_speed(:,6);

%COG
pdata.cogX = store_cog(:,1);
pdata.cogY = store_cog(:,2);
pdata.cogZ = store_cog(:,3);

%Simple Animation/ Playback
r = rateControl(60);
for i = 1:11:len
    show(kinova, pos_joints(2:8,i),'PreservePlot',false,'FastUpdate',true);
    drawnow;
    waitfor(r);
end
