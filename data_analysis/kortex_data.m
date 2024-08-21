%Get csvs
trials = dir("study_csv");

for i=1:length(trials)
    trial = trials(i);
end

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

pdata = table(time);


store_speed = [];
store_position = [];
store_cog = [];
%For Every Configuration In the Set
len = length(pos_joints);

%EE Pos
homo_ee = getTransform(kinova, config, 'EndEffector_Link');
position_ee = tform2trvec(homo);
%FA Pos
homo_fa = getTransform(kinova, config, 'ForeArm_Link');
position_fa = tform2trvec(homo);

%Half Arm Link 1 Pos
homo_ha = getTransform(kinova, config, 'HalfArm1_Link');
position_ha = tform2trvec(homo_ha);
%Base Pos
position_base = [0 0 0];


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


ee_pos = [store_position(1,1), store_position(1,2),store_position(1,3)];
%FA Pos
homo_fa = getTransform(kinova, pos_joints(2:8,1), 'ForeArm_Link');
position_fa = tform2trvec(homo);

%Half Arm Link 1 Pos

%Base Pos
homo_ha = getTransform(kinova, pos_joints(2:8,1), 'HalfArm1_Link');
position_ha = tform2trvec(homo_ha);


homo_ha = getTransform(kinova, pos_joints(2:8,1), 'Shoulder_Link');
position_base = tform2trvec(homo_ha);

consolidate = [position_base; position_ha; position_fa;  ee_pos];
show(kinova, pos_joints(2:8,1),'PreservePlot',false,'FastUpdate',true);
hold on
h = fill3(consolidate(:,1),consolidate(:,2),consolidate(:,3),'r','FaceAlpha',0.2);

%Simple Animation/ Playback
r = rateControl(60);
for i = 1:11:len
    show(kinova, pos_joints(2:8,i),'PreservePlot',false,'FastUpdate',true);
    
    ee_pos = [store_position(i,1), store_position(i,2),store_position(i,3)];
    %FA Pos
    homo_fa = getTransform(kinova, pos_joints(2:8,i), 'HalfArm2_Link');
    position_fa = tform2trvec(homo);

    %Half Arm Link 1 Pos
    homo_ha = getTransform(kinova, pos_joints(2:8,i), 'HalfArm1_Link');
    position_ha = tform2trvec(homo_ha);
    %Base Pos
    homo_ha = getTransform(kinova, pos_joints(2:8,1), 'Shoulder_Link');
    position_base = tform2trvec(homo_ha);
    consolidate = [position_base; position_ha; position_fa;  ee_pos];
    set(h, 'XData', consolidate(:,1), 'YData', consolidate(:,2), 'ZData', consolidate(:,3));

    drawnow;
    waitfor(r);
end

hold on
plot3(store_position(:,1), store_position(:,2), store_position(:,3));
plot3(store_cog(:,1), store_cog(:,2),store_cog(:,3))
arc_length = arclength(store_position(:,1), store_position(:,2), store_position(:,3));


mean_cog