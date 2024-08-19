gen3 = loadrobot("kinovaGen3");
showdetails(gen3)
gen3.DataFormat = 'column';
q_home = [0 15 180 -130 0 55 90]'*pi/180;
eeName = 'EndEffector_Link';
T_home = getTransform(gen3, q_home, eeName);

T = readtable('joint_states_data.csv');
%T.Time;
pos = T.Position;
len_pos = length(pos)
tracked_joints = [];
for i=1:len_pos
    cell = pos(i);
    string = cell{1};
    corrected = value(2:end-2);
    numeric = sscanf(corrected,'%f');
    num_corrected = numeric(1:end-1);
    tracked_joints = [tracked_joints, num_corrected];
end


%show(gen3,q_home);
%axis auto;
%view([60,10]);

figure; set(gcf,'Visible','on');
ax = show(gen3,tracked_joints(:,1));
ax.CameraPositionMode='auto';
hold on;

% Plot waypoints
% plot3(points(:,1),points(:,2),points(:,3),'-g','LineWidth',2);
% axis auto;
% view([60,10]);
% grid('minor');
% hold on;

title('Simulated Movement of the Robot');
% Animate
framesPerSecond = 20;
r = robotics.Rate(framesPerSecond);
for i = 1:len_pos
    show(gen3, tracked_joints(:,i),'PreservePlot',false);
    drawnow;
    waitfor(r);
end