gen3 = loadrobot("kinovaGen3");
showdetails(gen3)
gen3.DataFormat = 'column';
q_home = [0 15 180 -130 0 55 90]'*pi/180;
eeName = 'EndEffector_Link';
T_home = getTransform(gen3, q_home, eeName);

%% Replace CSV to display here!!
T = readtable('study_csv/P2_A2.csv');


%Setup some stored parameters
pos = T.Position;
len_pos = length(pos);
tracked_joints = [];
positions = splitColumn(pos);
time = T.Time - T.Time(1,1);

%Setup plotting arm
figure; set(gcf,'Visible','on');
ax = show(gen3,positions(2:8,1));
ax.CameraPositionMode='auto';
hold on;

title('Simulated Movement of the Robot');
%Animation happens here, we generally are running at ~60 FPS
framesPerSecond = 1/time(11,1);
r = rateControl(framesPerSecond);
for i = 1:11:length(positions)
    show(gen3, positions(2:8,i),'PreservePlot',false, 'FastUpdate',true);
    drawnow;
    waitfor(r);
end