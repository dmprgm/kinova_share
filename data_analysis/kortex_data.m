%Get csvs
trials = dir("study_csv");
%finalData = table('VariableNames',[''])
groupData = array2table(zeros(0,18));
groupData.Properties.VariableNames = {'id','condition', 'Yaw','Pitch', 'Roll','TwistX','TwistY', 'TwistZ','AvgVelcoity', 'MaxVelocity', 'AvgAccel', 'MaxAccel', 'AvgArea', 'COGZ', 'PathLengthDifference', 'RangeX', 'RangeY', 'RangeZ'};
all_data = [];
for i=1:length(trials)
    trial = trials(i).name;
    if contains(trial,'csv')
        %append('study_csv/',trial)
        new = split(trial, '.');
        part_cond = new{1,1}
        final = split(part_cond,'_');
        id = final{1,1};
        cond = final{2,1};
        T = readtable(append('study_csv/',trial));

        if ~isempty(T)
            time = T.Time;
            time = time - time(1);

            %Fix columns for pos/vel/efforts
            pos_joints = splitColumn(T.Position);
            vel_joints = splitColumn(T.Velocity);
            %eff_joints = splitColumn(T.Effort);

            kinova = loadrobot("kinovaGen3");
            kinova.DataFormat = 'col';
            %% SETUP STORAGE
            store_speed = [];
            store_position = [];
            store_cog = [];
            store_time = [];
            store_area = [];


            len = length(pos_joints);
            for i= 1:11:len
                store_time = [store_time; time(i)];

                config = pos_joints(2:8,i);
                %Calculate position
                homo = getTransform(kinova, config, 'EndEffector_Link');
                ee_pos = tform2trvec(homo);
                rotm = tform2rotm(homo);
                ypr = quadricFit.rot2ypr(rotm);

                store_position = [store_position; ee_pos, ypr];

                %Calculate Speed
                jacobian = geometricJacobian(kinova, config,'EndEffector_Link');
                speed = (jacobian*vel_joints(2:8, i))';
                store_speed = [store_speed; speed];

                % Determine COG
                cog = centerOfMass(kinova, config)';
                store_cog = [store_cog; cog];

                % Determine AREA / VOLUME CALC
                homo_fa = getTransform(kinova, pos_joints(2:8,i), 'ForeArm_Link');
                position_fa = tform2trvec(homo_fa);

                homo_ha = getTransform(kinova, pos_joints(2:8,i), 'HalfArm1_Link');
                position_ha = tform2trvec(homo_ha);
                %Base Pos
                homo_ha = getTransform(kinova, pos_joints(2:8,1), 'Shoulder_Link');
                position_base = tform2trvec(homo_ha);
                consolidate = [ee_pos;  position_fa; position_ha; position_base];
                area = area3D(consolidate(:,1),consolidate(:,2),consolidate(:,3));
                store_area = [store_area; area];
            end
            time = store_time;
            pdata = table(time);

            %Cartesian positions
            pdata.Xposition = store_position(:,1);
            pdata.Yposition = store_position(:,2);
            pdata.Zposition = store_position(:,3);
            pdata.yaw = store_position(:,4);
            pdata.pitch = store_position(:,5);
            pdata.roll = store_position(:,6);

            %Linear Velocities
            pdata.speedX = store_speed(:,1);
            pdata.speedY = store_speed(:,2);
            pdata.speedZ = store_speed(:,3);
            pdata.speedNorm = normal(pdata.speedX,pdata.speedY,pdata.speedZ);

            %Angular Velcoities
            pdata.twistX = store_speed(:,4);
            pdata.twistY = store_speed(:,5);
            pdata.twistZ = store_speed(:,6);

            %COG
            pdata.cogX = store_cog(:,1);
            pdata.cogY = store_cog(:,2);
            pdata.cogZ = store_cog(:,3);

            %Solve for acceleration
            accelerationX = acceleration(time,pdata.speedX);
            accelerationY = acceleration(time,pdata.speedY);
            accelerationZ = acceleration(time,pdata.speedZ);

            %Set Accceleration
            pdata.accelerationX = accelerationX;
            pdata.accelerationY = accelerationY;
            pdata.accelerationZ = accelerationZ;
            pdata.accelNorm = normal(accelerationX,accelerationY,accelerationZ);

            %Set Area
            pdata.area = store_area;
            rangeX = max(pdata.Xposition) - min(pdata.Xposition);
            rangeY = max(pdata.Yposition) - min(pdata.Yposition);
            rangeZ = max(pdata.Zposition) - min(pdata.Zposition);


            full = arclength(store_position(:,1), store_position(:,2),store_position(:,3));
            first_last = [store_position(1,:); store_position(height(store_position),:)];
            shortest = arclength(first_last(:,1), first_last(:,2), first_last(:,3));
            difference = full - shortest;
            cell = {id, cond, mean(pdata.yaw), mean(pdata.pitch), mean(pdata.roll), mean(pdata.twistX), mean(pdata.twistY), mean(pdata.twistZ), mean(pdata.speedNorm), max(pdata.speedNorm),mean(pdata.accelNorm),max(pdata.accelNorm),mean(pdata.area),mean(pdata.cogZ), difference, rangeX, rangeY, rangeZ};
            groupData = [groupData;cell];

        end






    end

end

writetable(groupData,'robot_data.csv')





%ee_pos = [store_position(1,1), store_position(1,2),store_position(1,3)];
%FA Pos
%homo_fa = getTransform(kinova, pos_joints(2:8,1), 'ForeArm_Link');
%position_fa = tform2trvec(homo);

%Half Arm Link 1 Pos

%Base Pos
%homo_ha = getTransform(kinova, pos_joints(2:8,1), 'HalfArm1_Link');
%position_ha = tform2trvec(homo_ha);


%homo_ha = getTransform(kinova, pos_joints(2:8,1), 'Shoulder_Link');
%position_base = tform2trvec(homo_ha);

%consolidate = [ee_pos; position_fa; position_ha; position_base ];
%show(kinova, pos_joints(2:8,1),'PreservePlot',false,'FastUpdate',true);


%hold on
%h = fill3(consolidate(:,1),consolidate(:,2),consolidate(:,3),'r','FaceAlpha',0.2);

% %Simple Animation/ Playback
% r = rateControl(60);
% for i = 1:11:len
%     show(kinova, pos_joints(2:8,i),'PreservePlot',false,'FastUpdate',true);
%
%     ee_pos = [store_position(i,1), store_position(i,2),store_position(i,3)];
%     %FA Pos
%     homo_fa = getTransform(kinova, pos_joints(2:8,i), 'ForeArm_Link');
%     position_fa = tform2trvec(homo_fa);
%
%     %Half Arm Link 1 Pos
%     homo_ha = getTransform(kinova, pos_joints(2:8,i), 'HalfArm1_Link');
%     position_ha = tform2trvec(homo_ha);
%     %Base Pos
%     homo_ha = getTransform(kinova, pos_joints(2:8,1), 'Shoulder_Link');
%     position_base = tform2trvec(homo_ha);
%     consolidate = [ ee_pos;  position_fa; position_ha; position_base];
%     set(h, 'XData', consolidate(:,1), 'YData', consolidate(:,2), 'ZData', consolidate(:,3));
%
%     drawnow;
%     waitfor(r);
% end

% hold on
% plot3(store_position(:,1), store_position(:,2), store_position(:,3));
% plot3(store_cog(:,1), store_cog(:,2),store_cog(:,3))






