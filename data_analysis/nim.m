kinova = loadrobot("kinovaGen3");
kinova.DataFormat = 'col';
showdetails(kinova)
T = readtable(append('study_csv/','P13_D1.csv'));
pos_joints = splitColumn(T.Position);
len = length(pos_joints);
%Simple Animation/ Playback
  time = T.Time;
            time = time - time(1);

            %Fix columns for pos/vel/efforts
            pos_joints = splitColumn(T.Position);
            vel_joints = splitColumn(T.Velocity);
            %eff_joints = splitColumn(T.Effort);

            %% SETUP STORAGE
            store_speed = [];
            store_position = [];
            store_cog = [];
            store_time = [];
            store_area = [];
            store_config = [];
            

            len = length(pos_joints);
            for i= 1:11:len
                store_time = [store_time; time(i)];
        
                config = pos_joints(2:8,i);
                store_config = [store_config; config'];
                
                
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
            
            store_distance = [0 0 0 0 0 0 0];
            for i=2:length(store_config)
                angle_difference = abs(store_config(i,:) - store_config(i-1,:));
                new_distance = store_distance(i-1,:)+ angle_difference;
                store_distance = [store_distance; new_distance];
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
            rangeX = abs(max(pdata.Xposition) - min(pdata.Xposition));
            rangeY = abs(max(pdata.Yposition) - min(pdata.Yposition));
            rangeZ = abs(max(pdata.Zposition) - min(pdata.Zposition));


            full = arclength(store_position(:,1), store_position(:,2),store_position(:,3));
            first_last = [store_position(1,:); store_position(height(store_position),:)];
            shortest = arclength(first_last(:,1), first_last(:,2), first_last(:,3));
            difference = full - shortest;
            
            %Joint Change in Position
            joint_difference = pos_joints(2:8,length(pos_joints)) - pos_joints(2:8,1);
            dis_len = length(store_distance);
            joint_7 = store_distance(dis_len,1);
            joint_6 = store_distance(dis_len,2);
            joint_5 = store_distance(dis_len,3);
            joint_4 = store_distance(dis_len,4);
            joint_3 = store_distance(dis_len,5);
            joint_2 = store_distance(dis_len,6);
            joint_1 = store_distance(dis_len,7);

% r = rateControl(60);
% for i = 1:11:len
%     show(kinova, pos_joints(2:8,i),'PreservePlot',false,'FastUpdate',true);
% 
%     drawnow;
%     waitfor(r);
% end