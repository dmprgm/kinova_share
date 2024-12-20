%Get csvs
trials = dir("E:\csvs");

kinova = loadrobot("kinovaGen3", Gravity=[0 0 -9.81]);
kinova.DataFormat = 'col';
showdetails(kinova)
%finalData = table('VariableNames',[''])
groupData = array2table(zeros(0,17));
groupData.Properties.VariableNames = {'id','condition',...
    'AvgVelcoity', 'MaxVelocity', 'AvgAccel', 'MaxAccel', 'AvgArea', ...
    'COGZ', 'PathLengthDifference',...
    'RangeX', 'RangeY', 'RangeZ','TimeElapsed', 'NrPks','AvgNormForce','ManipLinear','ManipCombined'};
all_data = [];
[wksp,cfgs] = generateRobotWorkspace(kinova,{});
for i=1:length(trials)
    trial = trials(i).name;
    if contains(trial,'csv')
        %append('study_csv/',trial)
        new = split(trial, '.');
        part_cond = new{1,1}
        final = split(part_cond,'_');
        id = final{1,1};
        cond = final{2,1}(1);
        T = readtable(append('E:\csvs\',trial));

        if ~isempty(T)
            time = T.Time;
            time = time - time(1);

            %Fix columns for pos/vel/efforts
            pos_joints = splitColumn(T.Position);
            vel_joints = splitColumn(T.Velocity);
            eff_joints = splitColumn(T.Effort);

            %% SETUP STORAGE
            store_speed = [];
            store_position = [];
            store_cog = [];
            store_time = [];
            store_area = [];
            store_config = [];
            store_torque = [];
            store_force = [];
            %For Manipulation Index
            store_manipLinear = [];
            store_manipCombined = [];
            

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

                consolidate = [ee_pos;  position_fa; position_ha; position_base; ee_pos(1) ee_pos(2) 0];
                area = area3D(consolidate(:,1),consolidate(:,2),consolidate(:,3));
                store_area = [store_area; area];
                gtau = gravityTorque(kinova,config);
                corrected_efforts = (eff_joints(2:8,i) - gtau)';
                store_torque = [store_torque; corrected_efforts];
                force = jacobian*corrected_efforts';
                store_force = [store_force; force'];

                mIndexCombined = manipulabilityIndex(kinova,config',MotionComponent='combined');
                mIndexLinear = manipulabilityIndex(kinova,config',MotionComponent='linear');
                store_manipCombined = [store_manipCombined;mIndexCombined];
                store_manipLinear = [store_manipLinear;mIndexLinear];

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
            %pdata.yaw = store_position(:,4);
            %pdata.pitch = store_position(:,5);
            %pdata.roll = store_position(:,6);
            

            pdata.forceX = store_force(:,4);
            pdata.forceY = store_force(:,5);
            pdata.forceZ = store_force(:,6);

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
            [pks,locs] = findpeaks(pdata.accelNorm);
            NrPks = numel(pks);
            %Set Area
            pdata.area = store_area;
            rangeX = abs(max(pdata.Xposition) - min(pdata.Xposition));
            rangeY = abs(max(pdata.Yposition) - min(pdata.Yposition));
            rangeZ = abs(max(pdata.Zposition) - min(pdata.Zposition));


            %Forces?
            test_force  = normal(store_force(:,4),store_force(:,5),store_force(:,6));
            test_torque = normal(store_force(:,1),store_force(:,2),store_force(:,3));

            full = arclength(store_position(:,1), store_position(:,2),store_position(:,3));
            first_last = [store_position(1,:); store_position(height(store_position),:)];
            shortest = arclength(first_last(:,1), first_last(:,2), first_last(:,3));
            difference = full - shortest;
            

            %Manips
            pdata.manipL = store_manipLinear;
            pdata.manipC = store_manipCombined;


            %Joint Change in Position

            cell = {id, cond, mean(pdata.speedNorm), max(pdata.speedNorm),...
                mean(pdata.accelNorm),max(pdata.accelNorm),mean(pdata.area),...
                mean(pdata.cogZ), difference, rangeX, rangeY, rangeZ,...
                time(length(time),1),NrPks,...
                mean(test_torque), mean(pdata.manipL), mean(pdata.manipC)};
            groupData = [groupData;cell];
            final_table = table(time,pdata.Xposition,pdata.Yposition,pdata.Zposition,pdata.cogX,pdata.cogY,pdata.cogZ,pdata.area,pdata.forceX,pdata.forceY,pdata.forceZ,pdata.manipL, pdata.manipC);
            final_table.Properties.VariableNames = {'time','x', 'y','z','cogx','cogy','cogz','area','forceX','forceY','forceZ','manipL','manipC'};
            %part_cond
            name = "OUTPUTS/CompleteTrajectories/trajectory_" + part_cond+  ".csv";
            
            writetable(final_table,name);


        end






    end

end

writetable(groupData,'robot_data.csv')





