kinova = importrobot('gen3_7dof_vision.urdf');
kinova.DataFormat = 'row';
%Do some processing per step of data csv file 
config = randomConfiguration(kinova);

showdetails(kinova)


%Get the Jacobian of config
jacobian = geometricJacobian(kinova, config,'tool_frame');

%EE Properties (Location, Speed, Acceleration)
% jacobian*
getTransform(kinova, config, 'tool_frame')

%Center of Mass
cog = centerOfMass(kinova, config);
show(kinova,config, Visuals='on');
hold on
plot3(cog(1), cog(2), cog(3), '-o', 'MarkerSize', 10);
