% Stefan Tosti - 400367761

clear;

% Connect to serial port for data communication
ports = serialportlist("available");
s = serialport(ports(2), 115200);
flush(s);

% Define some useful paramaters
% Depth controls the number of steps we will be taking in making the
% measurement 3D
% the number of measurements per depth is 32, since we are taking data
% every 11.25 degrees 
% we save our data in the results matrix 
depth = 3;
num_measurements = 32; % 11.25 deg
results = zeros(0,3);

i = 0;

% we read the incoming data until all of the data has been recieved 
% I.e. the number of depth multiplied by the number of measurements per
% depth will give us the total number of measurements to take 
while (i <= depth*num_measurements-1) 

    % read incoming data. If its empty break the loop
    data = readline(s);
    if (isempty(data))
        break;
    end

    % parse the data into tmp
    % temp matrix holds indeces 2, 3 and 4
    tmp = parse(data);
    temp_matrix = tmp([2 3 4]);
    
    % display the matrix in the console
    disp(temp_matrix);

    % vertically concatinate the results with temp matrix
    % stores all of the measurements
    results = vertcat(results,temp_matrix); %#ok<AGROW> 

    % increment the iterable variable
    i=i+1;
end

% measurements_data will hold all of our coordinates 
% concatinate the matrix
% columns represent the different depths 
measurement_data = results.';

% Convert polar to Cartesian
[x, y, z] = pol2cart(measurement_data(2,:).*(pi/180), measurement_data(1,:), measurement_data(3,:));

% flip the x and z components because we want to plot along the yz plane.
% Now, we plot in the yz plane and depth affect the x coord
cartesian_data = [z; y; x]; 

% Create Plots
% plot all points on a scatter plot 
figure;
scatter3(cartesian_data(1,:), cartesian_data(2,:), cartesian_data(3,:));
hold on;

% Here, we connect the points that have a depth difference of 1 
% this code loops through each point in the matrix and connects it to its
% adjacent point 
% since the matrix is transposed, every 32 measurements is a new depth 
% 0-31 is depth 1, 32 - 63 is depth 2, etc... 
% plot 3 gets the current node (xyz) and the next node (x+1, y+1, z+1)
for d = 1:depth
    offset = (d-1)*num_measurements;
    for i = 1:num_measurements-1
        plot3(cartesian_data(1,offset+i:offset+i+1), cartesian_data(2,offset+i:offset+i+1), cartesian_data(3,offset+i:offset+i+1), 'k-');
    end
    % Last point
    plot3([cartesian_data(1,offset+1), cartesian_data(1,offset+num_measurements)], [cartesian_data(2,offset+1), cartesian_data(2,offset+num_measurements)], [cartesian_data(3,offset+1), cartesian_data(3,offset+num_measurements)], 'k-');
end

% here, we connect the points on the same depths together
% loop through each depth 
for d = 1:depth-1
    for i = 1:num_measurements
        p1 = (d-1)*num_measurements + i;
        p2 = d*num_measurements - i + num_measurements+1;
        plot3([cartesian_data(1,p1), cartesian_data(1,p2)], [cartesian_data(2,p1), cartesian_data(2,p2)], [cartesian_data(3,p1), cartesian_data(3,p2)], 'k-');
    end
end

% Output 
hold off;
title('Stefan Tosti - 400367761 - 2DX3 Final Project');
xlabel('X Depth');
ylabel('Y Width');
zlabel('Z Height');
grid on;

% parse function 
function parsed_data = parse(n)
    % Read incoming string and display for debugging
    incoming_string = n;
    disp("INCOMING STRING: "+ incoming_string);

    % seperate the variable by commas 
    parsed_variables = sscanf(incoming_string, '%f,%f,%f,%f,%f');
    
    % from Keil, this wsa the order of data coming in
    % angle is converted from number of steps to degree by multiplying the
    % total step count by the degrees per single step 
    check_bit = parsed_variables(1);
    distance = parsed_variables(2);
    angle = parsed_variables(3)*11.25/16; 
    depth = parsed_variables(4);
    spad_num = parsed_variables(5);
    
    % return parsed data
    parsed_data = [check_bit, distance angle depth, spad_num];
end