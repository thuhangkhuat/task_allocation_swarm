% Number of robots
nRobots = 7;
% Define goal positions
goalPositions = [1, 1; 1, 2; 1, 3; 2, 2; 3, 1; 3, 2; 3, 3];  % Example "H" shape
% goalPositions = [         
%     0, 0;
%     0, 1;
%     0, 3;
%     1, 3;
%     1, 2;
%     0, 2;
%     0.5,0.5;
%     1, 0;
%     2, 0;
%     2, 1;
%     2, 2;
%     2, 3;
%     3, 3;
%     3, 2;
%     3, 1;
%     3, 0;
%      4, 0;
%     4, 1;
%     4, 2;
%     4, 3;
%     5, 3;
%     5, 2;
%     5, 1.5;
%     4.5, 1.5;
%     5, 0;
%      6, 0;
%     6, 1;
%     6, 2;
%     6, 3;
%     7, 3;
%     7, 2;
%     7, 1;
%     7, 0;
%      8, 3;
%     10, 3;
%     9, 3;
%     9, 2;
%     9, 1;
%     9, 0];            % Example "ROBOT" shape

%Example circle shape
% numEdges = 20;
% circumRadius = 8;
% theta = linspace(0, 2*pi-(2*pi)/numEdges, numEdges);
% x = circumRadius * cos(theta);
% y = circumRadius * sin(theta);
% goalPositions = [x', y'];

goalPositions= goalPositions*2;
nGoals = size(goalPositions, 1);
% Check if number of robots matches number of goal positions
if nRobots ~= nGoals
    error('Number of robots must be equal to the number of goal positions.');
end

% Generate random initial positions for the robots
 initialPositions = rand(nRobots, 2);  % Random (x, y) positions for each robot
 % initialPositions= initialPositions*2;
 
% Calculate distance matrix between initial and goal positions
distances = pdist2(initialPositions, goalPositions);  % Euclidean distances

% Solve assignment problem using Hungarian algorithm
assignment = munkres(distances);
for i = 1:nRobots
    reorderedGoalPositions(i, :) = goalPositions(assignment(i), :);
end

% Display original and reordered goal positions
disp('Original Goal Positions:');
disp(goalPositions);
disp('Reordered Goal Positions:');
disp(reorderedGoalPositions);

%% start locations
xs = initialPositions(:,1);
ys = initialPositions(:,2);
%% goal locations
xg = reorderedGoalPositions(:,1);
yg = reorderedGoalPositions(:,2);
%% init
v_ref = 1;
C = [0 0];
%% param
S = 3; %sensing range
R = 0.2; %radius of robot
wg = 0.6; %weight of detect goal
wo = 0.3; %weight of detect collision
wr = 0.01; %weight of random walk
position_accuracy = 0.01;
dstar = 1;
%% Parameters related to kinematic model
error_theta_max = deg2rad(45);
v_max = 2;
Kp_omega = 1.5;
omega_max = 0.5*pi; 
%% process
figure(1); 
t = 1;
dT = 0.2;
t_max = 250;
n = nRobots; %number of robot
% create matrix
X = cell(n, 1);
Y = cell(n, 1);
theta = zeros(1,n);
for i = 1:n
    X{i} = zeros(1,t_max);
    Y{i} = zeros(1,t_max);
end
for i = 1:n
    X{i}(1) = xs(i);
    Y{i}(1) = ys(i);
end
while t < t_max
    for i = 1:n
        if (norm([xg(i) yg(i)] - [xs(i) ys(i)]) > position_accuracy )
            % Calculate detect goal
            if norm([xg(i) yg(i)]-[xs(i) ys(i)]) <= dstar
                v_goal =  ([xg(i) yg(i)]-[xs(i) ys(i)]);
            else 
                v_goal = ([xg(i) yg(i)]-[xs(i) ys(i)])/norm([xg(i) yg(i)]-[xs(i) ys(i)]);
            end
            % Calculate detect collision
             v_obs = 0;
            for j = 1:n
                dist = sqrt((xs(i) - xs(j))^2 + (ys(i) - ys(j))^2);
                if dist >= S || dist == 0
                    C = [0 0];
                else
                    C = (S-dist)/(S-R)*([xs(i) ys(i)]-[xs(j) ys(j)])/sqrt((xs(i) - xs(j))^2 + (ys(i) - ys(j))^2);
                end
                v_obs = (v_obs+C);
            end
            % Calculate random walk
            v_rand = [rand() rand()];
            % Calculate final velocity
            vel = wg * v_goal + wo * v_obs + wr * v_rand;
            xs(i) = xs(i) + vel(1) * dT;
            ys(i) = ys(i) + vel(2) * dT;
        end
            X{i}(t) = xs(i);
            Y{i}(t) = ys(i);
        
    end
    % Archive and plot it
    t = t + 1;
           
end   
for i = 1:t_max
    daspect([1 1 1]); 
        title('Task allocation swarm robot');
        xlabel('m') 
        ylabel('m') 
        xlim([-2 8]);  ylim([-2 8]);
        box on; hold on;
        for l = 1:n
           plot(xg(l) , yg(l), 'o',"MarkerEdgeColor", 'black',"MarkerFaceColor",'blue' );
           plot(X{l}(i) , Y{l}(i), 'o',"MarkerEdgeColor", 'black',"MarkerFaceColor",'y' ); 
        end
        pause(dT);
       clf;
end
t = t*dT;
disp("Travel time: " + t);