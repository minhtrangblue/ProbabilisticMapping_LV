function MyAlg = driveToGoal(MyAlg,Sensor)
% This function drives the robot to a specific goal defined by the user.
%
% Obstacle avoidance can be triggered in the initializer.
%
% Implementation:   Mohamed Mustafa, University of Manchester, January 2012
% -------------------------------------------------------------------------

% Turn on Localization
MyAlg = localization(MyAlg,Sensor);

% Initialize
if MyAlg.firstTime
    % User inputs:
    MyAlg.goal = [3 0]';            % define goal
    MyAlg.obstacle_avoidance = 1;   % Boolean:	0 = no obstacle avoidance
                                    %           1 = apply obstacle avoidance
end

% Extract information
goal = MyAlg.goal;                  % goal
s = MyAlg.pose_mean;                % robot pose
w_max = 2;                          % rad/sec

dist = distance_point_point(s(1:2),goal);               % distance between robot and goal
if dist>0.1
    ang_err = atan2(goal(2)-s(2), goal(1)-s(1)) - s(3);	% angle between robot pose and goal
    ang_err = wrapToPi(ang_err)/pi;	% limit angle to [-1:1]
    
    MyAlg.v = 0.5;
    MyAlg.w = w_max*ang_err;            % p-controller for angular velocity (K = 2)
else
    MyAlg.v = 0;
    MyAlg.w = 0;
    disp('Done!')
    MyAlg.isDone = 1;               % stop simulation
end

% Obstacle Avoidance
% ------------------
if MyAlg.obstacle_avoidance && strcmp(Sensor.Type,'LIDAR')
    % Avoid obstacles (simple algorithm):
    % If obstacle is in front of the robot and the goal is farther than the
    % obstacle, turn fast in the same direction of turn, otherwise keep the
    % same velocities as above.
    safeDistance = 0.5;     % in meters in front of the robot
    alpha = atan(1.5*MyAlg.robot_wb/(2*safeDistance));    % max angle to get values from the sensor.
    
    % For the sonar in NI robot, just adjust the sonar motor to cover angles in
    % <-alpha, alpha>
    ind = Sensor.theta>-alpha & Sensor.theta<alpha;
    range = min(Sensor.rho(ind));   % only care about the minimal value
    
    % consider new velocities only if the goal is farther than the closest obstacle
    if range<safeDistance && dist>range
        MyAlg.v = MyAlg.v/2;
        MyAlg.w = w_max;
    end
end

return