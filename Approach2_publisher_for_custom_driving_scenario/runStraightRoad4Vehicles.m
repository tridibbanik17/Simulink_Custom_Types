%% runSensorFusionCustomScenario.m
% Live sensor fusion + ROS 2 odometry publishing using generateSensorData.m

clearvars
close all
clc

scenarioName="StraightRoad4Vehicles";
f = str2func(scenarioName);
[~, scenario, sensors] = f();

%% If sensors is a single sensor object, iscell(sensors) is false, so it wraps it into a 1x1 cell array
if ~iscell(sensors)
    sensors = {sensors};
end
%%

egoCar = scenario.Actors(1);
numSensors = numel(sensors);

%% Tracker
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationThreshold', [4 5]);

positionSelector = [1 0 0 0; 0 0 1 0];
velocitySelector = [0 1 0 0; 0 0 0 1];

BEP = createDemoDisplay(egoCar, sensors);

%% ROS 2 publisher
% Create ROS 2 publisher once, before the while-loop
% Node Name: /ego_state_node
node = ros2node("/ego_state_node");
% TopicName: /ego_odom
% MessageType: nav_msgs/Odometry
odomPub = ros2publisher(node, "/ego_odom", "nav_msgs/Odometry");

%% Run simulation
toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent)
    time = scenario.SimulationTime;

    detectionClusters = {};
    isValidTime = false(1, numSensors);

    for i = 1:numSensors
        sensor = sensors{i};
        % If a sensor is an INS sensor (inertial navigation sensor object),
        % it gets detections using just time.
        % Otherwise, it gets actor poses with targetPoses(...) and 
        % calls the sensor with poses and time.
        % isValidTime(i) stores whether that sensor produced valid data at that time step.
        if isa(sensor, 'insSensor')
            [sensorDets, numValidDets, isValidTime(i)] = sensor(time);
        else
            poses = targetPoses(scenario, sensor.SensorIndex);
            [sensorDets, numValidDets, isValidTime(i)] = sensor(poses, time);
        end

        if numValidDets
            for j = 1:numValidDets
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end
                
                % check that the detection has a measurement vector long enough to slice.
                if isprop(sensorDets{j}, 'Measurement') && numel(sensorDets{j}.Measurement) >= 5
                    sensorDets{j}.Measurement = sensorDets{j}.Measurement([1 2 4 5]);
                    sensorDets{j}.MeasurementNoise = ...
                        sensorDets{j}.MeasurementNoise([1 2 4 5], [1 2 4 5]);
                end
            end

            detectionClusters = [detectionClusters; sensorDets];
        end
    end

    if any(isValidTime)
        confirmedTracks = updateTracks(tracker, detectionClusters, time);
        updateBEP(BEP, egoCar, detectionClusters, confirmedTracks, positionSelector, velocitySelector, sensors);
    end

    if egoCar.Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end

    %% 
    % read the vehicle's current position and velocity from the scenario
    egoPos = egoCar.Position;   % [x y z]
    egoVel = egoCar.Velocity;   % [vx vy vz]

    % creates a blank odometry message with the right type.
    msg = ros2message(odomPub);
    % frame_id = "map" means
    % pose is expressed in the global map frame.
    msg.header.frame_id = 'map';
    % child_frame_id = "base_link" means
    % the tracked body frame is the vehicle frame.
    msg.child_frame_id = 'base_link';

    % pose has 2 structs: position and orientation
    % Position: x, y, z
    % Orientation: roll, pitch, yaw stored as a quaternion

    % A quaternion is a 4-number representation of 3D orientation.
    % It stores rotation as x, y, z, w in ROS message terms.
    % It avoids gimbal lock, which can happen with Euler angles like roll/pitch/yaw.
    msg.pose.pose.position.x = egoPos(1);
    msg.pose.pose.position.y = egoPos(2);
    msg.pose.pose.position.z = egoPos(3);

    % pose.pose.orientation.w = 1 sets an identity quaternion, 
    % meaning no rotation is being reported.
    msg.pose.pose.orientation.w = 1;  

    % twist has 2 structs: linear and angular
    % Linear velocity: vx, vy, vz
    % Angular velocity: wx, wy, wz
    msg.twist.twist.linear.x = egoVel(1);
    msg.twist.twist.linear.y = egoVel(2);
    msg.twist.twist.linear.z = egoVel(3);

    % no angular velocity is set, so it stays 0.

    % publish the message
    send(odomPub, msg);
    %%
end

%% Supporting Functions
function filter = initSimDemoFilter(detection)
H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
filter = trackingKF('MotionModel', '2D Constant Velocity', ...
    'State', H' * detection.Measurement, ...
    'MeasurementModel', H, ...
    'StateCovariance', H' * detection.MeasurementNoise * H, ...
    'MeasurementNoise', detection.MeasurementNoise);
end

function BEP = createDemoDisplay(egoCar, sensors)
hFigure = figure('Position', [0, 0, 1200, 640], ...
    'Name', 'Sensor Fusion with Synthetic Data Example');
movegui(hFigure, [0 -1]);

hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
hCarPlot = axes(hCarViewPanel);
chasePlot(egoCar, 'Parent', hCarPlot);

hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
hCarPlot = axes(hTopViewPanel);
chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);

hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');
hBEVPlot = axes(hBEVPanel);

frontBackLim = 60;
BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);

%% visualizing each sensor's coverage area
for i = 1:numel(sensors)
    sensor = sensors{i};
    [mountPos, range, yaw, fov, color] = getCoverageInfo(sensor);

    if isempty(mountPos)
        continue
    end

    cap = coverageAreaPlotter(BEP, 'FaceColor', color, 'EdgeColor', color);
    plotCoverageArea(cap, mountPos, range, yaw, fov);
end
%%

detectionPlotter(BEP, 'DisplayName', 'vision', 'MarkerEdgeColor', 'blue', 'Marker', '^');
detectionPlotter(BEP, 'DisplayName', 'radar', 'MarkerEdgeColor', 'red');
laneMarkingPlotter(BEP, 'DisplayName', 'lane markings');
trackPlotter(BEP, 'DisplayName', 'track', 'HistoryDepth', 10);

axis(BEP.Parent, 'equal');
xlim(BEP.Parent, [-frontBackLim frontBackLim]);
ylim(BEP.Parent, [-40 40]);
outlinePlotter(BEP, 'Tag', 'Ground truth');
end

% extract plotting settings for one sensor so the BEV (Bird's-Eye View) display can draw its coverage area.
function [mountPos, range, yaw, fov, color] = getCoverageInfo(sensor)
mountPos = [];
range = [];
yaw = 0;
fov = 45;
color = 'blue';

if isa(sensor, 'drivingRadarDataGenerator')
    mountPos = sensor.MountingLocation(1:2);
    range = sensor.RangeLimits(2);
    yaw = sensor.MountingAngles(1);
    fov = sensor.FieldOfView(1);
    color = 'red';
elseif isa(sensor, 'visionDetectionGenerator')
    mountPos = sensor.SensorLocation(1:2);
    range = sensor.MaxRange;
    if isprop(sensor, 'Yaw')
        yaw = sensor.Yaw;
    end
    if isprop(sensor, 'FieldOfView')
        fov = sensor.FieldOfView(1);
    else
        fov = 45;
    end
    color = 'blue';
end
end

function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel, sensors)
[lmv, lmf] = laneMarkingVertices(egoCar);
plotLaneMarking(findPlotter(BEP, 'DisplayName', 'lane markings'), lmv, lmf);

[position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
plotOutline(findPlotter(BEP, 'Tag', 'Ground truth'), position, yaw, length, width, ...
    'OriginOffset', originOffset, 'Color', color);

[bPosition, bYaw, bLength, bWidth, bOriginOffset, bColor, numBarrierSegments] = targetOutlines(egoCar, 'Barriers');
plotBarrierOutline(findPlotter(BEP, 'Tag', 'Ground truth'), numBarrierSegments, ...
    bPosition, bYaw, bLength, bWidth, 'OriginOffset', bOriginOffset, 'Color', bColor);

N = numel(detections);
detPos = zeros(N, 2);
isRadar = false(N, 1);

% 2 lookup arrays to understand which detection came from which sensor
sensorIndexList = zeros(1, numel(sensors));
sensorIsRadar = false(1, numel(sensors));

for k = 1:numel(sensors)
    sensorIndexList(k) = sensors{k}.SensorIndex;
    sensorIsRadar(k) = isa(sensors{k}, 'drivingRadarDataGenerator');
end

for i = 1:N
    detPos(i,:) = detections{i}.Measurement(1:2)';
    % mark whether a sensor is a radar.
    sensorIdx = detections{i}.SensorIndex;
    k = find(sensorIndexList == sensorIdx, 1, 'first');
    if ~isempty(k)
        isRadar(i) = sensorIsRadar(k);
    end
end

if any(~isRadar)
    plotDetection(findPlotter(BEP, 'DisplayName', 'vision'), detPos(~isRadar,:));
end
if any(isRadar)
    plotDetection(findPlotter(BEP, 'DisplayName', 'radar'), detPos(isRadar,:));
end

isNotBarrier = arrayfun(@(t) t.ObjectClassID, confirmedTracks) > 0;
confirmedTracks = confirmedTracks(isNotBarrier);

trackIDs = {confirmedTracks.TrackID};
labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
[tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
tracksVel = getTrackVelocities(confirmedTracks, vsel);
plotTrack(findPlotter(BEP, 'DisplayName', 'track'), tracksPos, tracksVel, tracksCov, labels);
end
