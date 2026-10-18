%% runSensorFusionCustomScenario.m
% Live sensor fusion + ROS 2 odometry publishing using generateSensorData.m

clearvars
close all
clc

scenarioScriptName="CarsInOppositeDirection";
f = str2func(scenarioScriptName);
[~, scenario, sensors] = f();

%%
if isempty(sensors)
    sensors = {};
% If sensors is a single sensor object, iscell(sensors) is false, so it wraps it into a 1x1 cell array
elseif ~iscell(sensors)
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

frameTemplate = struct('Time', [], ...
                  'ObjectDetections', {{}}, ...
                  'LaneDetections', {{}}, ...
                  'PointClouds', {{}}, ...
                  'INSMeasurements', {{}});
frameLog = frameTemplate([]);
logIdx = 0;

while advance(scenario) && ishghandle(BEP.Parent)
    time = scenario.SimulationTime;

    objectDetections = {};
    laneDetections = {};
    ptClouds = {};
    insMeas = {};

    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        frame = readSensorData(sensor, scenario, egoCar, time);

        objectDetections = [objectDetections; frame.ObjectDetections]; %#ok<AGROW>
        laneDetections = [laneDetections; frame.LaneDetections]; %#ok<AGROW>
        ptClouds = [ptClouds; frame.PointClouds]; %#ok<AGROW>
        insMeas = [insMeas; frame.INSMeasurements]; %#ok<AGROW>
    end
    
    logIdx = logIdx + 1;
    frameLog(logIdx) = struct('Time', time, ...
                              'ObjectDetections', {objectDetections}, ...
                              'LaneDetections', {laneDetections}, ...
                              'PointClouds', {ptClouds}, ...
                              'INSMeasurements', {insMeas}); 

    detectionClusters = normalizeObjectDetections(objectDetections);

    if ~isempty(detectionClusters)
        confirmedTracks = updateTracks(tracker, detectionClusters, time);
        updateBEP(BEP, egoCar, detectionClusters, confirmedTracks, positionSelector, velocitySelector);
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

frameLog = frameLog(1:logIdx);

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

    if isempty(mountPos) || isempty(range)
        continue
    end

    cap = coverageAreaPlotter(BEP, 'FaceColor', color, 'EdgeColor', color);
    plotCoverageArea(cap, mountPos, range, yaw, fov);
end
%%

detectionPlotter(BEP, 'DisplayName', 'detections', 'MarkerEdgeColor', 'blue', 'Marker', '^');
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
    if isprop(sensor, 'MountingLocation')
        mountPos = sensor.MountingLocation(1:2);
    end
    if isprop(sensor, 'RangeLimits')
        range = sensor.RangeLimits(2);
    end
    if isprop(sensor, 'MountingAngles')
        yaw = sensor.MountingAngles(1);
    end
    if isprop(sensor, 'FieldOfView')
        fov = sensor.FieldOfView(1);
    end
    color = 'red';

elseif isa(sensor, 'visionDetectionGenerator')
    if isprop(sensor, 'SensorLocation')
        mountPos = sensor.SensorLocation(1:2);
    end
    if isprop(sensor, 'MaxRange')
        range = sensor.MaxRange;
    end
    if isprop(sensor, 'Yaw')
        yaw = sensor.Yaw;
    end
    if isprop(sensor, 'FieldOfView')
        fov = sensor.FieldOfView(1);
    end
    color = 'blue';

elseif isa(sensor, 'ultrasonicDetectionGenerator')
    if isprop(sensor, 'MountingLocation')
        mountPos = sensor.MountingLocation(1:2);
    end
    if isprop(sensor, 'RangeLimits')
        range = sensor.RangeLimits(2);
    elseif isprop(sensor, 'MaxRange')
        range = sensor.MaxRange;
    end
    if isprop(sensor, 'MountingAngles')
        yaw = sensor.MountingAngles(1);
    end
    if isprop(sensor, 'FieldOfView')
        fov = sensor.FieldOfView(1);
    end
    color = 'magenta';

elseif isa(sensor, 'lidarPointCloudGenerator')
    if isprop(sensor, 'SensorLocation')
        mountPos = sensor.SensorLocation(1:2);
    end
    if isprop(sensor, 'RangeLimits')
        range = sensor.RangeLimits(2);
    elseif isprop(sensor, 'MaxRange')
        range = sensor.MaxRange;
    end
    color = 'green';

end
end

function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel)
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

for i = 1:N
    z = detections{i}.Measurement(:);
    detPos(i,:) = z(1:2).';
end

if N > 0
    plotDetection(findPlotter(BEP, 'DisplayName', 'detections'), detPos);
end

if isempty(confirmedTracks)
    return 
end

isNotBarrier = arrayfun(@(t) t.ObjectClassID, confirmedTracks) > 0;
confirmedTracks = confirmedTracks(isNotBarrier);

if isempty(confirmedTracks)
    return 
end

trackIDs = {confirmedTracks.TrackID};
labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
[tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
tracksVel = getTrackVelocities(confirmedTracks, vsel);
plotTrack(findPlotter(BEP, 'DisplayName', 'track'), tracksPos, tracksVel, tracksCov, labels);
end

function frame = readSensorData(sensor, scenario, egoCar, time)
frame = struct('ObjectDetections', {{}}, ...
               'LaneDetections', {{}}, ...
               'PointClouds', {{}}, ...
               'INSMeasurements', {{}}, ...
               'IsValid', false);

if isa(sensor, "insSensor")
    poses = actorPoses(scenario);
    egoPose = poses([poses.ActorID] == egoCar.ActorID);

    if isempty(egoPose)
        return
    end

    actorState = struct( ...
        'Position', egoPose.Position, ...
        'Velocity', egoPose.Velocity, ...
        'Orientation', [egoPose.Roll egoPose.Pitch egoPose.Yaw]);

    frame.INSMeasurements = {sensor(actorState, time)};
    frame.IsValid = true;
    return
end

if isa(sensor, "lidarPointCloudGenerator")
    [ptCloud, isValid] = sensor();
    frame.PointClouds = {ptCloud};
    frame.IsValid = isValid;
    return
end

if ~isprop(sensor, 'SensorIndex')
    return
end

poses = targetPoses(scenario, sensor.SensorIndex);

isValid = false;

if isa(sensor, "visionDetectionGenerator")
    outputType = string(sensor.DetectorOutput);

    if outputType == "Objects only"
        [objDets, numObjects, isValid] = sensor(poses, time);
        frame.ObjectDetections = objDets(1:numObjects);

    elseif outputType == "Lanes only"
        maxLaneDetectionRange = min(500, sensor.MaxRange);
        lanes = laneBoundaries(egoCar, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
        [laneDets, ~, isValid] = sensor(lanes, time);
        frame.LaneDetections = laneDets;

    elseif outputType == "Lanes and objects"
        maxLaneDetectionRange = min(500, sensor.MaxRange);
        lanes = laneBoundaries(egoCar, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
        [objDets, numObjects, isValidObj, laneDets, ~, isValidLane] = sensor(poses, lanes, time);
        frame.ObjectDetections = objDets(1:numObjects);
        frame.LaneDetections = laneDets;
        isValid = isValidObj || isValidLane;

    elseif outputType == "Lanes with occlusion"
        maxLaneDetectionRange = min(500, sensor.MaxRange);
        lanes = laneBoundaries(egoCar, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
        [laneDets, ~, isValid] = sensor(poses, lanes, time);
        frame.LaneDetections = laneDets;
    end

    frame.IsValid = isValid;
    return
end

if isa(sensor, "drivingRadarDataGenerator")
    [objDets, numObjects, isValid] = sensor(poses, time);
    frame.ObjectDetections = objDets(1:numObjects);
    frame.IsValid = isValid;
    return
end

if isa(sensor, "ultrasonicDetectionGenerator")
    [objDets, isValid] = sensor(poses, time);
    frame.ObjectDetections = objDets;
    frame.IsValid = isValid;
    return
end
end

function detectionsOut = normalizeObjectDetections(detectionsIn)
detectionsOut = {};

for i = 1:numel(detectionsIn)
    d = detectionsIn{i};

    if ~isprop(d, "Measurement") || numel(d.Measurement) < 2
        continue
    end

    z = d.Measurement(:);

    if numel(z) >= 4
        z = z(1:4);   % [x y vx vy]
        if isprop(d, "MeasurementNoise") && size(d.MeasurementNoise,1) >= 4 && size(d.MeasurementNoise,2) >= 4
            R = d.MeasurementNoise(1:4, 1:4);
        else
            R = eye(4);
        end
    else
        z = [z(1); z(2); 0; 0];
        R = diag([10 10 100 100]);
    end

    d.Measurement = z;
    d.MeasurementNoise = R;
    detectionsOut{end+1,1} = d; %#ok<AGROW>
end
end
