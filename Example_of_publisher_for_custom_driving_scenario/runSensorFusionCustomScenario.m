%% runSensorFusionCustomScenario.m
% Live sensor fusion + ROS 2 odometry publishing for a custom driving scenario

%%
%% runSensorFusionCustomScenario.m
% Change only these parts when you create a new custom scenario:
%
% 1) Scenario function call:
%    [scenario, egoCar] = createDrivingScenario();
%    -> replace createDrivingScenario with your new scenario function name.
%
% 2) Sensor count:
%    sensors = cell(8,1);
%    -> change 8 if you add/remove sensors.
%
% 3) Sensor mounting:
%    All lines using egoCar.Wheelbase, FrontOverhang, RearOverhang, Width
%    depend on the ego vehicle geometry in the scenario file.
%
% 4) Sensor type handling:
%    If you use only radar or only vision, update:
%    - the sensor creation block
%    - the profile assignment loop
%    - detection filtering in updateBEP
%
% 5) Loop limits:
%    for i = 1:8
%    -> change to your actual sensor count.
%
% 6) Vision/radar split:
%    if detections{i}.SensorIndex > 6
%    -> change 6 to the last radar sensor index.
%
% 7) ROS 2 publishing:
%    Keep this block if you still want ego position/velocity on /ego_odom.
%    Update topic names only if your ROS setup changes.
%
% 8) Scenario helper:
%    createDrivingScenario.m must:
%    - return [scenario, egoCar]
%    - set scenario.StopTime
%    - define a moving ego vehicle
%%

clearvars
close all
clc

[scenario, egoCar] = createDrivingScenario();

%% Define Radar and Vision Sensors
sensors = cell(8,1);

% Front radar
sensors{1} = drivingRadarDataGenerator('SensorIndex', 1, 'RangeLimits', [0 174], ...
    'MountingLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0, 0.2], ...
    'FieldOfView', [20, 5]);

% Rear radar
sensors{2} = drivingRadarDataGenerator('SensorIndex', 2, 'MountingAngles', [180 0 0], ...
    'MountingLocation', [-egoCar.RearOverhang, 0, 0.2], ...
    'RangeLimits', [0 30], 'FieldOfView', [20, 5]);

% Rear-left radar
sensors{3} = drivingRadarDataGenerator('SensorIndex', 3, 'MountingAngles', [120 0 0], ...
    'MountingLocation', [0, egoCar.Width/2, 0.2], 'RangeLimits', [0 30], ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], ...
    'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Rear-right radar
sensors{4} = drivingRadarDataGenerator('SensorIndex', 4, 'MountingAngles', [-120 0 0], ...
    'MountingLocation', [0, -egoCar.Width/2, 0.2], 'RangeLimits', [0 30], ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], ...
    'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front-left radar
sensors{5} = drivingRadarDataGenerator('SensorIndex', 5, 'MountingAngles', [60 0 0], ...
    'MountingLocation', [egoCar.Wheelbase, egoCar.Width/2, 0.2], 'RangeLimits', [0 30], ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], ...
    'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front-right radar
sensors{6} = drivingRadarDataGenerator('SensorIndex', 6, 'MountingAngles', [-60 0 0], ...
    'MountingLocation', [egoCar.Wheelbase, -egoCar.Width/2, 0.2], 'RangeLimits', [0 30], ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], ...
    'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front camera
sensors{7} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], 'Height', 1.1);

% Rear camera
sensors{8} = visionDetectionGenerator('SensorIndex', 8, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.2*egoCar.Wheelbase 0], 'Height', 1.1, 'Yaw', 180);

profiles = actorProfiles(scenario);
for m = 1:numel(sensors)
    if isa(sensors{m}, 'drivingRadarDataGenerator')
        sensors{m}.Profiles = profiles;
    else
        sensors{m}.ActorProfiles = profiles;
    end
end

%% Tracker
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationThreshold', [4 5]);

positionSelector = [1 0 0 0; 0 0 1 0];
velocitySelector = [0 1 0 0; 0 0 0 1];

BEP = createDemoDisplay(egoCar, sensors);

%% ROS 2 publisher
node = ros2node("/ego_state_node");
odomPub = ros2publisher(node, "/ego_odom", "nav_msgs/Odometry");

%% Run simulation
toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent)
    time = scenario.SimulationTime;
    ta = targetPoses(egoCar);

    detectionClusters = {};
    isValidTime = false(1, 8);

    for i = 1:8
        [sensorDets, numValidDets, isValidTime(i)] = sensors{i}(ta, time);

        if numValidDets
            for j = 1:numValidDets
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end

                sensorDets{j}.Measurement = sensorDets{j}.Measurement([1 2 4 5]);
                sensorDets{j}.MeasurementNoise = sensorDets{j}.MeasurementNoise([1 2 4 5], [1 2 4 5]);
            end

            detectionClusters = [detectionClusters; sensorDets]; %#ok<AGROW>
        end
    end

    if any(isValidTime)
        confirmedTracks = updateTracks(tracker, detectionClusters, time);
        updateBEP(BEP, egoCar, detectionClusters, confirmedTracks, positionSelector, velocitySelector);
    end

    if ta(1).Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end

    % Publish ego state
    egoPos = egoCar.Position;
    egoVel = egoCar.Velocity;

    msg = ros2message(odomPub);
    msg.header.frame_id = 'map';
    msg.child_frame_id = 'base_link';

    msg.pose.pose.position.x = egoPos(1);
    msg.pose.pose.position.y = egoPos(2);
    msg.pose.pose.position.z = egoPos(3);

    % Identity orientation (replace with yaw quaternion if needed)
    msg.pose.pose.orientation.w = 1;

    msg.twist.twist.linear.x = egoVel(1);
    msg.twist.twist.linear.y = egoVel(2);
    msg.twist.twist.linear.z = egoVel(3);

    send(odomPub, msg);
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

for i = 1:6
    cap = coverageAreaPlotter(BEP, 'FaceColor', 'red', 'EdgeColor', 'red');
    plotCoverageArea(cap, sensors{i}.MountingLocation(1:2), ...
        sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), sensors{i}.FieldOfView(1));
end

for i = 7:8
    cap = coverageAreaPlotter(BEP, 'FaceColor', 'blue', 'EdgeColor', 'blue');
    plotCoverageArea(cap, sensors{i}.SensorLocation, sensors{i}.MaxRange, sensors{i}.Yaw, 45);
end

detectionPlotter(BEP, 'DisplayName', 'vision', 'MarkerEdgeColor', 'blue', 'Marker', '^');
detectionPlotter(BEP, 'DisplayName', 'radar', 'MarkerEdgeColor', 'red');
laneMarkingPlotter(BEP, 'DisplayName', 'lane markings');
trackPlotter(BEP, 'DisplayName', 'track', 'HistoryDepth', 10);

axis(BEP.Parent, 'equal');
xlim(BEP.Parent, [-frontBackLim frontBackLim]);
ylim(BEP.Parent, [-40 40]);
outlinePlotter(BEP, 'Tag', 'Ground truth');
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
isRadar = true(N, 1);

for i = 1:N
    detPos(i,:) = detections{i}.Measurement(1:2)';
    if detections{i}.SensorIndex > 6
        isRadar(i) = false;
    end
end

plotDetection(findPlotter(BEP, 'DisplayName', 'vision'), detPos(~isRadar,:));
plotDetection(findPlotter(BEP, 'DisplayName', 'radar'), detPos(isRadar,:));

isNotBarrier = arrayfun(@(t) t.ObjectClassID, confirmedTracks) > 0;
confirmedTracks = confirmedTracks(isNotBarrier);

trackIDs = {confirmedTracks.TrackID};
labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
[tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
tracksVel = getTrackVelocities(confirmedTracks, vsel);
plotTrack(findPlotter(BEP, 'DisplayName', 'track'), tracksPos, tracksVel, tracksCov, labels);
end
