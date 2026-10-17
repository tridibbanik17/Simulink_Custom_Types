%% runSensorFusionCustomScenario.m
% Live sensor fusion + ROS 2 odometry publishing using generateSensorData.m

clearvars
close all
clc

[~, scenario, sensors] = generateSensorData();
egoCar = scenario.Actors(1);
numSensors = numel(sensors);

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

    detectionClusters = {};
    isValidTime = false(1, numSensors);

    for i = 1:numSensors
        poses = targetPoses(scenario, sensors{i}.SensorIndex);
        [sensorDets, numValidDets, isValidTime(i)] = sensors{i}(poses, time);

        if numValidDets
            for j = 1:numValidDets
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end

                sensorDets{j}.Measurement = sensorDets{j}.Measurement([1 2 4 5]);
                sensorDets{j}.MeasurementNoise = ...
                    sensorDets{j}.MeasurementNoise([1 2 4 5], [1 2 4 5]);
            end

            detectionClusters = [detectionClusters; sensorDets]; %#ok<AGROW>
        end
    end

    if any(isValidTime)
        confirmedTracks = updateTracks(tracker, detectionClusters, time);
        updateBEP(BEP, egoCar, detectionClusters, confirmedTracks, positionSelector, velocitySelector);
    end

    if egoCar.Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end

    egoPos = egoCar.Position;
    egoVel = egoCar.Velocity;

    msg = ros2message(odomPub);
    msg.header.frame_id = 'map';
    msg.child_frame_id = 'base_link';

    msg.pose.pose.position.x = egoPos(1);
    msg.pose.pose.position.y = egoPos(2);
    msg.pose.pose.position.z = egoPos(3);
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

for i = 1:4
    cap = coverageAreaPlotter(BEP, 'FaceColor', 'blue', 'EdgeColor', 'blue');
    plotCoverageArea(cap, sensors{i}.SensorLocation, sensors{i}.MaxRange, sensors{i}.Yaw, 45);
end

for i = 5:6
    cap = coverageAreaPlotter(BEP, 'FaceColor', 'red', 'EdgeColor', 'red');
    plotCoverageArea(cap, sensors{i}.MountingLocation(1:2), ...
        sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), sensors{i}.FieldOfView(1));
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
isRadar = false(N, 1);

for i = 1:N
    detPos(i,:) = detections{i}.Measurement(1:2)';
    isRadar(i) = detections{i}.SensorIndex > 4;
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
