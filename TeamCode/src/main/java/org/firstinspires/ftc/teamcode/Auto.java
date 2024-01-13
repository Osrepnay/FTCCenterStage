package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opencv.Propecessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

@Autonomous(name = "Auto", group = "Iterative Opmode", preselectTeleOp = "MainTele")
public class Auto extends OpMode {
    private Propecessor processor;
    private VisionPortal portal;
    private Propecessor.Spike spike;
    private SampleMecanumDrive drive;
    private StateManager stateManager;
    private boolean autoFinished = false;

    private enum AutoState {
        TO_SPIKE(0, 0),
        DROP_PURPLE(-1, 1000),
        TO_BACKDROP(1, 0),
        DROP_YELLOW(-1, 1000),
        PARK(2, 0);
        public final int pathIdx;
        public final long minTime;
        private static final AutoState[] vals = values();

        AutoState(int pathIdx, long minTime) {
            this.pathIdx = pathIdx;
            this.minTime = minTime;
        }

        public Optional<AutoState> next() {
            if (this == vals[vals.length - 1]) {
                return Optional.empty();
            } else {
                return Optional.of(vals[this.ordinal() + 1]);
            }
        }

        public void start() {
            switch (this) {
                case TO_SPIKE:
                    break;
                case DROP_PURPLE:
                    break;
                case TO_BACKDROP:
                    break;
                case DROP_YELLOW:
                    break;
                case PARK:
                    break;
            }
        }
    }

    private AutoState autoState;

    private Map<Propecessor.Spike, List<TrajectorySequence>> paths;
    private final boolean IS_RED = true;

    // frormular: x * 23.563 + 11.781, y * 23.563 + 11.781 for tile

    private Pose2d flipPose(Pose2d pose) {
        if (IS_RED) {
            return pose;
        } else {
            return new Pose2d(pose.getX(), -pose.getY(), 2 * Math.PI - pose.getHeading());
        }
    }

    private Vector2d flipVector(Vector2d vec) {
        if (IS_RED) {
            return vec;
        } else {
            return new Vector2d(vec.getX(), -vec.getY());
        }
    }

    private Pose2d startPoseRaw = new Pose2d(-37.719, -61.281, Math.toRadians(-90));
    private List<TrajectorySequence> chosenPaths;

    @Override
    public void init() {
        stateManager = new StateManager(hardwareMap, telemetry);
        processor = new Propecessor(telemetry);
        processor.isRed = IS_RED;
        portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                processor
        );
        autoState = AutoState.TO_SPIKE;

        Pose2d startPoseCorrected = flipPose(startPoseRaw);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPoseCorrected);
        paths = new EnumMap<>(Propecessor.Spike.class);
        List<TrajectorySequence> spikePaths = new ArrayList<>();
        spikePaths.add(drive.trajectorySequenceBuilder(startPoseCorrected)
                .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), Math.toRadians(90))
                .splineToSplineHeading(flipPose(new Pose2d(-35.344, -31.244, Math.toRadians(180))), Math.toRadians(100))
                .build());
        spikePaths.add(drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
                .splineToConstantHeading(flipVector(new Vector2d(-35.344, -35.344)), Math.toRadians(-10))
                .splineToConstantHeading(flipVector(new Vector2d(15, -31.244)), Math.toRadians(0))
                .splineToSplineHeading(flipPose(new Pose2d(47.126, -35.344 + 6, Math.toRadians(0))), Math.toRadians(0))
                .build());
        spikePaths.add(drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(54, -58.907)), 0)
                .build());
        paths.put(Propecessor.Spike.LEFT, spikePaths);
        spikePaths = new ArrayList<>();
        spikePaths.add(drive.trajectorySequenceBuilder(startPoseCorrected)
                .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), Math.toRadians(90))
                .splineToSplineHeading(flipPose(new Pose2d(-35.344, -31.244, Math.toRadians(0))), Math.toRadians(80))
                .build());
        spikePaths.add(drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
                .splineToConstantHeading(flipVector(new Vector2d(-32, -35.344)), Math.toRadians(-10))
                .splineToConstantHeading(flipVector(new Vector2d(15, -35.344)), Math.toRadians(0))
                .splineToSplineHeading(flipPose(new Pose2d(47.126, -35.344 + 6, Math.toRadians(0))), Math.toRadians(0))
                .build());
        spikePaths.add(drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(54, -58.907)), 0)
                .build());
        paths.put(Propecessor.Spike.RIGHT, spikePaths);
        spikePaths = new ArrayList<>();
        spikePaths.add(drive.trajectorySequenceBuilder(startPoseCorrected)
                .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), Math.toRadians(90))
                .splineToSplineHeading(flipPose(new Pose2d(-35.344, -35.344, Math.toRadians(90))), Math.toRadians(90))
                .build());
        spikePaths.add(drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
                .splineToLinearHeading(flipPose(new Pose2d(-33, -35.344, Math.toRadians(0))), Math.toRadians(0))
                .splineToConstantHeading(flipVector(new Vector2d(15, -35.344)), Math.toRadians(0))
                .splineToSplineHeading(flipPose(new Pose2d(47.126, -35.344 + 6, Math.toRadians(0))), Math.toRadians(0))
                .build());
        spikePaths.add(drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(54, -58.907)), 0)
                .build());
        paths.put(Propecessor.Spike.CENTER, spikePaths);
    }

    @Override
    public void start() {
        // portal.stopStreaming();
        chosenPaths = paths.get(processor.selectedSpike);
        // TODO technically bad; this shouldn't know what TO_SPIKE is
        drive.followTrajectorySequenceAsync(chosenPaths.get(0));
    }

    private long delayStart = System.currentTimeMillis();
    private long delayLen = 0;

    @Override
    public void loop() {
        if (!autoFinished) {
            drive.update();
            stateManager.update();
            boolean busy = (delayStart != -1 && System.currentTimeMillis() - delayStart < delayLen)
                    || (autoState.pathIdx == -1 ? stateManager.isBusy() : drive.isBusy());
            if (!busy) {
                Optional<AutoState> newState = autoState.next();
                if (newState.isPresent()) {
                    autoState = newState.get();
                    if (autoState.pathIdx != -1) { // roadrunner bit, probably
                        drive.followTrajectorySequenceAsync(chosenPaths.get(autoState.pathIdx));
                    } else {
                        switch (autoState) {
                            case DROP_PURPLE:
                                stateManager.queueState(StateManager.State.RAISIN);
                                stateManager.armTicksOverride = 1000;
                                stateManager.queueState(StateManager.State.SCORE_SINGLE);
                                stateManager.armTicksOverride = -1;
                                stateManager.queueState(StateManager.State.LOWERED);
                                break;
                            case DROP_YELLOW:
                                stateManager.raiseArm();
                                stateManager.queueState(StateManager.State.SCORE_DOUBLE);
                                stateManager.queueState(StateManager.State.LOWERED);
                                break;
                            default:
                                throw new IllegalStateException("unknown nonpath: " + autoState);
                        }
                        stateManager.update();
                    }
                    delayStart = System.currentTimeMillis();
                    delayLen = autoState.minTime;
                } else {
                    autoFinished = true;
                }
            }
            telemetry.addData("state", autoState);
        }
    }
}
