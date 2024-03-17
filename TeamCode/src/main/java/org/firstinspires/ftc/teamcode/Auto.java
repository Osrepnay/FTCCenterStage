package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opencv.Propecessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

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
        DROP_PURPLE(-1, 2000),
        STOP_PURPLE_DROP(-1, 0),
        TO_BACKDROP(-1, 0),
        RAISE_YELLOW(-1, 2000),
        DROP_YELLOW(-1, 2000),
        RETRACT_YELLOW(-1, 500),
        PARK(-1, 0);
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
    private final boolean SHORT = false;
    private final boolean IS_RED = true;
    private final boolean GO_TOP = true;

    // frormular: x * 23.563 + 11.781, y * 23.563 + 11.781 for tile
    private double flipDirection(double rad) {
        if (IS_RED) {
            return rad;
        } else {
            return 2 * Math.PI - rad;
        }
    }

    private Pose2d flipPose(Pose2d pose) {
        if (IS_RED) {
            return pose;
        } else {
            return new Pose2d(pose.getX(), -pose.getY(), flipDirection(pose.getHeading()));
        }
    }

    private Vector2d flipVector(Vector2d vec) {
        if (IS_RED) {
            return vec;
        } else {
            return new Vector2d(vec.getX(), -vec.getY());
        }
    }

    private static TrajectorySequenceBuilder sequence(TrajectorySequenceBuilder init, Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder>... fs) {
        for (Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> f : fs) {
            init = f.apply(init);
        }
        return init;
    }

    private List<TrajectorySequence> chosenPaths;

    @Override
    public void init() {
        stateManager = new StateManager(hardwareMap, telemetry);
        processor = new Propecessor(telemetry);
        processor.isRed = IS_RED;
        processor.robotAlignLeft = IS_RED
                ? SHORT ? false : true
                : SHORT ? true : false;
        portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                processor
        );
        autoState = AutoState.TO_SPIKE;

        Pose2d startPoseRaw;
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftStart, centerStart, rightStart;
        if (!SHORT) {
            startPoseRaw = new Pose2d(-37.719, -61.281, Math.toRadians(-90));
            leftStart = (b) -> b
                    .setReversed(true)
                    .splineTo(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(-35.344 - 5, -35.34 + 8, Math.toRadians(0))), flipDirection(Math.toRadians(90)))
                    .forward(5)
                    .addDisplacementMarker(() -> {
                        stateManager.lowerExtendo();
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException ignored) {}
                        stateManager.raiseExtendo();
                    })
                    .setReversed(false)
                    .waitSeconds(3)
                    .lineToConstantHeading(flipVector(new Vector2d(-35.344, -12)))
                    .waitSeconds(0)
                    .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
            centerStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(-35.344, -34, Math.toRadians(-90))), flipDirection(Math.toRadians(80)))
                    .forward(5)
                    .addDisplacementMarker(() -> {
                        stateManager.lowerExtendo();
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException ignored) {}
                        stateManager.raiseExtendo();
                    })
                    .setReversed(false)
                    .setReversed(false)
                    .lineToLinearHeading(flipPose(new Pose2d(-35.34, -35.34, Math.toRadians(0))))
                    .lineToConstantHeading(flipVector(new Vector2d(-11.78, -35.34)))
                    .lineToConstantHeading(flipVector(new Vector2d(-11.78, -12)))
                    .waitSeconds(0)
                    .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
            rightStart = (b) -> b
                    .setReversed(true)
                    .splineTo(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(-35.344 + 5, -35.34, Math.toRadians(180))), flipDirection(Math.toRadians(0)))
                    .forward(5)
                    .addDisplacementMarker(() -> {
                        stateManager.lowerExtendo();
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException ignored) {}
                        stateManager.raiseExtendo();
                    })
                    .setReversed(false)
                    .setReversed(false)
                    .splineTo(flipVector(new Vector2d(-20, -12)), flipDirection(Math.toRadians(0)))
                    .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
        } else {
            startPoseRaw = new Pose2d(14.166, -70.281 + 18 / 2, Math.toRadians(-90));
            leftStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(14.166, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(11.781, -29 - 5, Math.toRadians(0))), flipDirection(Math.toRadians(100)))
                    .setReversed(false);
            centerStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(14.166, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(11.781, -34, Math.toRadians(-90))), flipDirection(Math.toRadians(100)))
                    .setReversed(false);
            rightStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(14.166, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(11.781, -29 + 5, Math.toRadians(180))), flipDirection(Math.toRadians(100)))
                    .setReversed(false);
        }
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftEnd = (b) -> b
                .splineTo(flipVector(new Vector2d(49.9, -35.344 + 6)), flipDirection(Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    stateManager.raiseArm();
                    stateManager.raiseArm();
                    stateManager.queueState(StateManager.State.SCORE_DOUBLE);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException ignored) {}
                    stateManager.queueState(StateManager.State.LOWERED);
                });
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> centerEnd = (b) -> b
                .splineTo(flipVector(new Vector2d(49.9, -35.344)), flipDirection(Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    stateManager.raiseArm();
                    stateManager.raiseArm();
                    stateManager.queueState(StateManager.State.SCORE_DOUBLE);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException ignored) {}
                    stateManager.queueState(StateManager.State.LOWERED);
                });
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rightEnd = (b) -> b
                .splineTo(flipVector(new Vector2d(49.9, -35.344 - 6)), flipDirection(Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    stateManager.raiseArm();
                    stateManager.raiseArm();
                    stateManager.queueState(StateManager.State.SCORE_DOUBLE);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException ignored) {}
                    stateManager.queueState(StateManager.State.LOWERED);
                });
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> park = (b) -> b
                .setReversed(true)
                .back(5)
                .splineTo(flipVector(new Vector2d(57, -58.907)), flipDirection(0))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> cycle = (b) -> b
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(25, -10.485)), flipDirection(Math.toRadians(180)))
                .splineTo(flipVector(new Vector2d(-59, -10.485)), flipDirection(Math.toRadians(180)))
                .waitSeconds(0)
                .splineTo(flipVector(new Vector2d(-61, -10.485)), flipDirection(Math.toRadians(180)))
                .setReversed(false)
                .splineTo(flipVector(new Vector2d(25, -10.485)), flipDirection(Math.toRadians(0)));

        Pose2d startPoseCorrected = flipPose(startPoseRaw);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPoseCorrected);
        paths = new EnumMap<>(Propecessor.Spike.class);
        List<TrajectorySequence> spikePaths = new ArrayList<>();
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                leftStart, leftEnd
        ).build());
        paths.put(Propecessor.Spike.LEFT, spikePaths);
        spikePaths = new ArrayList<>();
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                rightStart, rightEnd
        ).build());
        paths.put(Propecessor.Spike.RIGHT, spikePaths);
        spikePaths = new ArrayList<>();
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                centerStart, centerEnd
        ).build());
        paths.put(Propecessor.Spike.CENTER, spikePaths);
    }

    @Override
    public void start() {
        portal.setProcessorEnabled(processor, false);
        chosenPaths = paths.get(IS_RED ? processor.selectedSpike : processor.selectedSpike.mirror());
        // TODO HACK
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
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
                                // stateManager.intake.setPower(0.4);
                                break;
                            case STOP_PURPLE_DROP:
                                // stateManager.intake.setPower(0);
                                break;
                            case RAISE_YELLOW:
                                // stateManager.raiseArm();
                                // stateManager.raiseArm();
                                break;
                            case DROP_YELLOW:
                                // stateManager.queueState(StateManager.State.SCORE_DOUBLE);
                                break;
                            case RETRACT_YELLOW:
                                // stateManager.queueState(StateManager.State.LOWERED);
                                break;
                            default:
                                // throw new IllegalStateException("unknown nonpath: " + autoState);
                        }
                        stateManager.update();
                    }
                    delayStart = System.currentTimeMillis();
                    delayLen = autoState.minTime;
                } else {
                    autoFinished = true;
                }
            }
            // telemetry.addData("state", autoState);
        } else {
            // crash itself
            portal.close();
        }
    }
}
