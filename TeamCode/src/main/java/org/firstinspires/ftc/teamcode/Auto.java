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
        TO_BACKDROP(1, 0),
        RAISE_YELLOW(-1, 2000),
        DROP_YELLOW(-1, 2000),
        RETRACT_YELLOW(-1, 500),
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
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftStart, centerStart, rightStart, back;
        if (!SHORT) {
            startPoseRaw = new Pose2d(-37.719, -61.281, Math.toRadians(-90));
            leftStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(-35.344, -29 + 5, Math.toRadians(0))), flipDirection(Math.toRadians(80)))
                    .setReversed(false);
            centerStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(-35.344, -34, Math.toRadians(-90))), flipDirection(Math.toRadians(80)))
                    .setReversed(false);
            rightStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(-35.344, -29 - 5, Math.toRadians(180))), flipDirection(Math.toRadians(80)))
                    .setReversed(false);
            back = (b) -> b
                    .lineToConstantHeading(flipVector(new Vector2d(-35.455, -35)))
                    .splineToSplineHeading(flipPose(new Pose2d(-48, -58.907, Math.toRadians(0))), flipDirection(Math.toRadians(0)))
                    .lineToConstantHeading(flipVector(new Vector2d(-11.781, -58.907)));
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
            back = (b) -> b
                    .lineToSplineHeading(flipPose(new Pose2d(16, -51, Math.toRadians(0))))
                    .splineToSplineHeading(flipPose(new Pose2d(35.344, -56, Math.toRadians(0))), flipDirection(Math.toRadians(0)));
        }
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> pushProp = (b) -> b
                .back(6)
                .forward(6);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> bottomRendevous = (b) -> b
                .lineToConstantHeading(flipVector(new Vector2d(35.344, -58.907)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> topRendevous = (b) -> b
                .lineToConstantHeading(flipVector(new Vector2d(-11.781, -10)))
                .lineToConstantHeading(flipVector(new Vector2d(35.344, -10)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rendevous = SHORT
                ? ((x) -> x)
                : GO_TOP
                ? topRendevous
                : bottomRendevous;
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftEnd = (b) -> b
                .splineToSplineHeading(flipPose(new Pose2d(53.5, -35.344 + 6, Math.toRadians(0))), flipDirection(Math.toRadians(0)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> centerEnd = (b) -> b
                .splineToSplineHeading(flipPose(new Pose2d(53.5, -35.344, Math.toRadians(0))), flipDirection(Math.toRadians(0)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rightEnd = (b) -> b
                .splineToSplineHeading(flipPose(new Pose2d(53.5, -35.344 - 6, Math.toRadians(0))), flipDirection(Math.toRadians(0)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> park = (b) -> b
                .setReversed(true)
                .back(5)
                .splineTo(flipVector(new Vector2d(57, -58.907)), flipDirection(0))
                .setReversed(false);

        Pose2d startPoseCorrected = flipPose(startPoseRaw);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPoseCorrected);
        paths = new EnumMap<>(Propecessor.Spike.class);
        List<TrajectorySequence> spikePaths = new ArrayList<>();
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                leftStart,
                pushProp
        ).build());
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end()),
                back,
                rendevous,
                leftEnd
        ).build());
        spikePaths.add(park.apply(
                drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
        ).build());
        paths.put(Propecessor.Spike.LEFT, spikePaths);
        spikePaths = new ArrayList<>();
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                rightStart,
                pushProp
        ).build());
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end()),
                back,
                rendevous,
                rightEnd
        ).build());
        spikePaths.add(park.apply(
                drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
        ).build());
        paths.put(Propecessor.Spike.RIGHT, spikePaths);
        spikePaths = new ArrayList<>();
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                centerStart,
                pushProp
        ).build());
        spikePaths.add(sequence(
                drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end()),
                back,
                rendevous,
                centerEnd
        ).build());
        spikePaths.add(park.apply(
                drive.trajectorySequenceBuilder(spikePaths.get(spikePaths.size() - 1).end())
        ).build());
        paths.put(Propecessor.Spike.CENTER, spikePaths);
    }

    @Override
    public void start() {
        portal.setProcessorEnabled(processor, false);
        chosenPaths = paths.get(IS_RED ? processor.selectedSpike : processor.selectedSpike.mirror());
        // TODO HACK
        try {
            Thread.sleep(0);
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
                                /*stateManager.queueState(StateManager.State.RAISIN);
                                stateManager.armTicksOverride = 610;
                                stateManager.queueState(StateManager.State.SCORE_SINGLE);
                                stateManager.armTicksOverride = -1;
                                stateManager.queueState(StateManager.State.LOWERED);*/
                                stateManager.intake.setPower(0.4);
                                break;
                            case STOP_PURPLE_DROP:
                                stateManager.intake.setPower(0);
                                break;
                            case RAISE_YELLOW:
                                stateManager.raiseArm();
                                break;
                            case DROP_YELLOW:
                                stateManager.queueState(StateManager.State.SCORE_DOUBLE);
                                break;
                            case RETRACT_YELLOW:
                                stateManager.queueState(StateManager.State.LOWERED);
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
            telemetry.addData("state", autoState);
        } else {
            // crash itself
            portal.close();
        }
    }
}
