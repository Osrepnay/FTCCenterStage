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

import java.util.EnumMap;
import java.util.Map;
import java.util.function.Function;

@Autonomous(name = "AutoRedFar", group = "Iterative Opmode", preselectTeleOp = "MainTele")
public class AutoRedFar extends OpMode {
    private Propecessor processor;
    private VisionPortal portal;
    private Propecessor.Spike spike;
    private SampleMecanumDrive drive;
    private StateManager stateManager;
    private Map<Propecessor.Spike, TrajectorySequence> paths;
    private boolean autoFinished = false;

    private final boolean SHORT = false;
    private final boolean IS_RED = true;

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

    private static TrajectorySequenceBuilder sequence(TrajectorySequenceBuilder init,
                                                      Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder>... fs) {
        for (Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> f : fs) {
            init = f.apply(init);
        }
        return init;
    }

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
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftEnd = (b) -> b
                .addDisplacementMarker(() -> {
                    stateManager.raiseArm();
                    stateManager.raiseArm();
                })
                .splineTo(flipVector(new Vector2d(52.5, -35.344 + 5)), flipDirection(Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.SCORE_DOUBLE))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> stateManager.queueState(StateManager.State.LOWERED))
                .waitSeconds(1.2);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> centerEnd = (b) -> b
                .addDisplacementMarker(() -> {
                    stateManager.raiseArm();
                    stateManager.raiseArm();
                })
                .splineTo(flipVector(new Vector2d(52.5, -35.344 + 1.2)), flipDirection(Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.SCORE_DOUBLE))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> stateManager.queueState(StateManager.State.LOWERED))
                .waitSeconds(1.2);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rightEnd = (b) -> b
                .addDisplacementMarker(() -> {
                    stateManager.raiseArm();
                    stateManager.raiseArm();
                })
                .splineTo(flipVector(new Vector2d(52.5, -35.344 - 4.5)), flipDirection(Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.SCORE_DOUBLE))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> stateManager.queueState(StateManager.State.LOWERED))
                .waitSeconds(1.2);
        Pose2d startPoseRaw;
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftStart, centerStart, rightStart;
        if (!SHORT) {
            startPoseRaw = new Pose2d(-37.838, -61.777, Math.toRadians(-90));
            leftStart = (b) -> b
                    .setReversed(true)
                    .splineTo(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(-35.344 - 5, -35.34 + 8, Math.toRadians(0))),
                            flipDirection(Math.toRadians(90)))
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.lowerExtendo())
                    .UNSTABLE_addTemporalMarkerOffset(0.7, () -> stateManager.raiseExtendo())
                    .waitSeconds(0.7)
                    .setReversed(false)
                    .lineToConstantHeading(flipVector(new Vector2d(-35.344, -12)))
                    .waitSeconds(0)
                    .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
            leftStart = leftStart.andThen(leftEnd);
            centerStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineTo(flipVector(new Vector2d(-45, -25)), flipDirection(Math.toRadians(90)))
                    .lineToLinearHeading(flipPose(new Pose2d(-40, -25, Math.toRadians(180))))
                    .lineToLinearHeading(flipPose(new Pose2d(-37, -12, Math.toRadians(90))))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.lowerExtendo())
                    .UNSTABLE_addTemporalMarkerOffset(0.7, () -> stateManager.raiseExtendo())
                    .waitSeconds(0.7)
                    .setReversed(false)
                    .strafeLeft(5)
                    .lineToLinearHeading(flipPose(new Pose2d(-35, -12, Math.toRadians(0))))
                    .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
            centerStart = centerStart.andThen(centerEnd);
            rightStart = (b) -> b
                    .setReversed(true)
                    .splineTo(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                    .splineTo(flipVector(new Vector2d(-35.344 + 5, -35.34)), flipDirection(Math.toRadians(0)))
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.lowerExtendo())
                    .UNSTABLE_addTemporalMarkerOffset(0.7, () -> stateManager.raiseExtendo())
                    .waitSeconds(0.7)
                    .setReversed(false)
                    .splineTo(flipVector(new Vector2d(-20, -12)), flipDirection(Math.toRadians(0)))
                    .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
            rightStart = rightStart.andThen(rightEnd);
        } else {
            startPoseRaw = new Pose2d(14.276, -61.777, Math.toRadians(-90));
            leftStart = (b) -> b
                    .setReversed(true)
                    .splineTo(flipVector(new Vector2d(14.166, -55)), flipDirection(Math.toRadians(90)))
                    .splineTo(flipVector(new Vector2d(11.78 - 5, -35.34)), flipDirection(Math.toRadians(180)))
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.lowerExtendo())
                    .UNSTABLE_addTemporalMarkerOffset(0.7, () -> stateManager.raiseExtendo())
                    .waitSeconds(0.7)
                    .setReversed(false)
                    .addDisplacementMarker(() -> {
                        stateManager.raiseArm();
                        stateManager.raiseArm();
                    })
                    .splineTo(flipVector(new Vector2d(52.5, -35.344 + 6)), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.SCORE_DOUBLE))
                    .waitSeconds(0.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.LOWERED));
            centerStart = (b) -> b
                    .setReversed(true)
                    .splineToConstantHeading(flipVector(new Vector2d(14.17, -55)), flipDirection(Math.toRadians(90)))
                    .splineToSplineHeading(flipPose(new Pose2d(11.78, -30, Math.toRadians(-90))),
                            flipDirection(Math.toRadians(80)))
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.lowerExtendo())
                    .UNSTABLE_addTemporalMarkerOffset(0.7, () -> stateManager.raiseExtendo())
                    .waitSeconds(0.7)
                    .setReversed(false)
                    .addDisplacementMarker(() -> {
                        stateManager.raiseArm();
                        stateManager.raiseArm();
                    })
                    .lineToLinearHeading(flipPose(new Pose2d(52.5, -35.344 + 1.2, Math.toRadians(0))))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.SCORE_DOUBLE))
                    .waitSeconds(0.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.LOWERED));
            rightStart = (b) -> b
                    .setReversed(true)
                    .splineTo(flipVector(new Vector2d(35.34 - 7, -35.34)), flipDirection(Math.toRadians(180)))
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.lowerExtendo())
                    .UNSTABLE_addTemporalMarkerOffset(0.7, () -> stateManager.raiseExtendo())
                    .waitSeconds(0.7)
                    .setReversed(false)
                    .addDisplacementMarker(() -> {
                        stateManager.raiseArm();
                        stateManager.raiseArm();
                    })
                    .splineTo(flipVector(new Vector2d(52.5, -35.344 - 6)), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.SCORE_DOUBLE))
                    .waitSeconds(0.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.LOWERED));
        }

        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> park = (b) -> b
                .setReversed(true)
                .back(5)
                .splineTo(flipVector(new Vector2d(57, -58.907)), flipDirection(0))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> cycle = (b) -> b
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(180)))
                .splineTo(flipVector(new Vector2d(-54, -12)), flipDirection(Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    stateManager.queueState(StateManager.State.INTAKING);
                    stateManager.lowerExtendo();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> stateManager.lowerExtendo())
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    stateManager.raiseExtendo();
                    stateManager.raiseExtendo();
                })
                .waitSeconds(1)
                .splineTo(flipVector(new Vector2d(-58.3, -12)), flipDirection(Math.toRadians(180)))
                .waitSeconds(1)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stateManager.queueState(StateManager.State.LOWERED))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stateManager.queueState(StateManager.State.INTAKING))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> stateManager.queueState(StateManager.State.LOWERED))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> stateManager.queueState(StateManager.State.INTAKING))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> stateManager.queueState(StateManager.State.LOWERED))
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> stateManager.queueState(StateManager.State.INTAKING))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> stateManager.queueState(StateManager.State.LOWERED))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> stateManager.propsTo(p -> new StateManager.StateProps(1, p.liftHeight, p.wristState, p.grabberState, p.extendoIdx)))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> stateManager.queueState(StateManager.State.LOWERED))
                .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));


        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> park2 = (b) -> b
                .lineToConstantHeading(flipVector(new Vector2d(50, -12)))
                .forward(10);
        Pose2d startPoseCorrected = flipPose(startPoseRaw);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPoseCorrected);
        paths = new EnumMap<>(Propecessor.Spike.class);
        TrajectorySequence left = sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                leftStart, cycle, rightEnd
        ).build();
        paths.put(Propecessor.Spike.LEFT, left);
        TrajectorySequence center = sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                centerStart, cycle, leftEnd
        ).build();
        paths.put(Propecessor.Spike.CENTER, center);
        TrajectorySequence right = sequence(
                drive.trajectorySequenceBuilder(startPoseCorrected),
                rightStart, cycle, leftEnd
        ).build();
        paths.put(Propecessor.Spike.RIGHT, right);
        telemetry.addLine("init done");
    }

    @Override
    public void init_loop() {
        telemetry.addData("spike", processor.selectedSpike);
        telemetry.addData("short", SHORT);
        telemetry.addData("red", IS_RED);
    }

    @Override
    public void start() {
        portal.setProcessorEnabled(processor, false);
        TrajectorySequence path = paths.get(IS_RED ? processor.selectedSpike : processor.selectedSpike.mirror());
        // TODO HACK
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        drive.followTrajectorySequenceAsync(path);
    }

    private long delayStart = System.currentTimeMillis();
    private long delayLen = 0;

    @Override
    public void loop() {
        if (!autoFinished) {
            drive.update();
            stateManager.update();
        } else {
            // crash itself
            portal.close();
        }
    }
}
