package com.example.meeper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.Function;

public class Meeper {
    private static final boolean IS_RED = false;

    private static double flipDirection(double rad) {
        if (IS_RED) {
            return rad;
        } else {
            return 2 * Math.PI - rad;
        }
    }

    private static Pose2d flipPose(Pose2d pose) {
        if (IS_RED) {
            return pose;
        } else {
            return new Pose2d(pose.getX(), -pose.getY(), flipDirection(pose.getHeading()));
        }
    }

    private static Vector2d flipVector(Vector2d vec) {
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

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meep = new MeepMeep(800);
        final boolean SHORT = true;
        Pose2d startPose = flipPose(new Pose2d(-37.719, -70.281 + 18 / 2, Math.toRadians(-90)));
        Pose2d startPoseClose = flipPose(new Pose2d(14.166, -70.281 + 18 / 2, Math.toRadians(-90)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftStart = (b) -> b
                .setReversed(true)
                .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                .splineToSplineHeading(flipPose(new Pose2d(-35.344, -29 + 5, Math.toRadians(0))), flipDirection(Math.toRadians(80)))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> centerStart = (b) -> b
                .setReversed(true)
                .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                .splineToSplineHeading(flipPose(new Pose2d(-35.344, -34, Math.toRadians(-90))), flipDirection(Math.toRadians(80)))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rightStart = (b) -> b
                .setReversed(true)
                .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                .splineToSplineHeading(flipPose(new Pose2d(-35.344, -29 - 5, Math.toRadians(180))), flipDirection(Math.toRadians(80)))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftStartClose = (b) -> b
                .setReversed(true)
                .splineToConstantHeading(flipVector(new Vector2d(14.166, -55)), flipDirection(Math.toRadians(90)))
                .splineToSplineHeading(flipPose(new Pose2d(11.781, -29 + 5, Math.toRadians(0))), flipDirection(Math.toRadians(100)))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> centerStartClose = (b) -> b
                .setReversed(true)
                .splineToConstantHeading(flipVector(new Vector2d(14.166, -55)), flipDirection(Math.toRadians(90)))
                .splineToSplineHeading(flipPose(new Pose2d(11.781, -34, Math.toRadians(-90))), flipDirection(Math.toRadians(100)))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rightStartClose = (b) -> b
                .setReversed(true)
                .splineToConstantHeading(flipVector(new Vector2d(14.166, -55)), flipDirection(Math.toRadians(90)))
                .splineToSplineHeading(flipPose(new Pose2d(11.781, -29 - 5, Math.toRadians(180))), flipDirection(Math.toRadians(100)))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> pushProp = (b) -> b
                .back(3)
                .forward(3);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> back = (b) -> b
                .lineToConstantHeading(flipVector(new Vector2d(-35.455, -35)))
                .splineToSplineHeading(flipPose(new Pose2d(-48, -58.907, Math.toRadians(0))), flipDirection(Math.toRadians(0)))
                .lineToConstantHeading(flipVector(new Vector2d(-11.781, -58.907)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> backClose = (b) -> b
                .lineToSplineHeading(flipPose(new Pose2d(16, -51, Math.toRadians(0))))
                .splineToSplineHeading(flipPose(new Pose2d(35.344, -56, Math.toRadians(0))), flipDirection(Math.toRadians(0)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> bottomRendevous = (b) -> b
                .lineToConstantHeading(flipVector(new Vector2d(35.344, -58.907)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> topRendevous = (b) -> b
                .lineToConstantHeading(flipVector(new Vector2d(-11.781, -10)))
                .lineToConstantHeading(flipVector(new Vector2d(35.344, -10)));
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
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meep)
                .setConstraints(30, 30, Math.toRadians(140), Math.toRadians(30), 16.8)
                .setStartPose(startPose)
        // left red
                .followTrajectorySequence(drive ->
                        sequence(
                                drive.trajectorySequenceBuilder(startPose),
                                leftStart,
                                pushProp,
                                back,
                                topRendevous,
                                leftEnd,
                                park
                                /*
                                (b) -> b
                                        .waitSeconds(0)
                                        .setReversed(true)
                                        // has to be line- spline takes away the pixel
                                        .setReversed(false)
                                        .setReversed(true)
                                 */
                        )
                        .build());
                /* right red */
                /* .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                        .splineToSplineHeading(flipPose(new Pose2d(-35.344, -31.244, Math.toRadians(180))), flipDirection(Math.toRadians(80)))
                        .setReversed(false)
                        .waitSeconds(0)
                        .splineToConstantHeading(flipVector(new Vector2d(-35.344, -58.907)), flipDirection(Math.toRadians(0)))
                        .lineToConstantHeading(flipVector(new Vector2d(15, -58.907)))
                        .splineToSplineHeading(flipPose(new Pose2d(47.126, -35.344 - 6, Math.toRadians(0))), flipDirection(Math.toRadians(0)))
                        .setReversed(true)
                        .splineTo(flipVector(new Vector2d(57, -58.907)), flipDirection(0))
                        .build());
                 */
        // center - surprisingly mostly unchanged from left, just the spike parts reoriented/shifted a little
        // turn into truss passthrough is isolated and basically a point turn - sucks
                /*
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                        .splineToSplineHeading(flipPose(new Pose2d(-35.344, -32, Math.toRadians(-90))), flipDirection(Math.toRadians(80)))
                        .setReversed(false)
                        .waitSeconds(0)
                        .splineToSplineHeading(flipPose(new Pose2d(-40, -58.907, Math.toRadians(0))), flipDirection(Math.toRadians(-90)))
                        .lineToConstantHeading(flipVector(new Vector2d(15, -58.907)))
                        .splineToSplineHeading(flipPose(new Pose2d(47.126, -35.344, Math.toRadians(0))), flipDirection(Math.toRadians(90)))
                        .setReversed(true)
                        .splineTo(flipVector(new Vector2d(57, -58.907)), flipDirection(0))
                        .build());
                 */
        meep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}