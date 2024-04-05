package com.example.meeper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.Function;

public class Meeper {
    private static final boolean IS_RED = true;

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

    private static TrajectorySequenceBuilder sequence(TrajectorySequenceBuilder init,
                                                      Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder>... fs) {
        for (Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> f : fs) {
            init = f.apply(init);
        }
        return init;
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meep = new MeepMeep(800);
        final boolean SHORT = true;
        Pose2d startPose = flipPose(new Pose2d(-37.838, -61.777, Math.toRadians(-90)));
        Pose2d startPoseClose = flipPose(new Pose2d(14.276, -61.777, Math.toRadians(-90)));

        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftStart = (b) -> b
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                .splineToSplineHeading(flipPose(new Pose2d(-35.344 - 5, -35.34 + 8, Math.toRadians(0))),
                        flipDirection(Math.toRadians(90)))
                .forward(7)
                .addDisplacementMarker(() -> {})
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {})
                .setReversed(false)
                .lineToConstantHeading(flipVector(new Vector2d(-35.344, -12)))
                .waitSeconds(0)
                .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftStartClose = (b) -> b
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(14.166, -55)), flipDirection(Math.toRadians(90)))
                .splineTo(flipVector(new Vector2d(11.78 - 5, -35.34)), flipDirection(Math.toRadians(180)))
                .forward(7)
                .addDisplacementMarker(() -> {})
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {})
                .setReversed(false)
                .splineTo(flipVector(new Vector2d(52.5, -35.344 + 6)), Math.toRadians(0))
                .addDisplacementMarker(() -> {})
                .waitSeconds(1)
                .addDisplacementMarker(() -> {});
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> centerStart = (b) -> b
                .setReversed(true)
                .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                .splineTo(flipVector(new Vector2d(-45, -25)), flipDirection(Math.toRadians(90)))
                .lineToLinearHeading(flipPose(new Pose2d(-40, -25, flipDirection(Math.toRadians(180)))))
                .lineToLinearHeading(flipPose(new Pose2d(-37, -12, flipDirection(Math.toRadians(90)))))
                .addDisplacementMarker(() -> {})
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {})
                .setReversed(false)
                .strafeLeft(5)
                .lineToLinearHeading(flipPose(new Pose2d(-35, -12, Math.toRadians(0))))
                .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> centerStartClose = (b) -> b
                .setReversed(true)
                .splineToConstantHeading(flipVector(new Vector2d(14.17, -55)), flipDirection(Math.toRadians(90)))
                .splineToSplineHeading(flipPose(new Pose2d(11.78, -34, Math.toRadians(-90))),
                        flipDirection(Math.toRadians(80)))
                .forward(7)
                .addDisplacementMarker(() -> {})
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {})
                .setReversed(false)
                .lineToLinearHeading(flipPose(new Pose2d(52.5, -35.344, Math.toRadians(0))))
                .addDisplacementMarker(() -> {})
                .waitSeconds(1)
                .addDisplacementMarker(() -> {});
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rightStart = (b) -> b
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                .splineTo(flipVector(new Vector2d(-35.344 + 5, -35.34)), flipDirection(Math.toRadians(0)))
                .forward(7)
                .addDisplacementMarker(() -> {})
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {})
                .setReversed(false)
                .splineTo(flipVector(new Vector2d(-20, -12)), flipDirection(Math.toRadians(0)))
                .splineTo(flipVector(new Vector2d(25, -12)), flipDirection(Math.toRadians(0)));
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rightStartClose = (b) -> b
                .setReversed(true)
                // .setTangent(Math.toRadians(0))
                .splineTo(flipVector(new Vector2d(35.34 - 5, -35.34)), flipDirection(Math.toRadians(180)))
                .forward(7)
                .addDisplacementMarker(() -> {})
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {})
                .setReversed(false)
                .splineTo(flipVector(new Vector2d(52.5, -35.344 - 6)), Math.toRadians(0))
                .addDisplacementMarker(() -> {})
                .waitSeconds(1)
                .addDisplacementMarker(() -> {});

        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> leftEnd = (b) -> b
                .splineTo(flipVector(new Vector2d(52.5, -35.344 + 6)), Math.toRadians(0))
                .addDisplacementMarker(() -> {})
                .waitSeconds(1.2)
                .addDisplacementMarker(() -> {});
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> centerEnd = (b) -> b
                .splineTo(flipVector(new Vector2d(52.5, -35.344)), Math.toRadians(0))
                .addDisplacementMarker(() -> {})
                .waitSeconds(1.2)
                .addDisplacementMarker(() -> {});
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> rightEnd = (b) -> b
                .splineTo(flipVector(new Vector2d(52.5, -35.344 - 6)), Math.toRadians(0))
                .addDisplacementMarker(() -> {})
                .waitSeconds(1.2)
                .addDisplacementMarker(() -> {});
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> park = (b) -> b
                .setReversed(true)
                .back(5)
                .splineTo(flipVector(new Vector2d(57, -58.907)), flipDirection(0))
                .setReversed(false);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> park2 = (b) -> b
                .lineToConstantHeading(flipVector(new Vector2d(50, -12)))
                .forward(10);
        Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> cycle = (b) -> b
                .setReversed(true)
                .splineTo(flipVector(new Vector2d(25, -10.485)), flipDirection(Math.toRadians(180)))
                .splineTo(flipVector(new Vector2d(-54, -10.485)), flipDirection(Math.toRadians(180)))
                .waitSeconds(0)
                .splineTo(flipVector(new Vector2d(-58.3, -10.485)), flipDirection(Math.toRadians(180)))
                .setReversed(false)
                .splineTo(flipVector(new Vector2d(25, -10.485)), flipDirection(Math.toRadians(0)));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meep)
                .setConstraints(45, 30, Math.toRadians(140), Math.toRadians(60), 16.8)
                .setStartPose(startPose)
                .followTrajectorySequence(drive ->
                        sequence(
                                drive.trajectorySequenceBuilder(startPose),
                                centerStart, cycle, centerEnd
                        )
                                .build());
        meep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}