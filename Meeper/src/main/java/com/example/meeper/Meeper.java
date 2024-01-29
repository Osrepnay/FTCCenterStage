package com.example.meeper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meep = new MeepMeep(800);
        Pose2d startPose = flipPose(new Pose2d(-46.719 + 18 / 2, -70.281 + 18 / 2, Math.toRadians(-90)));
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meep)
                .setConstraints(30, 30, Math.toRadians(140), Math.toRadians(30), 16.8)
                .setStartPose(startPose)
        // left red
                /*
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineToConstantHeading(flipVector(new Vector2d(-37.719, -55)), flipDirection(Math.toRadians(90)))
                        .splineToSplineHeading(flipPose(new Pose2d(-35.344, -31.244, Math.toRadians(0))), flipDirection(Math.toRadians(80)))
                        .setReversed(false)
                        .waitSeconds(0)
                        .setReversed(true)
                        // has to be line- spline takes away the pixel
                        .lineToConstantHeading(flipVector(new Vector2d(-35.344, -58.907)))
                        .lineToConstantHeading(flipVector(new Vector2d(15, -58.907)))
                        .splineToSplineHeading(flipPose(new Pose2d(47.126, -35.344 + 6, Math.toRadians(0))), flipDirection(Math.toRadians(0)))
                        .setReversed(false)
                        .setReversed(true)
                        .splineTo(flipVector(new Vector2d(57, -58.907)), flipDirection(0))
                        .build());
        */
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