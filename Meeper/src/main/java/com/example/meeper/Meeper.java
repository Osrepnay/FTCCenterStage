package com.example.meeper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Meeper {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-46.719 + 18 / 2, -70.281 + 18 / 2, Math.toRadians(-90));
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meep)
                .setConstraints(30, 30, Math.toRadians(140), Math.toRadians(30), 16.8)
                .setStartPose(startPose)
                /* left spike
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(-37.719, -55), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-35.344, -31.244, Math.toRadians(180)), Math.toRadians(80))
                        .waitSeconds(0)
                        .splineToConstantHeading(new Vector2d(-35.344, -35.344), Math.toRadians(-10))
                        .splineToConstantHeading(new Vector2d(15, -35.344), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(47.126, -35.344 + 6, Math.toRadians(0)), Math.toRadians(0))
                        .setReversed(true)
                        .splineTo(new Vector2d(54, -58.907), 0)
                        .build());
                 */
                 /* right spike
                 .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(-37.719, -55), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-35.344, -31.244, Math.toRadians(0)), Math.toRadians(80))
                        .waitSeconds(0)
                        .splineToConstantHeading(new Vector2d(-32, -35.344), Math.toRadians(-10))
                        .splineToConstantHeading(new Vector2d(15, -35.344), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(47.126, -35.344 + 6, Math.toRadians(0)), Math.toRadians(0))
                        .setReversed(true)
                        .splineTo(new Vector2d(54, -58.907), 0)
                        .build());
                  */
                // center - surprisingly mostly unchanged from left, just the spike parts reoriented/shifted a little
                // turn into truss passthrough is isolated and basically a point turn - sucks
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(-37.719, -55), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-35.344, -35.344, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(0)
                        .splineToLinearHeading(new Pose2d(-33, -35.344, Math.toRadians(0)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(15, -35.344), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(47.126, -35.344 + 6, Math.toRadians(0)), Math.toRadians(0))
                        .setReversed(true)
                        .splineTo(new Vector2d(54, -58.907), 0)
                        .build());
        meep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}