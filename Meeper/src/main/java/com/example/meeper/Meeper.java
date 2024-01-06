package com.example.meeper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(startPose.getX(), startPose.getY() + 3, startPose.getHeading()), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(0)), Math.toRadians(80))
                        .build());
        meep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}