package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initPose = new Pose2d(11.66, 62, Math.toRadians(-90));
        Vector2d moveBeyondTrussPose = new Vector2d(11.66, 40);
        Vector2d dropPurplePixelPose = new Vector2d(0, 0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(initPose)
                                .splineToConstantHeading(moveBeyondTrussPose, Math.toRadians(-90))
                                .splineToConstantHeading(moveBeyondTrussPose, Math.toRadians(-90))
                                //.lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(0)))
                                /*.forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .waitSeconds(3)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))*/
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}