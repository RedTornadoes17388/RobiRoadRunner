package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48,66,Math.toRadians(0)))
                                .strafeRight(3)
                                .setReversed(true)
                                .splineTo(new Vector2d(-60, 60), Math.toRadians(205))//to Duck
                                //separate: spin duck wheel
                                .waitSeconds(3)
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(-60, 48, Math.toRadians(0)))
                                //async: fly-by of team shipping element for scanning
                                .splineTo(new Vector2d(-12, 48), Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                                //separate: place object on correct level
                                //head to warehouse
                                .lineToLinearHeading(new Pose2d(10,66, Math.toRadians(0)))
                                .forward(40)//maybe interrupt when object is acquired
                                //return to shipping station
                                .back(40)
                                .strafeRight(1)
                                .splineTo(new Vector2d(-12, 48), Math.toRadians(180))
                                //return to warehouse to park
                                .lineToLinearHeading(new Pose2d(10,66, Math.toRadians(0)))
                                .forward(40)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}