package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous
public class AutonomousAlpha21_22 extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor duckWheel = hardwareMap.get(DcMotor.class, "spinner");
        DcMotor elbow = hardwareMap.get(DcMotor.class, "elbow");
        DistanceSensor teamElementSensor = hardwareMap.get(DistanceSensor.class, "elementDetector");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        double startingElbowPos = elbow.getCurrentPosition();
        Pose2d startPose = new Pose2d(-48,66,Math.toRadians(0));
        ArrayList<Double> sensorData = new ArrayList<Double>();
/*
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(3)
                    .setReversed(true)
                    .splineTo(new Vector2d(-60, 60), Math.toRadians(205))//to Duck
                    //separate: spin duck wheel
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
                    //.lineToLinearHeading(new Pose2d(10,66, Math.toRadians(-90)))

                    .build()
*/

        //trajectory to duck
        Trajectory traj1 = drive.trajectoryBuilder(startPose,true)
                .strafeRight(8)
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj1.end(), true)
                .back(14)//to Duck
                .build();
        //trajectory past the team shipping element to laser distance scan it
        Trajectory traj2 = drive.trajectoryBuilder(traj15.end())
                .lineToLinearHeading(new Pose2d(-50, 48, Math.toRadians(270)))
                .build();

        Trajectory traj25 = drive.trajectoryBuilder((traj2.end()))
                //async: fly-by of team shipping element for scanning
                .lineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(270)))
                .build();

        /*Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
            .turn(Math.toRadians(-90))
            .build();//after this place object as necessary
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(10,66, Math.toRadians(0)))
                //return to shipping station
                .build();
        Trajectory traj45 = drive.trajectoryBuilder((traj4.end()))
                .forward(40)//maybe interrupt when object is acquired
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj45.end())
                .lineToLinearHeading(new Pose2d(10,66, Math.toRadians(-90)))
                .build();
        Trajectory traj55 = drive.trajectoryBuilder(traj5.end())
                .splineTo(new Vector2d(-12, 48), Math.toRadians(-90))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(10,66, Math.toRadians(0)))
                .build();
        Trajectory traj65 = drive.trajectoryBuilder(traj6.end())
                .forward(40)//after this we're done and we sit here
                .build();
        //.lineToLinearHeading(new Pose2d(10,66, Math.toRadians(-90)))
        */

        waitForStart();


        drive.setPoseEstimate(startPose);


        if (isStopRequested()) return;



        drive.followTrajectory(traj1);
        drive.followTrajectory(traj15);
        //now we spin the duckwheel

        duckWheel.setPower(1);
        sleep(150);
        duckWheel.setPower(0.4);
        sleep(2000);
        duckWheel.setPower(0);

        drive.followTrajectory(traj2);

        //drive.followTrajectoryAsync(traj25);

       /* while(drive.isBusy()){
            drive.update();
            //scan, do math, decide path
            sensorData.add(teamElementSensor.getDistance(DistanceUnit.INCH));
            //Leftmost position is lowest level
        }
        double sumLeft = 0;
        for(int i = 0; i<(int)(sensorData.size()/3); i++){
            sumLeft+=sensorData.get(i);
        }
        double sumCenter = 0;
        for(int i = (int)sensorData.size()/3; i<(int)(2*sensorData.size()/3); i++){
            sumCenter+=sensorData.get(i);
        }
        double sumRight = 0;
        for(int i = (int)2*sensorData.size()/3; i<(int)(sensorData.size()); i++){
            sumRight+=sensorData.get(i);
        }

        if(sumLeft<sumCenter && sumLeft<sumRight){
           //elbow low position
            while(-39 > elbow.getCurrentPosition() - ((startingElbowPos / 12500.0) * 360.0) - 60.0){
                elbow.setPower(1);
            }
        }
        else if(sumCenter<sumLeft && sumCenter<sumRight){
            //middle arm position
            while(-17 > elbow.getCurrentPosition() - ((startingElbowPos / 12500.0) * 360.0) - 60.0){
                elbow.setPower(1);
            }
        }
        else{
            while(10 > elbow.getCurrentPosition() - ((startingElbowPos / 12500.0) * 360.0) - 60.0){
                elbow.setPower(1);
            }
        }
        elbow.setPower(0);


        drive.followTrajectory(traj3);
        intake.setPower(1);//place object on correct level
        sleep(1000);
        intake.setPower(0);

        //Lower arm and turn on intake
        while(0 < elbow.getCurrentPosition() - ((startingElbowPos / 12500.0) * 360.0) - 60.0){
            elbow.setPower(-1.0);
        }

        //drive to warehouse and pick up an object. Interrupt when object acquired
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj45);
        while(drive.isBusy()){
            drive.update();
            intake.setPower(-1);
            //lower arm, break or something when a thing is acquired
            if(colorSensor.green() > 1000) {

                traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(10,66, Math.toRadians(-90)))
                        .build();

                drive.cancelFollowing();
                break;
            }
        }
        intake.setPower(0);


        //drive back to shipping hub
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj55);
        drive.followTrajectory(drive.trajectoryBuilder(traj55.end())
                .splineTo(new Vector2d(-12, 48), Math.toRadians(-90))
                .build()
        );
        //place object on correct level
        while(10 > elbow.getCurrentPosition() - ((startingElbowPos / 12500.0) * 360.0) - 60.0){
            elbow.setPower(1.0);
        }

        drive.followTrajectory(traj6);
        drive.followTrajectory(traj65);
        */


        //now we scan for the team piece
        //based on result deposit duck on appropriate level
        //go to warehouse

    }
}