package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Autonomous(group = "LIAMS ODOMETRY CODE  ")
public class RRTrajectoryTune extends LinearOpMode {
     @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        Pose2d originPose = new Pose2d(0, 36, 0);
        drivetrain.setPoseEstimate(originPose);
        waitForStart();

        if (isStopRequested()) return;

         Pose2d poseEstimate = drivetrain.getPoseEstimate();
         telemetry.addData("x", poseEstimate.getX());
         telemetry.addData("y", poseEstimate.getY());
         telemetry.addData("heading", poseEstimate.getHeading());
         telemetry.update();

             Trajectory splineToPosition = drivetrain.trajectoryBuilder(originPose)
                     .splineTo(new Vector2d(60, 36), 180)
                     //.splineTo(new Vector2d(60, 84), 180)
                     //.splineTo(new Vector2d(60, 36), Math.toRadians(0))
                     //.splineTo(new Vector2d(60, 36), Math.toRadians(0))
                     //.splineTo(new Vector2d(60, 36), Math.toRadians(0))
                     .build();

             drivetrain.followTrajectory(splineToPosition);

         poseEstimate = drivetrain.getPoseEstimate();
             telemetry.addData("x", poseEstimate.getX());
             telemetry.addData("y", poseEstimate.getY());
             telemetry.addData("heading", poseEstimate.getHeading());
             telemetry.update();
    }
}