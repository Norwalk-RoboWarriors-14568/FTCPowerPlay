package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "auto")
@Config
public class RedSideAuto extends LinearOpMode {
    public static double MAX_VEL = 100;
    public static double MAX_ACC = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())

                .splineTo(new Vector2d(1, 0), 0)
                .splineTo(new Vector2d(1, 24), 0)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(24, 24), 0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(48, 24), 0)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .forward(2).build();
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d())
               .strafeLeft(24).build();

        drive.followTrajectory(traj4);
        sleep(300);
        //drive.followTrajectory(traj5);
        sleep(300);
        //drive.followTrajectory(traj3);
        sleep(2000);
/*
        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180),
                                new TranslationalVelocityConstraint(MAX_VEL),
                                new ProfileAccelerationConstraint(MAX_ACC))
                        .build()
        );*/
    }
}
