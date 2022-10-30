package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "auto")
@Config
public class RedSideAuto extends LinearOpMode {
    public static double MAX_VEL = 100;
    public static double MAX_ACC = 20;
    SampleMecanumDrive drive;

    //private Servo ConeGrabber, OdLift;
    @Override
    public void runOpMode() throws InterruptedException {
        drive= new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());


        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .forward(2).build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
               .strafeLeft(21).build();

        Trajectory traj10 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(32.4, 24, Math.toRadians(-60)))
                .build();


        Trajectory traj7 = drive.trajectoryBuilder(traj10.end())
                .forward(4.5).build();
        Trajectory traj9 = drive.trajectoryBuilder(traj7.end())
                //  .lineToSplineHeading(new Pose2d(24,20, Math.toRadians(0)))
                .back(4)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.ConeGrabber.setPosition(0);

        drive.followTrajectory(traj4);
        sleep(300);
        drive.followTrajectory(traj5);
        sleep(300);
       // drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);


        // drive.followTrajectory(traj6);
        //encoderDrive(1, 39);
        sleep(300);
       // drive.followTrajectory(turn);


        drive.motorLift.setPower(-1);
        sleep(1500);
        drive.motorLift.setPower(0);

        drive.followTrajectory(traj7);
        sleep(1500);
        drive.motorLift.setPower(0.1);
        sleep(750);
        drive.motorLift.setPower(0);
        drive.ConeGrabber.setPosition(0.4);



        //turn
        drive.followTrajectory(traj9);
        drive.turn(Math.toRadians(-145));
        drive.motorLift.setPower(0.4);
        sleep(300);
        drive.OdLift.setPosition(0.5);




        sleep(2000);
/*
        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180),
                                new TranslationalVelocityConstraint(MAX_VEL),
                                new ProfileAccelerationConstraint(MAX_ACC))
                        .build()
        )*/
    }
    /*
    public void encoderDrive(double liftPower, double liftInches) {
        int newArmTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            drive.motorLift.setMode(STOP_AND_RESET_ENCODER); //stop and reset encoders
            //drive.motorLift.setMode(RUN_USING_ENCODER); //start encoders


            newArmTarget = drive.motorLift.getCurrentPosition() + (int) (79.2 * liftInches);
            drive.motorLift.setTargetPosition(newArmTarget);


            // Turn On RUN_TO_POSITION
            drive.motorLift.setMode(RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //while (!IsInRange(drive.motorLift.getCurrentPosition(), newArmTarget)) {
                drive.motorLift.setPower(liftPower);
           // }
            drive.motorLift.setPower(0);

            //drive(Math.abs(leftSpeed), Math.abs(rightSpeed));
            // motorwg.setPower(auxSpeed);



            telemetry.update();
        }

    }
    */
     
    /*public boolean IsInRange(double inches, double target){
        final float DEAD_RANGE = 20;

        if(Math.abs(target - inches) <= DEAD_RANGE){
            return true;
        }
        return false;
    }*/
}
