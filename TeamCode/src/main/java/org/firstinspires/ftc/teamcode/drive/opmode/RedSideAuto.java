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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "auto")
@Config
public class RedSideAuto extends LinearOpMode {
    public static double MAX_VEL = 100;
    public static double MAX_ACC = 20;
    SampleMecanumDrive drive;
    OpenColorV_2 openCv;

    //private Servo ConeGrabber, OdLift;
    @Override
    public void runOpMode() throws InterruptedException {
        drive= new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        openCv = new OpenColorV_2();
        openCv.OpenCv(hardwareMap, telemetry);
        Vector2d vector1 = new Vector2d(30,27);

       // Trajectory trajV1 = drive.trajectoryBuilder()
/*

        TrajectorySequence trajSeqRED = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2).waitSeconds(0.3)
                .strafeLeft(21).waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(34.9, 24, Math.toRadians(-60))).waitSeconds(0.3)
                .addTemporalMarker(() -> drive.motorLift.setPower(-1)).waitSeconds(1.5)
                .addTemporalMarker(() -> drive.motorLift.setPower(0))
                .forward(3.25).waitSeconds(1.5)
                .addTemporalMarker(() -> drive.motorLift.setPower(0.1)).waitSeconds(0.75)
                .addTemporalMarker(() -> drive.motorLift.setPower(0))
                .addTemporalMarker(() -> drive.ConeGrabber.setPosition(0.4))
                .back(4)
                .lineToSplineHeading(new Pose2d(28, 18, Math.toRadians(-180))).waitSeconds(0.3)
                .addTemporalMarker(() -> drive.OdLift.setPosition(0.5))
                .build();
*/
        TrajectorySequence trajSeqBLUE = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2).waitSeconds(0.3)
                .splineToConstantHeading(new Vector2d(2,20.7), Math.toRadians(0)).waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(32.15, 28.5, Math.toRadians(-56))).waitSeconds(0.3)
                .addTemporalMarker(() -> drive.motorLift.setPower(-1)).waitSeconds(1.5)
                .addTemporalMarker(() -> drive.motorLift.setPower(0))
                .forward(5.5).waitSeconds(1.5)
                .addTemporalMarker(() -> drive.motorLift.setPower(0.1)).waitSeconds(0.75)
                .addTemporalMarker(() -> drive.motorLift.setPower(0))
                .addTemporalMarker(() -> drive.ConeGrabber.setPosition(0.4))
                .back(4)
                .turn(Math.toRadians(-120)).lineToConstantHeading(new Vector2d(30, 27)).waitSeconds(0.3)
                .addTemporalMarker(() -> drive.OdLift.setPosition(0.5))
                .build();
/*
        TrajectorySequence trajSeqYELLOW = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2).waitSeconds(0.3)
                .strafeLeft(21).waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(34.9, 24, Math.toRadians(-60))).waitSeconds(0.3)
                .addTemporalMarker(() -> drive.motorLift.setPower(-1)).waitSeconds(1.5)
                .addTemporalMarker(() -> drive.motorLift.setPower(0))
                .forward(3.25).waitSeconds(1.5)
                .addTemporalMarker(() -> drive.motorLift.setPower(0.1)).waitSeconds(0.75)
                .addTemporalMarker(() -> drive.motorLift.setPower(0))
                .addTemporalMarker(() -> drive.ConeGrabber.setPosition(0.4))
                .back(4)
                .lineToSplineHeading(new Pose2d(28, 18, Math.toRadians(-180))).waitSeconds(0.3)
                .addTemporalMarker(() -> drive.OdLift.setPosition(0.5))
                .build();
*/

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis : ", openCv.pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        int snapshotAnalysis = openCv.analysis();

        telemetry.addData("Snapshot post-START analysis : ", snapshotAnalysis);
        telemetry.update();

        if (isStopRequested()) return;

        drive.ConeGrabber.setPosition(0);
/*
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

*/
        switch (snapshotAnalysis)
        {
            case 0://RED
            {
                drive.followTrajectorySequence(trajSeqBLUE);
                break;
            }

            case 1://BLUE
            {
                drive.followTrajectorySequence(trajSeqBLUE);
                break;
            }

            case 2://YELLOW
            {
                drive.followTrajectorySequence(trajSeqBLUE);
                break;
            }
        }


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
    public void armHeight(double armSpeed, double armInches) {
        int newArmTarget = drive.motorLift.getCurrentPosition() + (int) (85.44444444444444 * armInches);
        drive.motorLift.setPower(armSpeed);
        while (opModeIsActive() && !IsInRange(drive.motorLift.getCurrentPosition(), newArmTarget)){
            telemetry.addData("Target Left: ", newArmTarget);
            telemetry.addData("Current Pos Right:", drive.motorLift.getCurrentPosition());
            telemetry.addData("left power: ", drive.motorLift.getPower());
            telemetry.update();
        }
        drive.motorLift.setPower(0);
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

    public boolean IsInRange(double inches, double target){
        final float DEAD_RANGE = 20;

        if(Math.abs(target - inches) <= DEAD_RANGE){
            return true;
        }
        return false;
    }
}
