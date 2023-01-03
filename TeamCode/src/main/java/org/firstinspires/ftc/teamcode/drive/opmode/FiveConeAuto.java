package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Autonomous(group = "auto")
//@Disabled
@Config
public class FiveConeAuto extends LinearOpMode {
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
        Pose2d Rewd = new Pose2d(50,15, Math.toRadians(180));
        Pose2d Blew = new Pose2d(52,-9,Math.toRadians(180));
        Pose2d Ygren = new Pose2d(52,-33,Math.toRadians(180));
        Pose2d park = new Pose2d();

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

        //drive.ConeGrabber.setPosition(0);

        drive.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.motorLift.setMode(RUN_USING_ENCODER);
        drive.motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        switch (snapshotAnalysis)
        {
            
            case 1://BLUE
            {
                park = Blew;
                break;
            }
            case 2://YELLOW
            {
                park = Ygren;
                break;
            } default:
                park = Rewd;
                break;
        }

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> drive.ConeGrabber.setPosition(0))

                .lineToConstantHeading(new Vector2d(2,20))
                .addTemporalMarker(3,() -> armHeight(1,35))
                .lineToLinearHeading(new Pose2d(42, 15, Math.toRadians(90))).waitSeconds(0.3)
                .forward(6)//.waitSeconds(1.5)
                .addTemporalMarker(() -> drive.ConeGrabber.setPosition(0.4))
                .lineToLinearHeading(new Pose2d(52, 15, Math.toRadians(270)))
                .addTemporalMarker(() -> armHeight(-1,-20))
                //Experiment
                //.lineToLinearHeading(new Pose2d(55, 26, Math.toRadians(270)))
                //.addTemporalMarker(-1,() -> armHeight(1,7))
               // .lineToConstantHeading(new Vector2d(55,28))
               // .addTemporalMarker(() -> drive.ConeGrabber.setPosition(0.4))
                //.addTemporalMarker(15,() -> armHeight(1,35))
                //.lineToLinearHeading(new Pose2d(42, 15, Math.toRadians(90))).waitSeconds(0.3)
                //Experiment
                .lineToLinearHeading(park)
                .build();

        drive.followTrajectorySequence(trajSeq);

     //   sleep(2000);
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
    public void armHeight(double armSpeed, double armInches) {
        int newArmTarget = drive.motorLift.getCurrentPosition() + (int) (84.5* armInches);
        drive.motorLift.setTargetPosition(newArmTarget);
        drive.motorLift.setPower(armSpeed);
        drive.motorLift.setMode(RUN_TO_POSITION);


    }
    public boolean IsInRange(double inches, double target){
        final float DEAD_RANGE = 20;

        if(Math.abs(target - inches) <= DEAD_RANGE){
            return true;
        }
        return false;
    }
}
