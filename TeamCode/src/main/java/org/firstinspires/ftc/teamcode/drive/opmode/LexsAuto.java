package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

    //@Autonomous(group = "auto")
//@Disabled
    @Config
    public class LexsAuto extends LinearOpMode {

        SampleMecanumDrive drive;
        OpenColorV_2 openCv;

        //private Servo ConeGrabber, OdLift;
        @Override
        public void runOpMode() throws InterruptedException {
            drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(new Pose2d());
            openCv = new OpenColorV_2();
            openCv.OpenCv(hardwareMap, telemetry);
            Pose2d Rewd = new Pose2d(50, 15, Math.toRadians(180));
            Pose2d Blew = new Pose2d(52, -9, Math.toRadians(180));
            Pose2d Ygren = new Pose2d(52, -33, Math.toRadians(180));
            Pose2d park = new Pose2d();

            while (!isStarted() && !isStopRequested()) {
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


            switch (snapshotAnalysis) {

                case 1://BLUE
                {
                    park = Blew;
                    break;
                }
                case 2://YELLOW
                {
                    park = Ygren;
                    break;
                }
                default:
                    park = Rewd;
                    break;
            }
            GO(0.5, 10, false);
        }


        public void GO(double speed, double distance, boolean strafe) {
            RUE();
            int newDistanceTarget = drive.motorLift.getCurrentPosition() + (int) (84.5 * distance);
            setTargets(newDistanceTarget, strafe);
            setDriveSpeed(speed);
            RTP();

        }

        public void armHeight(double armSpeed, double armInches) {
            int newArmTarget = drive.motorLift.getCurrentPosition() + (int) (84.5 * armInches);
            drive.motorLift.setTargetPosition(newArmTarget);
            drive.motorLift.setPower(armSpeed);
            drive.motorLift.setMode(RUN_TO_POSITION);


        }
        public void setTargets(int target, boolean strafe){
            if (strafe){
                drive.leftFront.setTargetPosition(-target);
                drive.rightFront.setTargetPosition(target);
                drive.leftRear.setTargetPosition(target);
                drive.rightRear.setTargetPosition(-target);
            }else {
                drive.leftFront.setTargetPosition(target);
                drive.rightFront.setTargetPosition(target);
                drive.leftRear.setTargetPosition(target);
                drive.rightRear.setTargetPosition(target);
            }
        }
        public void setDriveSpeed(double speed){
            drive.leftFront.setPower(speed);
            drive.leftRear.setPower(speed);
            drive.rightFront.setPower(speed);
            drive.rightRear.setPower(speed);

        }
        public void RTP(){
            drive.leftFront.setMode(RUN_TO_POSITION);
            drive.leftRear.setMode(RUN_TO_POSITION);
            drive.rightRear.setMode(RUN_TO_POSITION);
            drive.rightFront.setMode(RUN_TO_POSITION);

        }
        public void RUE(){
            drive.leftFront.setMode(RUN_USING_ENCODER);
            drive.leftRear.setMode(RUN_USING_ENCODER);
            drive.rightFront.setMode(RUN_USING_ENCODER);
            drive.rightRear.setMode(RUN_USING_ENCODER);

        }

        public boolean IsInRange(double inches, double target) {
            final float DEAD_RANGE = 20;

            if (Math.abs(target - inches) <= DEAD_RANGE) {
                return true;
            }
            return false;
        }
}

