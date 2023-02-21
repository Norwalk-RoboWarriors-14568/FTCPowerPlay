package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "222")
public class LeftBlueMed extends LinearOpMode {
    SampleMecanumDrive drive;
    openCV33 openCv;
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();
    final double ARMTPI = 84.5;
    int highPoleTicks =(int) (ARMTPI* 35);;
    int topOfStack = (int) (ARMTPI* 5.25 ) ;;
    double waitTime = 0.3;
    int stackConesGrabbed = 0;
    enum State {
        START,
        FIRST_JUNCTION,
        WAIT_1,
        STACK,
        WAIT_2,
        POLE_RETURN,
        DROP_CONE,
        IDLE,
        SMALL_POLE,
        PARK
    }
    State currentState = State.START;
    Pose2d startPose = new Pose2d(0, 0, 0);

    Pose2d Yellow = new Pose2d(-49,0, Math.toRadians(0));
    Pose2d Blue = new Pose2d(-26,0,Math.toRadians(0));
    Pose2d Red = new Pose2d(-4,0,Math.toRadians(0));
    Pose2d park;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
        openCv = new openCV33();
        openCv.OpenCv(hardwareMap, telemetry);

        drive.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.motorLift.setMode(RUN_USING_ENCODER);
        drive.motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Trajectory startingStrafe = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(8,-24))
                .build();

        TrajectorySequence toFirstJunction = drive.trajectorySequenceBuilder(startingStrafe.end())
                .lineToLinearHeading(new Pose2d(43, -22.5, Math.toRadians(90)))
                // .lineToLinearHeading(new Pose2d(44, 18, Math.toRadians(-90)))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder((toFirstJunction.end()))
                .lineToLinearHeading(new Pose2d(55, -23.5, Math.toRadians(90)))

                // .lineToLinearHeading(new Pose2d(52.5, 11, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(56, 23, Math.toRadians(90)))
                .build();

        TrajectorySequence toBigPole = drive.trajectorySequenceBuilder(new Pose2d(0,0 ,0))
                //.lineToLinearHeading(new Pose2d(52.5, 6, Math.toRadians(-90)))
                //.lineToLinearHeading(new Pose2d(52, 10, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(55.75, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48,-2,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-42.75,3.5,Math.toRadians(45)))





                .build();

        TrajectorySequence toStackTwo = drive.trajectorySequenceBuilder((toBigPole.end()))
                //.lineToLinearHeading(new Pose2d(52.5, 11, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, -2, Math.toRadians(0)))

                .lineToLinearHeading(new Pose2d(0, -1, Math.toRadians(0)))

                .build();
/*
        TrajectorySequence toBigPoleTwo = drive.trajectorySequenceBuilder((toStackTwo.end()))
                .lineToLinearHeading(new Pose2d(52.5, 6, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(52, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(55.75, 10, Math.toRadians(0)))
                .build();
*/
        TrajectorySequence toSmallPole = drive.trajectorySequenceBuilder(toStack.end())
                .lineToLinearHeading(new Pose2d(47.1, -14.5, Math.toRadians(180)))
                .build();
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis : ", openCv.pipeline.getAnalysis());
            telemetry.update();
            sleep(50);
        }
        int snapshotAnalysis = openCv.analysis();
        drive.motorLift.setMode(STOP_AND_RESET_ENCODER);

        telemetry.addData("Snapshot post-START analysis : ", snapshotAnalysis);
        telemetry.update();

        if (isStopRequested()) return;

        switch (snapshotAnalysis)
        {
            case 1://BLUE
            {
                park = Blue;
                break;
            }
            case 2://YELLOW
            {
                park = Yellow;
                break;
            }
            default://RED
            {
                park = Red;
                break;
            }
        }



        TrajectorySequence Park = drive.trajectorySequenceBuilder(toBigPole.end())
                .lineToLinearHeading(new Pose2d(-48, -2, Math.toRadians(0)))

                // .lineToLinearHeading(new Pose2d(52.5, 11, Math.toRadians(0)))
                .lineToLinearHeading(park)
                .build();



        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectoryAsync(startingStrafe);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case START:
                    drive.ConeGrabber.setPosition(0);
                    if (!drive.isBusy()) {
                        currentState = State.FIRST_JUNCTION;
                        drive.followTrajectorySequenceAsync(toFirstJunction);
                    }
                    break;
                case FIRST_JUNCTION:
                    if (stackConesGrabbed == 0){
                        armHeight(1.0, (int) (28 * 84.5));

                    }else {
                        armHeight(1.0, (int) (25 * 84.5));
                    }
                    if (!drive.isBusy()) {
                        drive.ConeGrabber.setPosition(0.4);
                        sleep(100);

                        waitTimer.reset();

                        if (stackConesGrabbed == 0) {
                            drive.followTrajectorySequenceAsync(toStack);
                            currentState = State.WAIT_1;

                        } else  if (stackConesGrabbed <3 ) {
                            // currentState = State.;
                            currentState = State.WAIT_1;

                            drive.followTrajectorySequenceAsync(toStackTwo);
                        }else {
                            currentState = State.PARK;
                        }


                    }
                    break;
                case WAIT_1:
                    if(waitTimer.seconds()>= waitTime) {
                        if (stackConesGrabbed ==4) {
                            currentState = State.PARK;
                        } else {
                            currentState = State.STACK;
                        }
                    }
                    break;
                case STACK:
                    armHeight(-1,topOfStack);
                    if (!drive.isBusy()) {
                        drive.ConeGrabber.setPosition(0);
                        currentState = State.WAIT_2;
                        stackConesGrabbed++;
                        waitTimer.reset();
                    }
                    break;
                case WAIT_2:
                    if(waitTimer.seconds()>= waitTime) {
                        if (stackConesGrabbed < 4) {
                            currentState = State.POLE_RETURN;
                        } else {
                            currentState = State.PARK;
                        }
                        topOfStack-= (1.25 * ARMTPI);
                    }
                    break;
                case SMALL_POLE:
                    armHeight(1, (int)84.5 * 14);
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(toSmallPole);
                        currentState = State.DROP_CONE;
                    }
                    break;
                case POLE_RETURN:
                    armHeight(1, (int)(27*84.5));
                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(startPose);

                        drive.followTrajectorySequenceAsync(toBigPole);
                        currentState = State.FIRST_JUNCTION;
                    }
                    break;
                case DROP_CONE:
                    if (!drive.isBusy()) {
                        armHeight(-1, drive.motorLift.getCurrentPosition() - (int) (84.5 * 3.5));
                        sleep(100);
                        drive.ConeGrabber.setPosition(0.4);

                        waitTimer.reset();
                        currentState = State.WAIT_1;
                    }
                    break;
                case PARK:
                    drive.followTrajectorySequenceAsync(Park);
                    currentState = State.IDLE;
                    break;
                case IDLE:
                    if (!drive.isBusy()) {
                        armHeight(-1,0);
                    }
                    break;
            }
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("Lift POSE", drive.motorLift.getCurrentPosition());
            telemetry.addData("Lift TARGET POSE", drive.motorLift.getTargetPosition());
            telemetry.addData("topOfStack", topOfStack);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void armHeight(double armSpeed, int newArmTarget) {
        drive.motorLift.setTargetPosition(newArmTarget);
        drive.motorLift.setPower(armSpeed);
        drive.motorLift.setMode(RUN_TO_POSITION);
    }
}