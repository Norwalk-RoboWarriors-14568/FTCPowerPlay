package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "advanced")
public class LeftRed extends LinearOpMode {
    SampleMecanumDrive drive;
    openCV33 openCv;
    Pose2d YELLOW = new Pose2d(58,-28, Math.toRadians(90));
    Pose2d BLUE = new Pose2d(58,-1,Math.toRadians(180));
    Pose2d RED = new Pose2d(58, 24.75, Math.toRadians(180));
    Pose2d park = new Pose2d();
    final double ARMTPI = 84.5;
    int highPoleTicks =(int) (ARMTPI * 35);
    int lowPoleTicks =(int) (ARMTPI * 15);
    int topOfStack = (int) (ARMTPI * 5.25);
    int mediumPole =  (int) (ARMTPI * 25);
    double waitTime = 0.3;
    int stackConesGrabbed = 0;
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();
    int conesOffStack = 0;
    enum State {
        START,
        GRAB_CONE_FROM_STACK,
        WAIT_1,
        WAIT_2,
        TO_BIG_POLE,
        DROP_CONE,
        IDLE,
        PIVOT_AT_STACK,
        WAIT_3,
        TO_STACK,
        MEDIUM_POLE,
        DROP_CONE_2,
        PIVOT_AT_STACK_2,
        TO_STACK_2,
        FAR_BIG_POLE,
        PARK
    }
    State currentState = State.START;
    Pose2d startPose = new Pose2d(0, 0.5, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        openCv = new openCV33();
        openCv.OpenCv(hardwareMap, telemetry);
        TrajectorySequence turnAndStrafe = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10,0,Math.toRadians(90)))

                .lineToLinearHeading(new Pose2d(45,7.5,Math.toRadians(90)))
                .build();

        TrajectorySequence getFirstCone = drive.trajectorySequenceBuilder(turnAndStrafe.end())
                .lineToLinearHeading(new Pose2d(58, 6.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(58, 29, Math.toRadians(90)))//Stack
                .build();

        TrajectorySequence pivotAtStack = drive.trajectorySequenceBuilder(getFirstCone.end())
                .lineToLinearHeading(new Pose2d(56, 21, Math.toRadians(0)))
                .build();

        TrajectorySequence toBigPole = drive.trajectorySequenceBuilder((pivotAtStack.end()))
                .lineToLinearHeading(new Pose2d(63.75, -13, Math.toRadians(0)))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder((toBigPole.end()))
                .lineToLinearHeading(new Pose2d(56, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(58, 25.5, Math.toRadians(90)))
                .build();


        TrajectorySequence toMediumPole = drive.trajectorySequenceBuilder((toStack.end()))
                //.lineToLinearHeading(new Pose2d(54, 21, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(58, 15, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(54.25, -17, Math.toRadians(180)))

                .build();

        TrajectorySequence toFarPole = drive.trajectorySequenceBuilder((toStack.end()))
                //.lineToLinearHeading(new Pose2d(54, 21, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(58, 15, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(54, -43.5, Math.toRadians(180)))

                .build();
        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder((toMediumPole.end()))
                .lineToLinearHeading(new Pose2d(58, 15, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(56.5, 26, Math.toRadians(90)))
                .build();
        drive.ConeGrabber.setPosition(0);

        drive.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.motorLift.setMode(RUN_USING_ENCODER);
        drive.motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis : ", openCv.pipeline.getAnalysis());
            telemetry.update();
            sleep(50);
        }
        int snapshotAnalysis = openCv.analysis();

        telemetry.addData("Snapshot post-START analysis : ", snapshotAnalysis);
        telemetry.update();

        if (isStopRequested()) return;

        switch (snapshotAnalysis)
        {
            case 1://BLUE
            {
                park = BLUE;
                break;
            }
            case 2://YELLOW
            {
                park = YELLOW;
                break;
            }
            default:
            {
                park = RED;
                break;
            }
        }
        drive.setPoseEstimate(startPose);
        TrajectorySequence Park = drive.trajectorySequenceBuilder(toFarPole.end())
                .lineToLinearHeading(park)
                .build();

        waitForStart();

        if (isStopRequested()) return;
        currentState = State.START;
        drive.followTrajectorySequenceAsync(turnAndStrafe);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case START:
                    armHeight(1, lowPoleTicks);
                    if (!drive.isBusy()) {
                        armHeight(1, lowPoleTicks - (int)(3.5 * 84.5));
                        drive.ConeGrabber.setPosition(0.4);//drop cone
                        drive.followTrajectorySequenceAsync(getFirstCone);
                        waitTimer.reset();
                        currentState = State.WAIT_1;
                    }
                    break;
                case WAIT_1:

                    if(waitTimer.seconds()>= waitTime) {
                        if ( conesOffStack ==3){
                            currentState = State.PARK;
                        } else {
                            if (conesOffStack == 2) {
                                drive.followTrajectorySequenceAsync(toStack2);

                            } else if (conesOffStack == 1) {
                                drive.followTrajectorySequenceAsync(toStack);
                            }
                            currentState = State.GRAB_CONE_FROM_STACK;
                        }
                    }

                    break;
                case GRAB_CONE_FROM_STACK:
                    armHeight(-1,topOfStack);
                    if (!drive.isBusy()) {
                        drive.ConeGrabber.setPosition(0);
                        conesOffStack++;
                        currentState = State.WAIT_2;

                        waitTimer.reset();
                    }
                    break;
                case WAIT_2:
                    if(waitTimer.seconds()>= waitTime) {
                        if (conesOffStack == 1) {
                            currentState = State.PIVOT_AT_STACK;
                        } else if (conesOffStack == 2){
                            currentState = State.MEDIUM_POLE;
                        } else if (conesOffStack == 3){
                            currentState = State.FAR_BIG_POLE;
                        }
                        armHeight(1, lowPoleTicks);
                        topOfStack-= (1.25 * 84.5);
                    }
                    break;
                case  PIVOT_AT_STACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(pivotAtStack);
                        currentState = State.TO_BIG_POLE;
                    }
                    break;
                case TO_BIG_POLE:
                    armHeight(1, highPoleTicks);
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(toBigPole);
                        waitTimer.reset();
                        currentState = State.DROP_CONE;
                    }
                    break;
                case FAR_BIG_POLE:

                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(toFarPole);
                        armHeight(1, highPoleTicks);
                        waitTimer.reset();
                        currentState = State.DROP_CONE;
                    }
                    break;
                case MEDIUM_POLE:
                    armHeight(1, mediumPole);
                    if (!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(toMediumPole);
                        waitTimer.reset();
                        currentState = State.DROP_CONE;
                    }
                    break;
                case DROP_CONE:
                    if (!drive.isBusy()) {
                        waitTimer.reset();
                        currentState = State.WAIT_1;
                        armHeight(-1, drive.motorLift.getCurrentPosition() - (int)(84.5 * 3.5));
                        drive.ConeGrabber.setPosition(0.4);


                    }
                    break;
                case TO_STACK:
                    armHeight(-1, topOfStack);
                    if(!drive.isBusy()){
                        currentState = State.GRAB_CONE_FROM_STACK;
                        drive.ConeGrabber.setPosition(0);

                    }
                    break;
                case PARK:
                    drive.followTrajectorySequenceAsync(Park);
                    currentState = State.IDLE;
                    break;
                case IDLE:
                    if (!drive.isBusy()) {
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