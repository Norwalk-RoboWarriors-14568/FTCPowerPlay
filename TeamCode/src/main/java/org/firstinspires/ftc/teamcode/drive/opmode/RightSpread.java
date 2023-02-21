package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "222")
public class RightSpread extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenColorV_2 openCv;
    Pose2d RED = new Pose2d(52,20, Math.toRadians(-90));
    Pose2d BLUE = new Pose2d(52,-6,Math.toRadians(-90));
    Pose2d YELLOW = new Pose2d(52, -26, Math.toRadians(-90));
    Pose2d park = new Pose2d();
    final double ARMTPI = 84.5;
    int highPoleTicks =(int) (ARMTPI * 35);
    int lowPoleTicks =(int) (ARMTPI * 15);
    int topOfStack = (int) (ARMTPI * 5.25);
    int mediumPole =  (int) (ARMTPI * 25);
    double waitTime = 0.4;
    int stackConesGrabbed = 0;
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();
    int conesOffStack = 0;
    enum State {
        START,
        WAIT_1,
        WAIT_2,
        GO_FORWARD,
        GRAB_CONE_FROM_STACK,
        TO_STACK,
        TO_SMALL_RIGHT_POLE,
        TO_BIG_POLE,
        TO_MEDIUM_POLE,
        DROP_CONE,
        IDLE,
        PIVOT_AT_STACK,
        WAIT_3,
        MEDIUM_POLE,
        DROP_CONE_2,
        PIVOT_AT_STACK_2,
        TO_STACK_2,
        FAR_BIG_POLE,
        PARK
    }
    State currentState = State.START;
    Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        openCv = new OpenColorV_2();
        openCv.OpenCv(hardwareMap, telemetry);
        TrajectorySequence leftSmallPole = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(11,3,Math.toRadians(44)))
                .build();
        TrajectorySequence toStackTile = drive.trajectorySequenceBuilder(leftSmallPole.end())
                .lineToLinearHeading(new Pose2d(8,-2,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(57,-5.5,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52,3.5,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(52,-30,Math.toRadians(-90)))
                .build();

        TrajectorySequence toSmallPole = drive.trajectorySequenceBuilder(toStackTile.end())
                .lineToLinearHeading(new Pose2d(52,-5,Math.toRadians(-90)))
                //.lineToLinearHeading(new Pose2d(52,-5,Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(46,-11,Math.toRadians(-133)))


                // .lineToLinearHeading(new Pose2d(53.5,-30,Math.toRadians(-90)))
                .build();
        TrajectorySequence returnToStack = drive.trajectorySequenceBuilder(toSmallPole.end())
                .lineToLinearHeading(new Pose2d(52,-5,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(52,-30.5,Math.toRadians(-90)))

                .build();
        TrajectorySequence toMediumPole = drive.trajectorySequenceBuilder(returnToStack.end())
                .lineToLinearHeading(new Pose2d(52,18,Math.toRadians(-90)))
               // .lineToLinearHeading(new Pose2d(52,19,Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(46.25,11.25,Math.toRadians(-135)))
                        .build();

        TrajectorySequence getThirdCone = drive.trajectorySequenceBuilder(toMediumPole.end())
                .lineToLinearHeading(new Pose2d(52,19,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(52,-30.5,Math.toRadians(-90)))

                .build();
        TrajectorySequence toBigPole = drive.trajectorySequenceBuilder(getThirdCone.end())
                .lineToLinearHeading(new Pose2d(52,18,Math.toRadians(-90)))

                //.lineToLinearHeading(new Pose2d(52,19,Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(56.5,12, Math.toRadians(-45)))
                .build();


        drive.ConeGrabber.setPosition(0);

        drive.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.motorLift.setMode(RUN_USING_ENCODER);
        drive.motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis : ", openCv.pipeline.getAnalysis());
            telemetry.addData( "Distance (INCHES)", drive.distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(50);
        }
        int snapshotAnalysis = openCv.analysis();

        telemetry.addData("Snapshot post-START analysis : ", snapshotAnalysis);
        telemetry.update();
        drive.motorLift.setMode(STOP_AND_RESET_ENCODER);

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
       TrajectorySequence Park = drive.trajectorySequenceBuilder(getThirdCone.end())
             //  .lineToLinearHeading(new Pose2d(52,18,Math.toRadians(-90)))
               .lineToLinearHeading(park)
              .build();

        waitForStart();

        if (isStopRequested()) return;
        currentState = State.START;
        drive.followTrajectorySequenceAsync(leftSmallPole);


            while (opModeIsActive() && !isStopRequested()) {
                switch (currentState) {
                    case START:
                        armHeight(1, lowPoleTicks);
                        if (!drive.isBusy()) {
                           GOTOPOLE(true);
                            //armHeight(1, lowPoleTicks - (int)(1 * 84.5));

                            drive.ConeGrabber.setPosition(0.4);//drop cone
                            // armHeight(1, lowPoleTicks );

                            //drive.followTrajectorySequenceAsync(toStackTile);
                            waitTimer.reset();
                            currentState = State.WAIT_1;
                        }
                        break;
                    case WAIT_1:

                        if (waitTimer.seconds() >= waitTime) {

                            if (conesOffStack == 3) {
                              //  currentState = RightBlue.State.PARK;
                            } else {
                                if (conesOffStack == 1) {

                                } else if (conesOffStack == 0) {
                                    // armHeight(1, drive.motorLift.getCurrentPosition() + (int) (84.5 * 5.5));
                                    drive.followTrajectorySequenceAsync(toStackTile);

                                }
                                waitTimer.reset();
                                currentState = State.GRAB_CONE_FROM_STACK;
                            }
                        }

                        break;
                    case GRAB_CONE_FROM_STACK:

                        if (waitTimer.seconds() >= waitTime) {
                            armHeight(-1, topOfStack);
                            if (!drive.isBusy()) {
                                drive.ConeGrabber.setPosition(0);
                                conesOffStack++;

                                    currentState = State.WAIT_2;

                                waitTimer.reset();
                            }
                        }

                        break;
                    case WAIT_2:
                        if (waitTimer.seconds() >= waitTime) {
                            if (conesOffStack == 1) {
                                currentState = State.TO_SMALL_RIGHT_POLE;

                                //  currentState = RightBlue.State.PIVOT_AT_STACK;
                            } else if (conesOffStack == 2) {

                                  currentState = State.TO_MEDIUM_POLE;

                                //  currentState = RightBlue.State.MEDIUM_POLE;
                            } else if (conesOffStack == 3) {
                                 currentState = State.PARK;
                            }
                            armHeight(1, lowPoleTicks);
                            topOfStack -= (1.25 * 84.5);
                        }
                        break;
                    case TO_SMALL_RIGHT_POLE:
                        armHeight(1,lowPoleTicks);
                        if (!drive.isBusy()){
                            drive.followTrajectorySequenceAsync(toSmallPole);
                            waitTimer.reset();
                            currentState = State.DROP_CONE;
                        }
                        break;
                    case PIVOT_AT_STACK:
                        if (!drive.isBusy()) {
                          //  drive.followTrajectorySequence(pivotAtStack);
                           // currentState = RightBlue.State.TO_BIG_POLE;
                        }
                        break;
                    case TO_BIG_POLE:
                        armHeight(1, highPoleTicks);
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(toBigPole);
                            waitTimer.reset();
                            currentState = State.DROP_CONE;
                           // drive.followTrajectorySequenceAsync(toBigPole);
                            //currentState = RightBlue.State.DROP_CONE;
                        }
                        break;
                    case TO_MEDIUM_POLE:
                        armHeight(1.0, (int) (27.0 * 84.5));
                        if (!drive.isBusy()){
                            drive.followTrajectorySequenceAsync(toMediumPole);
                            waitTimer.reset();

                            currentState = State.DROP_CONE;
                        }
                        break;
                    case FAR_BIG_POLE:

                        if (!drive.isBusy()) {
                          //  drive.followTrajectorySequenceAsync(toFarPole);
                            armHeight(1, highPoleTicks);
                            waitTimer.reset();
                           // currentState = RightBlue.State.DROP_CONE;
                        }
                        break;
                    case MEDIUM_POLE:
                        armHeight(1, mediumPole);
                        if (!drive.isBusy()) {
                           // drive.followTrajectorySequenceAsync(toMediumPole);
                            waitTimer.reset();
                            currentState =State.DROP_CONE;
                        }
                        break;
                    case DROP_CONE:
                        if (!drive.isBusy()) {
                            waitTimer.reset();
                            if (conesOffStack == 3) {
                                GOTOPOLE(true);

                                //armHeight(-1, drive.motorLift.getCurrentPosition() + (int) (84.5 * 3.5));

                                currentState = State.PARK;
                            }else {
                                GOTOPOLE(true);
                                currentState = State.TO_STACK;

                               // armHeight(-1, drive.motorLift.getCurrentPosition() - (int) (84.5 * 3.5));

                            }
                            drive.ConeGrabber.setPosition(0.4);



                        }
                        break;
                    case TO_STACK:
                        armHeight(-1, topOfStack);
                        if (!drive.isBusy()) {
                            if (conesOffStack == 1) {
                                drive.followTrajectorySequenceAsync(returnToStack);
                            } else if (conesOffStack == 2) {
                                drive.followTrajectorySequenceAsync(getThirdCone);
                            }
                            currentState = State.GRAB_CONE_FROM_STACK;
                            waitTimer.reset();
                        }
                        break;

                    case PARK:
                        drive.followTrajectorySequenceAsync(Park);


                        currentState = State.IDLE;
                        break;
                    case IDLE:

                        if (!drive.isBusy()) {
                            armHeight(-1.0,0);
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
                telemetry.addData( "Distance (INCHES)", drive.distance.getDistance(DistanceUnit.INCH));
            //    telemetry.update();
                telemetry.update();

        }
    }
    public void GOTOPOLE(boolean turnRight){
     int i =0;
        while (drive.distance.getDistance(DistanceUnit.INCH) > 10 &&  turnRight && i < 1){
            drive.turn(Math.toRadians(-5));
            i++;
        }
        while (drive.distance.getDistance(DistanceUnit.INCH) > 10 && !turnRight && i < 1){
            drive.turn(Math.toRadians(5));
            i++;
        }
    }

    public void armHeight(double armSpeed, int newArmTarget) {
        drive.motorLift.setTargetPosition(newArmTarget);
        drive.motorLift.setPower(armSpeed);
        drive.motorLift.setMode(RUN_TO_POSITION);
    }

}