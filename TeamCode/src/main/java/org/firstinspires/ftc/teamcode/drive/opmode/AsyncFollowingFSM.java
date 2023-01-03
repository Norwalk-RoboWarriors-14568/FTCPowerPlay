package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Autonomous(group = "advanced")
public class AsyncFollowingFSM extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenColorV_2 openCv;
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory// Then we want to do a point turn
        WAIT_1,
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_2,
        TRAJECTORY_4,         // Then we're gonna wait a second
        TRAJECTORY_5,         // Finally, we're gonna turn again
        IDLE,
        PARK// Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        drive= new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        openCv = new OpenColorV_2();
        openCv.OpenCv(hardwareMap, telemetry);
        Pose2d Rewd = new Pose2d(50,19, Math.toRadians(-90));
        Pose2d Blew = new Pose2d(52,1,Math.toRadians(-90));
        Pose2d Ygren = new Pose2d(52,-28,Math.toRadians(-90));
        Pose2d park = new Pose2d();
        int highPoleticks =(int) (84.5* 35);;
        int topOfStack = (int) (84.5* 5.25 ) ;;
        double waitTime = 0.3;
        int stackConesGrabbed = 0;
        ElapsedTime waitTimer = new ElapsedTime();
        ElapsedTime matchTimer = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int i = 0;
        drive.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.motorLift.setMode(RUN_USING_ENCODER);
        drive.motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        // Initialize our lift

        // Initialize SampleMecanumDrive

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(8,20))
                .build();

        // Second trajectory
        // Ensure that we call turnAndStrafe.end() as the start for this one
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(48, 20, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(55.75, 10, Math.toRadians(0)))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder((trajectory2.end()))
                .lineToLinearHeading(new Pose2d(52.5, 11, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(54, -29, Math.toRadians(-90)))
                .build();

        TrajectorySequence toBigPole = drive.trajectorySequenceBuilder((toStack.end()))
                .lineToLinearHeading(new Pose2d(52.5, 6, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(52, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(55.75, 10, Math.toRadians(0)))
                .build();

        TrajectorySequence Park = drive.trajectorySequenceBuilder(toBigPole.end())
                .lineToLinearHeading(new Pose2d(52.5, 11, Math.toRadians(0)))
                .lineToLinearHeading(park)
                .build();

        // Define the angle to turn at
       // double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call getFirstCone.end()
        // Since there was a point turn before that
        // So we just take the pose from getFirstCone.end(), add the previous turn angle to it
       // Pose2d newLastPose = getFirstCone.end().plus(new Pose2d(0, 0, turnAngle1));

        // Define a 1.5 second wait time
        /*
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);
        */
        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to START, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    drive.ConeGrabber.setPosition(0);
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    armHeight(1,highPoleticks);
                    if (!drive.isBusy()) {
                        drive.ConeGrabber.setPosition(0.4);//drop cone
                        if (stackConesGrabbed < 3) {
                            waitTimer.reset();
                            currentState = State.WAIT_1;
                            drive.followTrajectorySequenceAsync(toStack);
                        }
                        else currentState = State.PARK;
                    }
                    break;
                case WAIT_1:
                    if(waitTimer.seconds()>= waitTime) {
                        currentState = State.TRAJECTORY_3;
                    }
                    break;
                case TRAJECTORY_3:
                    armHeight(-1,topOfStack);
                    if (!drive.isBusy()) {
                        drive.ConeGrabber.setPosition(0);//this grabs the cone off of the top of the stack
                        currentState = State.WAIT_2;
                        stackConesGrabbed++;
                        waitTimer.reset();
                    }
                    break;
                case WAIT_2:
                    if(waitTimer.seconds()>= waitTime) {
                        currentState = State.TRAJECTORY_4;
                        topOfStack-= (1.25 * 84.5);
                    }
                    break;
                case TRAJECTORY_4:
                    armHeight(1,highPoleticks);
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(toBigPole);
                        currentState = State.TRAJECTORY_2;
                    }
                    break;
                case TRAJECTORY_5:
                    if (!drive.isBusy()) {
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

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("Lift POSE", drive.motorLift.getCurrentPosition());
            telemetry.addData("Lift TARGET POSE", drive.motorLift.getTargetPosition());
            telemetry.addData("topOfStack", topOfStack);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
    }

    public void armHeight(double armSpeed, int newArmTarget) {
       // drive.motorLift.setMode(RUN_USING_ENCODER);
        //int newArmTarget = drive.motorLift.getCurrentPosition() + (int) (84.5* armInches);
        drive.motorLift.setTargetPosition(newArmTarget);
        drive.motorLift.setPower(armSpeed);
        drive.motorLift.setMode(RUN_TO_POSITION);
        /*if (IsInRange(armInches, newArmTarget)){
            drive.motorLift.setPower(0);

        }*/

    }
    public boolean IsInRange(double inches, double target){
        final float DEAD_RANGE = 20;

        if(Math.abs(target - inches) <= DEAD_RANGE){
            return true;
        }
        return false;
    }
}