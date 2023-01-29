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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "advanced")
public class Straight extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenColorV_2 openCv;
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();
    final double ARMTPI = 84.5;
    int highPoleTicks =(int) (ARMTPI* 35);;
    int topOfStack = (int) (ARMTPI* 5.25 ) ;;
    double waitTime = 0.3;
    int stackConesGrabbed = 0;
    enum State {
        START
    }
    State currentState = State.START;
    Pose2d startPose = new Pose2d(0, 0, 0);

    Pose2d Red = new Pose2d(50,22, Math.toRadians(-90));
    Pose2d Blue = new Pose2d(52,-1,Math.toRadians(-90));
    Pose2d Yellow = new Pose2d(52,-28,Math.toRadians(-90));
    Pose2d park;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
        openCv = new OpenColorV_2();
        openCv.OpenCv(hardwareMap, telemetry);

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



        Trajectory startingStrafe = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(72,0))
                .build();





        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectory(startingStrafe);
        drive.update();

            }

        }

