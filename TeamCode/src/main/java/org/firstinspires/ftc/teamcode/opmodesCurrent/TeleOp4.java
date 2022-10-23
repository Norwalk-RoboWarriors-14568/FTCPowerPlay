package org.firstinspires.ftc.teamcode.opmodesCurrent;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

//import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * All the code aside from importing, goes within a class file - essentially telling Android Studio-
 * to run this using java
 */
@TeleOp(name = "TeleOp")


public class TeleOp4 extends OpMode {


//*********** Experimental          Mecanums only
//*********** Rev Hub count: 2


    //TETRIX Motors        -recording the motor type as if this file ran autonomous

    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorLift;

    private Servo ConeGrabber;
    //private  servoCrFlapper;
    //private CRServo ConeGrabber;
    private boolean mecanumDriveMode = true, coastMotors = true;
    private float mecanumStrafe = 0, dominantXJoystick = 0;


    /*
     * Code to run when the op mode is first enabled goes here
     * This is all automatic- it prepares the robot to run loop() without error
     */
    @Override
    public void init() {


//rev hub 1
        motorLeft = hardwareMap.dcMotor.get("leftFront");
        motorRight = hardwareMap.dcMotor.get("rightFront");
        motorLeft2 = hardwareMap.dcMotor.get("leftRear");
        motorRight2 = hardwareMap.dcMotor.get("rightRear");

        motorLift = hardwareMap.dcMotor.get("lift");


        //motorFlapper = hardwareMap.dcMotor.get("motor_8");
        ConeGrabber = hardwareMap.servo.get("ConeGrabber");
        //ConeGrabber.scaleRange(0.0,1.0 );

        //servoCrFlapper = hardwareMap.crservo.get("servo_0");

        //so you don't have to wire red to black, to maintain program logic
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        //   motorShoulder.setDirection(DcMotor.Direction.REVERSE);
        //motorLift.setDirection(DcMotor.Direction.REVERSE); //(up makes the motor go negative in the code)
        //motorRail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //telemetry sends data to print onto both phones
        telemetry.addLine("Drive Base TeleOp\nInit Opmode");
    }


    /**
     * The next 2 lines ends the current state of Opmode, and starts Opmode.loop()
     * This enables control of the robot via the gamepad or starts autonomous functions
     */
    @Override
    public void loop() {

        telemetry.addLine("Loop OpMode\ndPadLeft: disable mecanum strafe is "+ !mecanumDriveMode +
                "\ndPadRight: enable mecanum strafe is "+ mecanumDriveMode +
                "\nX: brake motors is "+ !coastMotors +"\nY: coast motors is "+ coastMotors);

<<<<<<< Updated upstream
<<<<<<< Updated upstream
        telemetry.addData("LeftMTR  PWR: ", "50");
        telemetry.addData("RightMTR PWR: ", "50");
=======
        telemetry.addData("LeftMTR  PWR: ", motorLeft.getPower());
        telemetry.addData("RightMTR PWR: ", motorRight.getPower());
        telemetry.addData("Servo", ConeGrabber.getPosition());
>>>>>>> Stashed changes
=======
        telemetry.addData("LeftMTR  PWR: ", motorLeft.getPower());
        telemetry.addData("RightMTR PWR: ", motorRight.getPower());
        telemetry.addData("Servo", ConeGrabber.getPosition());
>>>>>>> Stashed changes
//gamepad1          -specifying the section of code giving control to gamepad1-this reduces confusion
        //else if's are required, so that a motor doesn't receive the power of multiple lines


        //allows for 3-speed joystick control

        if (abs(gamepad1.left_stick_x) > 0.15 || abs(gamepad1.right_stick_x) > 0.15) {
            //removes negatives from joystick values, to set variable to +/- for determing stick farther from zero
            dominantXJoystick = (abs(gamepad1.left_stick_x) - abs(gamepad1.right_stick_x));
            mecanumDriveMode = true;
        } else {
            mecanumDriveMode = false;
        }

        if (mecanumDriveMode) {     //when enabled, motors will only hit 100% when strafing and driving

            if (dominantXJoystick > 0) {
                mecanumStrafe = gamepad1.left_stick_x;
            } else if (dominantXJoystick < 0) {
                mecanumStrafe = gamepad1.right_stick_x;
            }

            if (gamepad1.left_bumper) {
<<<<<<< Updated upstream
<<<<<<< Updated upstream
                motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe*0.01)); // previously 2
                motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe*0.01));
                motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe*0.01));
                motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe*0.01));
            } else if (gamepad1.right_bumper) {
                motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe*0.1)); // previously 2 * .5
                motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe*0.1));
                motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe*0.1));
                motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe*0.1));
            } else {
                motorLeft.setPower((gamepad1.left_stick_y *0.5 + -mecanumStrafe)); // previously 2 * .75
                motorLeft2.setPower((gamepad1.left_stick_y *0.5 + mecanumStrafe));
                motorRight.setPower((gamepad1.right_stick_y *0.5 + mecanumStrafe));
                motorRight2.setPower((gamepad1.right_stick_y * 0.5+ -mecanumStrafe));
=======
=======
>>>>>>> Stashed changes
                motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe) / 3.0); // previously 2
                motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe) / 3.0);
                motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe) / 3.0);
                motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe) / 3.0);
            } else if (gamepad1.right_bumper) {
                motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe) / 1.5); // previously 2 * .5
                motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe) / 1.5);
                motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe) / 1.5);
                motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe) / 1.5);
            } else {
                motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe) / 2.25); // previously 2 * .75
                motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe) / 2.25);
                motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe) / 2.25);
                motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe) / 2.25 );
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
            }
        } else if (!mecanumDriveMode ) {
            if (gamepad1.left_bumper) {
                drive(gamepad1.left_stick_y * 0.8, gamepad1.right_stick_y * 0.8 );
            } else if (gamepad1.right_bumper) {
                drive(gamepad1.left_stick_y * 0.25, gamepad1.right_stick_y * 0.25);
            } else {
<<<<<<< Updated upstream
<<<<<<< Updated upstream
                drive(gamepad1.left_stick_y* 0.10, gamepad1.right_stick_y* 0.10);       /////main speed
=======
                drive(gamepad1.left_stick_y * 0.5, gamepad1.right_stick_y * 0.5);
>>>>>>> Stashed changes
=======
                drive(gamepad1.left_stick_y * 0.5, gamepad1.right_stick_y * 0.5);
>>>>>>> Stashed changes
            }
        }

        //button code to manipulate other code/robot
        if (gamepad1.x) {
            motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            coastMotors = false;
        } else if (gamepad1.y) {
            motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            coastMotors = true;
        } else if (gamepad1.dpad_left) {
            mecanumDriveMode = false;
        } else if (gamepad1.dpad_right) {
            mecanumDriveMode = true;
        }

//gamepad2

        //problematic as telemetry is changed when gamepad 1 x or y is pressed

        if (gamepad2.right_stick_y >0.2 || gamepad2.right_stick_y < -0.2 ) {
            motorLift.setPower(gamepad2.right_stick_y);
        }else {
            motorLift.setPower(0);
        }
        if (gamepad2.left_bumper) {
            ConeGrabber.setPosition(0);

        }else if (gamepad2.right_bumper){
            ConeGrabber.setPosition(0.4);
            //i = i + 0.01;//ConeGrabber.setPosition(0.5);
        }

        // motorShoulder.setPower(gamepad2.left_stick_y); //it's already reversed direction above:
        //travels in the same direcion as rail pivot, and mounted as translation of rightDriveMotors
        // motorRail.setPower(gamepad2.right_stick_y); //opposite is true with the shoulder

        //servoMarkerCR.setPower(gamepad2.right_stick_y34


//end of loop opmode programing
    }


    @Override
    public void stop() {
        telemetry.clearAll();
        telemetry.addLine("Stopped");
    }

    public void drive(double left, double right) {
        motorLeft.setPower(left);
        motorLeft2.setPower(left);
        motorRight.setPower(right);
        motorRight2.setPower(right);

    }

}

