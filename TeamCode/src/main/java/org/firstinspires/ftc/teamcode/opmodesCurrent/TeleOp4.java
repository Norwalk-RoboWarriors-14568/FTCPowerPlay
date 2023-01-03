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
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorLift, motorFlip;

    private Servo ConeGrabber, OdLift;

    private boolean mecanumDriveMode = true, coastMotors = true;
    private float mecanumStrafe = 0, dominantXJoystick = 0;

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("leftFront");
        motorRight = hardwareMap.dcMotor.get("rightFront");
        motorLeft2 = hardwareMap.dcMotor.get("leftRear");
        motorRight2 = hardwareMap.dcMotor.get("rightRear");
        motorLift = hardwareMap.dcMotor.get("lift");
        motorFlip = hardwareMap.dcMotor.get("rightEncoder");
        ConeGrabber = hardwareMap.servo.get("ConeGrabber");
        OdLift = hardwareMap.servo.get("OdLift");

        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Drive Base TeleOp\nInit Opmode");
        ConeGrabber.setPosition(0.4);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coastMotors = false;
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

        telemetry.addData("LeftMTR  PWR: ", motorLeft.getPower());
        telemetry.addData("RightMTR PWR: ", motorRight.getPower());
        telemetry.addData("Servo", ConeGrabber.getPosition());
        if (abs(gamepad1.left_stick_x) > 0.15 || abs(gamepad1.right_stick_x) > 0.15) {
            dominantXJoystick = (abs(gamepad1.left_stick_x) - abs(gamepad1.right_stick_x));
            mecanumDriveMode = true;
        } else {
            mecanumDriveMode = false;
        }

        if (mecanumDriveMode) {
            if (dominantXJoystick > 0) {
                mecanumStrafe = gamepad1.left_stick_x;
            } else if (dominantXJoystick < 0) {
                mecanumStrafe = gamepad1.right_stick_x;
            }
            if (gamepad1.left_bumper) {
                motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe) / 3.0);
                motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe) / 3.0);
                motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe) / 3.0);
                motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe) / 3.0);
            } else if (gamepad1.right_bumper) {
                motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe) / 1.5);
                motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe) / 1.5);
                motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe) / 1.5);
                motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe) / 1.5);
            } else {
                motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe) / 2);
                motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe) / 2);
                motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe) / 2);
                motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe) / 2 );
            }
        } else if (!mecanumDriveMode) {
                drive(gamepad1.left_stick_y*0.8, gamepad1.right_stick_y*0.8);

        }

        if (gamepad2.right_stick_y >0.2 || gamepad2.right_stick_y < -0.2 ) {
            motorLift.setPower(gamepad2.right_stick_y);
        }else {
            motorLift.setPower(0);
        }
        if (gamepad2.a){
            motorFlip.setPower(1);
        }else if(gamepad2.b){
            motorFlip.setPower(-0.3);
        }else{
            motorFlip.setPower(0);
        }
        if (gamepad2.left_bumper) {
            ConeGrabber.setPosition(0);

        }else if (gamepad2.right_bumper){
            ConeGrabber.setPosition(0.4);
        }
        /*
        if(gamepad1.dpad_up){
            OdLift.setPosition(0);
        } else if (gamepad1.dpad_down){
            OdLift.setPosition(1);

        }
        */
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

