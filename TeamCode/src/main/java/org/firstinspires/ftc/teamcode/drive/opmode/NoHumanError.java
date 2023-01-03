package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "NO ERROR TeleOp 4")

public class NoHumanError extends OpMode {
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2;

    // todo: write your code here
    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("leftFront");
        motorRight = hardwareMap.dcMotor.get("rightFront");
        motorLeft2 = hardwareMap.dcMotor.get("leftRear");
        motorRight2 = hardwareMap.dcMotor.get("rightRear");
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        drive(gamepad1.right_stick_y, gamepad1.right_stick_y);
    }


    public void drive(double left, double right) {
        motorLeft.setPower(left);
        motorLeft2.setPower(left);
        motorRight.setPower(right);
        motorRight2.setPower(right);

    }
}