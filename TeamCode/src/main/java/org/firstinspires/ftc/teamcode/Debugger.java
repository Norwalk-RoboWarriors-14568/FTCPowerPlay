package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;

public class Debugger extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        for (Map.Entry<String, DcMotor> motorEntry : hardwareMap.dcMotor.entrySet()) {
            DcMotor motor = motorEntry.getValue();
            telemetry.addData("Motor Name", motorEntry.getKey())
                    .addData("Encoder Position", motor.getCurrentPosition())
                    .addData("Motor Controller", motor.getController().getDeviceName())
                    .addData("Port Number", motor.getPortNumber());
        }
    }
}
