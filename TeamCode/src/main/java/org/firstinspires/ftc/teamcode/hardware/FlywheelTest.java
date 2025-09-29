package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FlywheelTest")
public class FlywheelTest extends OpMode {
    DcMotor flyL;
    DcMotor flyR;
    @Override
    public void init() {
        flyL = hardwareMap.get(DcMotor.class, "flyL");
        flyR = hardwareMap.get(DcMotor.class, "flyR");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            flyL.setPower(1);
            flyR.setPower(-1);
        }
        else if (gamepad1.b) {
            flyL.setPower(-1);
            flyR.setPower(1);
        }
        else {
            flyL.setPower(0);
            flyR.setPower(0);
        }
    }
}
