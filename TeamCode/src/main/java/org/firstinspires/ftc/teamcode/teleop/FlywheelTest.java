package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FlywheelTest")
public class FlywheelTest extends BaseTeleOp
{
    public DcMotor leftFlywheel;
    public DcMotor rightFlywheel;

    @Override
    public void setup()
    {
        leftFlywheel = hardwareMap.get(DcMotor.class, "flyL");
        rightFlywheel = hardwareMap.get(DcMotor.class, "flyR");
    }

    @Override
    public void run()
    {
        if (gamepad1.a)
        {
            leftFlywheel.setPower(1);
            rightFlywheel.setPower(-1);
        }
        else if (gamepad1.b)
        {
            leftFlywheel.setPower(-1);
            rightFlywheel.setPower(1);
        }
        else
        {
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
        }
    }
}
