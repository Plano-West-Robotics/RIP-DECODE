package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.control.Button;
import org.firstinspires.ftc.teamcode.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class RobotCentricDrive extends AbstractDrive
{
    public RobotCentricDrive(Hardware hardware)
    {
        super(hardware);
    }

    @Override
    public void update(Gamepads gamepads)
    {
        if (gamepads.justPressed(Button.GP1_RIGHT_BUMPER)) toggleSlowMode();
        super.update(gamepads);
    }

    @Override
    protected void drive(double y, double x, double rx)
    {
        double frPower = y - x;
        double flPower = y + x;
        double brPower = y + x;
        double blPower = y - x;

        frPower -= rx * turnSpeed;
        brPower -= rx * turnSpeed;
        flPower += rx * turnSpeed;
        blPower += rx * turnSpeed;

        frPower *= speed;
        flPower *= speed;
        brPower *= speed;
        blPower *= speed;

        drivetrainMotors.setPower(frPower, flPower, brPower, blPower);
    }
}
