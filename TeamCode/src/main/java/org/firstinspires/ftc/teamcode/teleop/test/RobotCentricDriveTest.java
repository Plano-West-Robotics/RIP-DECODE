package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@TeleOp(group = "Test")
public class RobotCentricDriveTest extends BaseTeleOp
{
    public AbstractDrive drive;

    @Override
    public void setup()
    {
        drive = new RobotCentricDrive(hardware);
    }

    @Override
    public void run()
    {
        drive.update(gamepads);
    }
}
