package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.templates.Drive;
import org.firstinspires.ftc.teamcode.subsystems.RobotCentricDrive;

@TeleOp
public class DrivetrainTeleOp extends BaseTeleOp
{
    public Drive drive;

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
