package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.RobotCentricDrive;

@TeleOp(group = "Test")
public class RobotCentricTest extends BaseTeleOp
{
    public AbstractDrive drive;

    @Override
    public void setup()
    {
        drive = new RobotCentricDrive(hardware);

        subsystems.add(drive);
    }
}
