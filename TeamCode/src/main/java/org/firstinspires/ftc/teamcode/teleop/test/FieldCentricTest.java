package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@TeleOp(group = "Test")
public class FieldCentricTest extends BaseTeleOp
{
    public AbstractDrive drive;

    @Override
    public void setup()
    {
        drive = new FieldCentricDrive(hardware);

        subsystems.add(drive);
    }
}
