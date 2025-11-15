package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.RobotCentricDrive;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends BaseTeleOp
{
    public AbstractDrive drive;

    @Override
    public void setup()
    {
        subsystems.add(drive);
        drive = new FieldCentricDrive(hardware);
    }

    @Override
    public void run()
    {



    }
}
