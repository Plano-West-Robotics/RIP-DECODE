package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.templates.StageServoMono;

public class Hardware
{
    public Drivetrain drivetrain;

    public Hardware(HardwareMap hardwareMap)
    {
        drivetrain = new Drivetrain(hardwareMap);
    }
}
