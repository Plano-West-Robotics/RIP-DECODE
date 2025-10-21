package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.templates.StageServoMono;

public class Hardware
{
    public Revolver revolver;

    public Hardware(HardwareMap hardwareMap)
    {
        revolver = new Revolver(hardwareMap);
    }
}
