package org.firstinspires.ftc.teamcode.core.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class EncoderWrapper
{
    public DcMotorEx motor;

    public EncoderWrapper(DcMotorEx motor)
    {
        this.motor = motor;
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }

    public void setTarget(int pos)
    {
        motor.setTargetPosition(pos);
    }
}
