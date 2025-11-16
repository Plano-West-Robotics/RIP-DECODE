package org.firstinspires.ftc.teamcode.core.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {
    private DcMotorEx motor;
    public Encoder(DcMotorEx motor)
    {
        this.motor = motor;
    }

    public int getPosition() { return motor.getCurrentPosition(); }
    public void setTarget(int pos) { motor.setTargetPosition(pos); }
}
