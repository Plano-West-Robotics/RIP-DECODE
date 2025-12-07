package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.wrappers.MotorWrapper;

public class DrivetrainMotors
{
    public static final double SPEED = 0.85;
    public static final double TURN_SPEED = 0.7;

    public MotorWrapper fr, fl, br, bl;
    public MotorWrapper[] motors;

    public DrivetrainMotors(HardwareMap hardwareMap)
    {
        fr = new MotorWrapper(hardwareMap, "fr", true);
        fl = new MotorWrapper(hardwareMap, "fl", true);
        br = new MotorWrapper(hardwareMap, "br", true);
        bl = new MotorWrapper(hardwareMap, "bl", true);

        fl.reverse();

        motors = new MotorWrapper[] {fr, fl, br, bl};

        for (MotorWrapper motor : motors) motor.noEncoder();
    }

    public void setPower(double frPower, double flPower, double brPower, double blPower)
    {
        fr.setPower(frPower);
        fl.setPower(flPower);
        br.setPower(brPower);
        bl.setPower(blPower);
    }
}
