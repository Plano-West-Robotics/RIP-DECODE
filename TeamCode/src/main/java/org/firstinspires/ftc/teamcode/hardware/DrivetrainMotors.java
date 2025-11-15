package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.base.MotorWrapper;

public class DrivetrainMotors
{
    public MotorWrapper fr, fl, br, bl;
    public MotorWrapper[] motors;

    public DrivetrainMotors(HardwareMap hardwareMap)
    {
        fr = new MotorWrapper(hardwareMap, "fr", true);
        fl = new MotorWrapper(hardwareMap, "fl", true);
        br = new MotorWrapper(hardwareMap, "br", true);
        bl = new MotorWrapper(hardwareMap, "bl", true);

        fl.reverse();
        bl.reverse();

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
