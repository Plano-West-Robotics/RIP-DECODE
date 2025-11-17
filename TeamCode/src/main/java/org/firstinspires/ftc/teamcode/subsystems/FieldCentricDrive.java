package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class FieldCentricDrive extends AbstractDrive implements Subsystem
{
    public IMU imu;

    public FieldCentricDrive(Hardware hardware)
    {
        super(hardware);
        imu = hardware.imu;
    }

    @Override
    public void update(Gamepads gamepads)
    {
        if (gamepads.isPressed(Button.GP1_DPAD_RIGHT)) imu.resetYaw();
        super.update(gamepads);
    }

    @Override
    public void drive(double y, double x, double rx)
    {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotY = (x * Math.sin(-botHeading) + y * Math.cos(-botHeading)) * speed;
        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * speed;
        rx *= turnSpeed;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frPower = (rotY - rotX - rx) / denominator;
        double flPower = (rotY + rotX + rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;

        drivetrainMotors.setPower(frPower, flPower, brPower, blPower);
    }
}
