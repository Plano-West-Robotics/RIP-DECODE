package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class FieldCentricDrive
{
    public DrivetrainMotors drivetrainMotors;
    public IMU imu;

    public FieldCentricDrive(Hardware hardware)
    {
        drivetrainMotors = hardware.drivetrainMotors;
        imu = hardware.imu;
    }

    public void update(Gamepads gamepads)
    {
        if (gamepads.isPressed(Button.GP1_DPAD_RIGHT))
        {
            resetHeading();
        }

        double drive = gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_Y);
        double strafe = gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_X);
        double turn = gamepads.getAnalogValue(Analog.GP1_RIGHT_STICK_X);
        drive(drive, strafe, turn);
    }

    public void drive(double y, double x, double rx)
    {
        double heading = getHeading(AngleUnit.RADIANS);

        double rotY = (x * Math.sin(-heading) + y * Math.cos(-heading)) * DrivetrainMotors.SPEED;
        double rotX = (x * Math.cos(-heading) - y * Math.sin(-heading)) * DrivetrainMotors.SPEED;
        rx *= DrivetrainMotors.TURN_SPEED;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frPower = (rotY - rotX - rx) / denominator;
        double flPower = (rotY + rotX + rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;

        drivetrainMotors.setPower(frPower, flPower, brPower, blPower);
    }

    public void resetHeading()
    {
        imu.resetYaw();
    }

    public double getHeading(AngleUnit angleUnit)
    {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }
}
