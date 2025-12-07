package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class RobotCentricDrive
{
    public DrivetrainMotors drivetrainMotors;

    public RobotCentricDrive(Hardware hardware)
    {
        drivetrainMotors = hardware.drivetrainMotors;
    }

    public void update(Gamepads gamepads)
    {
        double drive = gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_Y);
        double strafe = gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_X);
        double turn = gamepads.getAnalogValue(Analog.GP1_RIGHT_STICK_X);
        drive(drive, strafe, turn);
    }

    public void drive(double y, double x, double rx)
    {
        double frPower = y - x;
        double flPower = y + x;
        double brPower = y + x;
        double blPower = y - x;

        frPower -= rx * DrivetrainMotors.TURN_SPEED;
        brPower -= rx * DrivetrainMotors.TURN_SPEED;
        flPower += rx * DrivetrainMotors.TURN_SPEED;
        blPower += rx * DrivetrainMotors.TURN_SPEED;

        frPower *= DrivetrainMotors.SPEED;
        flPower *= DrivetrainMotors.SPEED;
        brPower *= DrivetrainMotors.SPEED;
        blPower *= DrivetrainMotors.SPEED;

        drivetrainMotors.setPower(frPower, flPower, brPower, blPower);
    }
}
