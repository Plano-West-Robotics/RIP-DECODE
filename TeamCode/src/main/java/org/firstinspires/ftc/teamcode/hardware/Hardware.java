package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.base.MotorWrapper;

public class Hardware
{
    public IMU imu;
    public IMU.Parameters imuParameters;

    public WebcamName webcam;

    public DrivetrainMotors drivetrainMotors;
    public MotorWrapper intakeMotor, outtakeMotor;

    public Hardware(HardwareMap hardwareMap)
    {
        imu = hardwareMap.get(IMU.class, "imu");
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(imuParameters);

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        drivetrainMotors = new DrivetrainMotors(hardwareMap);

        intakeMotor = new MotorWrapper(hardwareMap, "i");
        intakeMotor.reverse();

        outtakeMotor = new MotorWrapper(hardwareMap, "o");
    }
}
