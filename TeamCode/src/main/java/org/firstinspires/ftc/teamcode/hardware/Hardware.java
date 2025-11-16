package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.wrappers.CRServoWrapper;
import org.firstinspires.ftc.teamcode.core.wrappers.Encoder;
import org.firstinspires.ftc.teamcode.core.wrappers.MotorWrapper;

public class Hardware
{
    public IMU imu;
    public IMU.Parameters imuParameters;
    public WebcamName webcam;
    public DrivetrainMotors drivetrainMotors;
    public MotorWrapper intakeMotor, outtakeMotor;
    public PaddlesServoPair paddles;
    public CRServoWrapper transfer;
//    public Encoder rightOdo, frontOdo;

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

        intakeMotor = new MotorWrapper(hardwareMap, "i", false);
        intakeMotor.reverse();

        outtakeMotor = new MotorWrapper(hardwareMap, "o", false);

        transfer = new CRServoWrapper(hardwareMap, "t");
        transfer.reverse();

        paddles = new PaddlesServoPair(hardwareMap);

//        frontOdo = new Encoder((DcMotorEx) drivetrainMotors.br);
//        rightOdo = new Encoder((DcMotorEx) drivetrainMotors.fl);
    }
}
