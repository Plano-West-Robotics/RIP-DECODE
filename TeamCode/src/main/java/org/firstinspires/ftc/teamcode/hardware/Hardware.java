package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.base.CRServoWrapper;
import org.firstinspires.ftc.teamcode.hardware.base.MotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.base.ServoWrapper;
import org.firstinspires.ftc.teamcode.hardware.templates.StageServoMono;

public class Hardware
{
    public IMU imu;
    public IMU.Parameters imuParameters;

    public Drivetrain drivetrain;
    public MotorWrapper intakeMotor, outtakeMotor;

    public Hardware(HardwareMap hardwareMap)
    {
//        imu = hardwareMap.get(IMU.class, "imu");
//        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//            RevHubOrientationOnRobot.UsbFacingDirection.UP
//        ));
//        imu.initialize(imuParameters);

//        drivetrain = new Drivetrain(hardwareMap);
        intakeMotor = new MotorWrapper(hardwareMap, "i");
        intakeMotor.reverse();
        outtakeMotor = new MotorWrapper(hardwareMap, "o");
    }
}
