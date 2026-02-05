package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pinpointFiles.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.core.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.core.wrappers.ServoPairWrapper;

public class Hardware
{
    public IMU imu;
    public IMU.Parameters imuParameters;
    public WebcamName webcam;
    public DrivetrainMotors drivetrainMotors;
    public MotorWrapper intakeMotor, outtakeMotor;
    public VoltageSensor vs;
    public ServoPairWrapper stoppers;
    public RightStopper rightStopper;
    public GoBildaPinpointDriver pinpoint;

    public Hardware(HardwareMap hardwareMap)
    {
        imu = hardwareMap.get(IMU.class, "imu");
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        imu.initialize(imuParameters);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        vs = hardwareMap.voltageSensor.iterator().next();

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webcam = null;

        drivetrainMotors = new DrivetrainMotors(hardwareMap);

        intakeMotor = new MotorWrapper(hardwareMap, "i", false);

        outtakeMotor = new MotorWrapper(hardwareMap, "o", false);

//        stoppers = new ServoPairWrapper(hardwareMap, "stopL", "stopR", /*TODO: arbitrary*/ 0.5);

        rightStopper = new RightStopper(hardwareMap);
    }
}
