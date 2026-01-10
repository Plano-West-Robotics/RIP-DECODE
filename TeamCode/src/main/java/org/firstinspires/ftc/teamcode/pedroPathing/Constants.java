package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().
            mass(9.706876718)
//            .forwardZeroPowerAcceleration(-62.7355128)
            .lateralZeroPowerAcceleration(-57.93669)
            .forwardZeroPowerAcceleration(-34.5)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.0048, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.007, 0, 0.0003, 0.6, 0.01))
//            .centripetalScaling(0.022);
            .centripetalScaling(0.007);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.6)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(50.674863877)
//            .yVelocity(43.081820267);
            .xVelocity(62)
            .yVelocity(49);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("br")
            .strafeEncoder_HardwareMapName("bl")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            )
            .strafePodX(3.3125)
            .forwardPodY(4)
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
//            .forwardTicksToInches(0.00195616)
//            .strafeTicksToInches(0.00305353222)
            .forwardTicksToInches(0.0029938)
            .strafeTicksToInches(0.002);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
