package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.localization.CustomIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pinpointFiles.GoBildaPinpointDriver;

public class PinpointIMU implements CustomIMU {
    GoBildaPinpointDriver pinpoint;
    @Override
    public void initialize(HardwareMap hardwareMap, String hardwareMapName, RevHubOrientationOnRobot hubOrientation) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,hardwareMapName);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    @Override
    public void resetYaw() {
        pinpoint.resetPosAndIMU();
    }
}
