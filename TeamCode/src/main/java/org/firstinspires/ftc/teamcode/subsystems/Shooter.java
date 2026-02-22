package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.core.wrappers.ServoWrapper;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class Shooter {
    public static final double OVERHEAD = 0;
    public static final double DOWN = 1;
    public ServoWrapper hood;

    public Shooter(Hardware hardware)
    {
        hood = hardware.hood;
    }

    public void shootArtifacts()
    {

    }

    public void hoodDown() { hood.setPosition(DOWN); }
    public void hoodUp() { hood.setPosition(OVERHEAD); }

    public void update(AprilTagDetection detection, AprilTagWebcam webcam, MultipleTelemetry telemetry)
    {
        if (detection == null)
        {
            telemetry.addData("Goal ID Is Detected", false);
            telemetry.addLine();
        }
        else
        {
            telemetry.addData("Goal ID Is Detected", true);
            telemetry.addLine();

            webcam.updateRange(detection.ftcPose.range);
            webcam.updateBearing(detection.ftcPose.bearing);

            telemetry.addData("Range: ", webcam.getRange());
            telemetry.addLine();

        }
        double calculatedVel = Outtake.piecewise1CalculateFlywheelTangentialVelocityExperimental(webcam.getRange());
    }
}
