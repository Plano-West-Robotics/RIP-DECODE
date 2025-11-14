package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(group = "Test")
public class AprilTagTest extends BaseTeleOp
{
    public AprilTagWebcam webcam;
    
    @Override
    public void setup()
    {
        webcam = new AprilTagWebcam(hardware);

        subsystems.add(webcam);
    }

    @Override
    public void init_loop()
    {
        boolean webcamIsReady = webcam.portal.getCameraState() != VisionPortal.CameraState.STREAMING;
        telemetry.addData("Camera", webcamIsReady ? "Ready" : "Not Ready");
        telemetry.update();
    }

    @Override
    public void run()
    {
        List<AprilTagDetection> detections = webcam.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());

        int count = 0;
        for (AprilTagDetection detection : detections)
        {
            telemetry.addData(String.format("%d ID", count), "%d", detection.metadata.id);
            telemetry.addData(String.format("%d Range", count), "%5.5f", detection.ftcPose.range);
            telemetry.addData(String.format("%d Bearing", count), "%5.5f", detection.ftcPose.bearing);
            telemetry.addData(String.format("%d Elevation", count), "%5.5f", detection.ftcPose.elevation);
            count++;
        }
    }
}
