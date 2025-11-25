package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagTelemetry
{
    public static void init_loop_telemetry(Telemetry t, AprilTagWebcam w)
    {
        t.addData("Goal ID", w.getGoalId());

        boolean webcamIsReady = w.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
        t.addData("Camera", webcamIsReady ? "Ready" : "Not Ready");

        t.update();
    }

    public static void loop_telemetry(Telemetry t, AprilTagDetection detection)
    {
        t.addData("ID", "%d", detection.metadata.id);
        t.addData("Range", "%5.5f", detection.ftcPose.range);
        t.addData("Bearing", "%5.5f", detection.ftcPose.bearing);
        t.addData("Elevation", "%5.5f", detection.ftcPose.elevation);
    }

}
