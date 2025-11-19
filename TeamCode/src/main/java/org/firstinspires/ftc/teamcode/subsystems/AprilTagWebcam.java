package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

import javax.annotation.Nullable;

public class AprilTagWebcam
{
    public static final class LensIntrinsics
    {
        public static final double FX = 6.501905436046564546e2;
        public static final double FY = 6.527525718349902490e2;
        public static final double CX = 3.208106928325781837e2;
        public static final double CY = 2.438054456685325704e2;
    }

    /**
     * Adjust Image Decimation to trade-off detection-range for detection-rate.
     * <p>
     * e.g. Some typical detection data using a Logitech C920 WebCam
     * <p>
     * Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
     * <p>
     * Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
     * <p>
     * Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
     * <p>
     * Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
     * <p>
     * Note: Decimation can be changed on-the-fly to adapt during a match.
     */
    public static final int DECIMATION = 3;

    public static final int RESOLUTION_WIDTH = 640;
    public static final int RESOLUTION_HEIGHT = 480;
    public static final int EXPOSURE = 6; // milliseconds
    public static final int GAIN = 250;

    public static final int BLUE_GOAL_ID = 20;
    public static final int RED_GOAL_ID = 24;

    public WebcamName camera;
    public AprilTagProcessor processor;
    public VisionPortal portal;
    public int goalId;

    public AprilTagWebcam(Hardware hardware, int goalId)
    {
        camera = hardware.webcam;

        processor = new AprilTagProcessor.Builder()
            .setLensIntrinsics(
                LensIntrinsics.FX,
                LensIntrinsics.FY,
                LensIntrinsics.CX,
                LensIntrinsics.CY
            )
            .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setDrawTagID(true)
            .build();
        processor.setDecimation(DECIMATION);

        // Create the vision portal by using a builder.
        portal = new VisionPortal.Builder()
            .setCamera(camera)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
            .addProcessor(processor)
            .setAutoStartStreamOnBuild(true)
            .setShowStatsOverlay(true)
            .build();

        this.goalId = goalId;
    }

    public List<AprilTagDetection> getDetections()
    {
        return processor.getDetections();
    }

    @Nullable
    public AprilTagDetection getDetectionById(int id)
    {
        for (AprilTagDetection detection : processor.getDetections())
        {
            if (detection.id == id)
            {
                return detection;
            }
        }

        return null;
    }

    @Nullable
    public AprilTagDetection getGoalDetection()
    {
        return getDetectionById(goalId);
    }

    public void stopStreaming()
    {
        portal.stopStreaming();
    }

    public void resumeStreaming()
    {
        portal.resumeStreaming();
    }

    public int getGoalId()
    {
        return goalId;
    }

    public void toggleGoalId()
    {
        goalId = goalId == BLUE_GOAL_ID ? RED_GOAL_ID : BLUE_GOAL_ID;
    }
}
