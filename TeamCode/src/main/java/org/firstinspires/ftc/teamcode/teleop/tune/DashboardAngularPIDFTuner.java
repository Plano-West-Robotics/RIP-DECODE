package org.firstinspires.ftc.teamcode.teleop.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

@Config
@TeleOp(group = "Tune")
public class DashboardAngularPIDFTuner extends OpMode
{
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    public static double targetAngle = 0;

    public Hardware hardware;
    public AbstractDrive drive;
    public PIDFController controller;

    @Override
    public void init()
    {
        hardware = new Hardware(hardwareMap);
        drive = new FieldCentricDrive(hardware);
        controller = new PIDFController(p, i, d, f);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ((FieldCentricDrive) drive).imu.resetYaw();
    }

    @Override
    public void loop()
    {
        controller.setPIDF(p, i, d, f);

        /*
        Although FTCDashboard receives targetAngle in degrees, both currentAngle and targetAngle get
        converted to radians. This ensures the PIDF constants from this tuner will work in our
        competition opmodes, as the processor in AprilTagWebcam reads angles in radians.
         */
        double currentAngle = ((FieldCentricDrive) drive).imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        targetAngle = Math.toRadians(Range.clip(targetAngle, -180, 180));

        double rx = -controller.calculate(currentAngle, targetAngle);
        drive.drive(0, 0, rx);

        telemetry.addData("Current Angle (degrees)", Math.toDegrees(currentAngle));
        telemetry.addData("Target Angle (degrees)", Math.toDegrees(targetAngle));
        telemetry.addData("RX", rx);

        telemetry.addData("FR Power", drive.drivetrainMotors.fr.getPower());
        telemetry.addData("FL Power", drive.drivetrainMotors.fl.getPower());
        telemetry.addData("BR Power", drive.drivetrainMotors.br.getPower());
        telemetry.addData("BL Power", drive.drivetrainMotors.bl.getPower());

        telemetry.update();
    }
}
