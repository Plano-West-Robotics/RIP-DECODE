package org.firstinspires.ftc.teamcode.teleop.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

@Config
@TeleOp(group = "Tune")
public class DashboardAngularPIDFTuner extends OpMode
{
    public static double P = 0.0675;
    public static double I = 0;
    public static double D = 0.0035;
    public static double F = 0;
    public static double targetAngle = 0; // degrees
    public static boolean resetImuYaw = false;

    public Hardware hardware;
    public AbstractDrive drive;
    public PIDFController controller;

    @Override
    public void init()
    {
        hardware = new Hardware(hardwareMap);
        drive = new FieldCentricDrive(hardware);
        controller = new PIDFController(P, I, D, F);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ((FieldCentricDrive) drive).imu.resetYaw();
    }

    @Override
    public void loop()
    {
        // You probably want to set targetAngle=0 before setting resetImuYaw=true
        if (resetImuYaw)
        {
            ((FieldCentricDrive) drive).imu.resetYaw();
        }

        controller.setPIDF(P, I, D, F);

        double currentAngle = ((FieldCentricDrive) drive).imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        targetAngle = Range.clip(targetAngle, -180, 180);

        double rx = -controller.calculate(currentAngle, targetAngle);
        drive.drive(0, 0, rx);

        telemetry.addData("Current Angle (degrees)", currentAngle);
        telemetry.addData("Target Angle (degrees)", targetAngle);
        telemetry.addData("RX", rx);

        telemetry.addData("FR Power", drive.drivetrainMotors.fr.getPower());
        telemetry.addData("FL Power", drive.drivetrainMotors.fl.getPower());
        telemetry.addData("BR Power", drive.drivetrainMotors.br.getPower());
        telemetry.addData("BL Power", drive.drivetrainMotors.bl.getPower());

        telemetry.update();
    }
}
