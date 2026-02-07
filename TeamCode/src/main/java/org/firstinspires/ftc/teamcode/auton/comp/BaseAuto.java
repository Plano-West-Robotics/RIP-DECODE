package org.firstinspires.ftc.teamcode.auton.comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public abstract class BaseAuto extends OpMode
{
    public Hardware hardware;
    public Gamepads gamepads;
    public Timer pathTimer;
    public Intake intake;
    public Outtake outtake;
    public AprilTagWebcam webcam;
    public Follower follower;
    public DcMotorEx flywheel;

    @Override
    public final void init()
    {
        hardware = new Hardware(hardwareMap);
        gamepads = new Gamepads(gamepad1, gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
        flywheel = (DcMotorEx) outtake.motor.motor;
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        follower = Constants.createFollower(hardwareMap);
        setup();
        telemetry.update();
    }

    public abstract void setup();

    @Override
    public final void init_loop()
    {
        gamepads.update(gamepad1, gamepad2);
        setup_loop();
        telemetry.update();
    }

    public void setup_loop() {}

    @Override
    public final void start()
    {
        go();
    }

    public void go() {}

    @Override
    public final void loop()
    {
        follower.update();
        gamepads.update(gamepad1, gamepad2);
        while_running();
        telemetry.update();
    }

    public abstract void while_running();
}
