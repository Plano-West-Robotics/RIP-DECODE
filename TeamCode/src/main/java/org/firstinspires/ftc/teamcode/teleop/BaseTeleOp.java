/**
 * Subclasses of BaseTeleOp should override the setup() and run() to run code during init() and
 * loop(); init() and loop() should not be overridden, but init_loop() and start() can be.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public abstract class BaseTeleOp extends OpMode
{
    public Hardware hardware;
    public Gamepads gamepads;
    /**
     * Represents all the subsystems that should be updated in {@code loop()}. If you wish for a
     * subsystem to be automatically updated, add your subsystem to {@code subsystems} in the
     * {@code setup()} method body.
     */
    public List<Subsystem> subsystems;

    @Override
    public final void init()
    {
        hardware = new Hardware(hardwareMap);
        gamepads = new Gamepads(gamepad1, gamepad2);
        subsystems = new ArrayList<>();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        setup();
        telemetry.update();
    }

    public void setup() {}

    @Override
    public final void loop()
    {
        for (Subsystem subsystem : subsystems) subsystem.update(gamepads);
        gamepads.update(gamepad1, gamepad2);
        run();
        telemetry.update();
    }

    public void run() {}
}
