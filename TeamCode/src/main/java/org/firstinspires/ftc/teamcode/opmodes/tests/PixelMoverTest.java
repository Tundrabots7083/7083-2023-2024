package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.controls.PixelMoverController;

@TeleOp (name="Pixel Mover TeleOp", group="test")
public class PixelMoverTest extends OpMode {
    private PixelMoverController pixelMoverController;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pixelMoverController = new PixelMoverController(hardwareMap, telemetry);
        telemetry.addLine("Initialization Complete");
    }

    @Override
    public void loop() {
        pixelMoverController.execute(gamepad1, gamepad2);
    }
}
