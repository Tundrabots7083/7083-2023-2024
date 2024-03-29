package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controls.Controller;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
    private Robot robot;
    private boolean started = false;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = new Robot(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.armController.start();
        telemetry.addLine("Robot Started");
    }

    @Override
    public void loop() {
        // TODO: see if this drops the container flip servo
        if (!started) {
            robot.armController.start();
            telemetry.addLine("Robot Started");
            started = true;
        }
        for (Controller controller : robot.controllers) {
            controller.execute(gamepad1, gamepad2);
        }
        telemetry.update();
    }
}
