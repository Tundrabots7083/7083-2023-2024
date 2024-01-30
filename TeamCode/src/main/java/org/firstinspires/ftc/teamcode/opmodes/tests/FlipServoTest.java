package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controls.ArmController;

@Config
@TeleOp (name="Flip Servo Test", group="test")
public class FlipServoTest extends OpMode {
    public static double FLIP_SERVO_POS = 0.625;
    private Servo flipServo;
    private ArmController armController;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armController = new ArmController(hardwareMap, telemetry);
        flipServo = hardwareMap.get(Servo.class, "containerFlip");
        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        flipServo.setPosition(FLIP_SERVO_POS);
        telemetry.addData("[Flip Servo] Pos", FLIP_SERVO_POS);
        telemetry.update();
    }
}
