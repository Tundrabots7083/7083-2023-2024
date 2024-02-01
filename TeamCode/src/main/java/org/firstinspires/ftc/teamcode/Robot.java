package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controls.LiftController;
import org.firstinspires.ftc.teamcode.controls.Controller;
import org.firstinspires.ftc.teamcode.controls.DroneLauncherController;
import org.firstinspires.ftc.teamcode.controls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controls.PixelContainerController;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

import java.util.Arrays;
import java.util.List;

public class Robot {
    public static Robot robot;
    public MecanumDriveController mecanumDriveController;
    public LiftController liftController;
    public PixelContainerController pixelContainerController;
    public DroneLauncherController droneLauncherController;
    private final Telemetry telemetry;
    public final List<Controller> controllers;

    public VisionSensor visionSensor;
    public RobotState state;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        robot = this;
        this.telemetry = telemetry;

        mecanumDriveController = new MecanumDriveController(hardwareMap, telemetry);
        liftController = new LiftController(hardwareMap, telemetry);
//        droneLauncherController = new DroneLauncherController(hardwareMap, telemetry);
        pixelContainerController = new PixelContainerController(hardwareMap, telemetry);
        controllers = Arrays.asList(mecanumDriveController, liftController, pixelContainerController);

        telemetry.addLine("[Robot] controllers initialized");
    }

    public static Robot getRobot() {
        return robot;
    }
}
