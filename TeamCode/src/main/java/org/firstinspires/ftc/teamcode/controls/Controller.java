package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Controller is the base interface for all controls on the robot.
 */
public interface Controller {

    /**
     * init initializes the controller.
     * @param hardwareMap the hardware map for the robot.
     */
    public void init(HardwareMap hardwareMap);

    public void execute(Gamepad gamepad, Telemetry telemetry);
}
