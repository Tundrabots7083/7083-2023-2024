package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Obtains the distance from the sensor to the object in front of the sensor.
 */
public class DistanceSensor {
    private final Telemetry telemetry;
    private com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;

    /**
     * Creates a DistanceSensor object from the included hardware map. The distance sensor must be
     * named `distanceSensor`.
     * @param hardwareMap the hardware map for the robot.
     * @param telemetry the telemetry to be used for any output to the driver station.
     */
    public DistanceSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "distanceSensor");
    }

    /**
     * Gets the distance to the nearest object in inches.
     * @return the distance to the nearest object in inches.
     */
    public double getDistance() {
        return getDistance(DistanceUnit.INCH);
    }

    /**
     * Gets the distance to the nearest object in the requested distance unit.
     * @param du the distance unit to use in calculating the distance.
     * @return the distance to the nearest object in the requested distance unit.
     */
    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }
}