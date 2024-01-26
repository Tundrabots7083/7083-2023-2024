package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorDistance {
    private final Telemetry telemetry;
    private DistanceSensor distanceSensor;

    SensorDistance(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public double getDistance() {
        return getDistance(DistanceUnit.INCH);
    }

    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }
}
