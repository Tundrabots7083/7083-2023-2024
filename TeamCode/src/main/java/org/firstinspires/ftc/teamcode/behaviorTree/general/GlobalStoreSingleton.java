package org.firstinspires.ftc.teamcode.behaviorTree.general;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageVisionDetectorSingleton;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class GlobalStoreSingleton {
    private static GlobalStoreSingleton instance;
    private final Map<String, Object> data = new HashMap<String, Object>();

    private LinearOpMode opMode=null;
    // Private constructor to prevent instantiation from other classes
    private GlobalStoreSingleton(LinearOpMode opMode) {
        this.opMode = opMode;

    }

    // Public static method to get the single instance of the class
    public static GlobalStoreSingleton getInstance(LinearOpMode opMode) {
        if (instance == null) {
            instance = new GlobalStoreSingleton(opMode);
        }
        return instance;
    }

    public static  void reset(){
        instance = null;
    }

    public void setValue(String key, Object value) {
        data.put(key, value);
    }
    public Object removeValue(String key) {
        return data.remove(key);
    }

    public Object getValue(String key) {
        return data.get(key);
    }

}
