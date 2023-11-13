package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class AprilTagDetectorSingleton {
    // Private static variable to hold the single instance of the class
    private static AprilTagDetectorSingleton instance;

    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTagProcessor;
    private LinearOpMode opMode=null;

    // Private constructor to prevent instantiation from other classes
    private AprilTagDetectorSingleton(LinearOpMode opMode) {
        this.opMode = opMode;

        this.initAprilTag();
        this.setManualExposure(6, 250);
    }


    // Public static method to get the single instance of the class
    public static AprilTagDetectorSingleton getInstance(LinearOpMode opMode) {
        if (instance == null) {
            instance = new AprilTagDetectorSingleton(opMode);
        }
        return instance;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return this.aprilTagProcessor;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        //opMode.telemetry.addData("initAprilTag", "initAprilTag started");
        // opMode.telemetry.update();
        // Create the vision portal by using a builder.

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

    }

    /*
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initAprilTag(), and only works for
     * Webcams;
     */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting");
            opMode.telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() !=
                    VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            opMode.telemetry.addData("Camera", "Ready");
            opMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested()) {
            ExposureControl exposureControl =
                    visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            opMode.sleep(20);
        }
    }
}

