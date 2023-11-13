package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class SpikeElementDetector {

    //This is a placeholder class. to be implemented to detect the spike element
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTagProcessor; //TODO change to tensor flow detector
    private LinearOpMode opMode=null;
    public SpikeElementDetector(LinearOpMode opMode) {
        opMode.telemetry.addData("AprilTagDetector", "AprilTagDetector started");
        opMode.telemetry.update();
        this.opMode = opMode;

        this.initAprilTag();
        this.setManualExposure(6, 250);
    }

    public AprilTagProcessor getSpikeElementProcessor() {
        return this.aprilTagProcessor;
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        opMode.telemetry.addData("initAprilTag", "initAprilTag started");
        opMode.telemetry.update();
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
                // sleep(20);
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
                // sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            // sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            // sleep(20);
        }
    }
}
