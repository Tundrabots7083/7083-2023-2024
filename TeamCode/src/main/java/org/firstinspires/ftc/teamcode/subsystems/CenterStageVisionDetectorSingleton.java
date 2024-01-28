package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class CenterStageVisionDetectorSingleton {
    private static CenterStageVisionDetectorSingleton instance;
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTagProcessor;
    private FirstVisionProcessor visionProcessor;
    private LinearOpMode opMode=null;
    // Private constructor to prevent instantiation from other classes
    private CenterStageVisionDetectorSingleton(LinearOpMode opMode) {
        this.opMode = opMode;

        this.initialize();
        this.setManualExposure(6, 250);
    }

    // Public static method to get the single instance of the class
    public static CenterStageVisionDetectorSingleton getInstance(LinearOpMode opMode) {
        if (instance == null) {
            instance = new CenterStageVisionDetectorSingleton(opMode);
        }
        return instance;
    }

    public static  void reset(){
        instance = null;
    }
    public AprilTagProcessor getAprilTagProcessor() {
        return this.aprilTagProcessor;
    }
    public FirstVisionProcessor getVisionProcessor() {
        return this.visionProcessor;
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initialize() {
        // Create the AprilTag processor by using a builder.
      //  if(aprilTagProcessor == null) {
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
       // }

        // Create the AprilTag processor by using a builder.
       // if(visionProcessor == null) {
            visionProcessor = new FirstVisionProcessor();
       // }

        opMode.telemetry.addData("CenterStageVisionDetector", "initialize");
        opMode.telemetry.update();

        // Create the vision portal by using a builder.
        //  if(visionPortal == null) {

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                //.addProcessor(aprilTagProcessor)
                .addProcessors(visionProcessor,aprilTagProcessor)
                .build();


        // }
    }

    /*
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initialize(), and only works for
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
