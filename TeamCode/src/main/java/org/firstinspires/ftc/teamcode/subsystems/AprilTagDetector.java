package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;
public class AprilTagDetector {

        private  VisionPortal visionPortal; // Used to manage the video source.
        private  AprilTagProcessor aprilTagProcessor;
        private LinearOpMode opMode=null;
        public AprilTagDetector(LinearOpMode opMode) {
            this.opMode = opMode;

            this.initialize();
            this.setManualExposure(6, 250);
        }

        public AprilTagProcessor getAprilTagProcessor() {
            return this.aprilTagProcessor;
        }

        /**
         * Initialize the AprilTag processor.
         */
        private void initialize() {
            // Create the AprilTag processor by using a builder.
            if(aprilTagProcessor == null) {
                aprilTagProcessor = new AprilTagProcessor.Builder().build();
            }

            opMode.telemetry.addData("AprilTagDetector", "initialize");
            opMode.telemetry.update();
            // Create the vision portal by using a builder.
          //  if(visionPortal == null) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTagProcessor)
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


