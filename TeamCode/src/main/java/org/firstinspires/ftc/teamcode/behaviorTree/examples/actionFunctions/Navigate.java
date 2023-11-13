package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.DriveTrainConfig;
import org.firstinspires.ftc.teamcode.models.ErrorTolerances;
import org.firstinspires.ftc.teamcode.models.PIDCoeficients;
import org.firstinspires.ftc.teamcode.models.RelativePosition;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
@Config
public class Navigate implements ActionFunction {
        private DriveTrain driveTrain;

        private AprilTagDetection referenceTag = null;
        private double drive = 0; // Desired forward power/speed (-1 to +1)
        private double strafe = 0; // Desired strafe power/speed (-1 to +1)
        private double turn = 0; // Desired turning power/speed (-1 to +1)

        private DriveTrainConfig driveTrainConfig = null;
        private int referenceTagId = -1;
        private double headingErrorTolerance = 1;
        private double rangeErrorTolerance = 1;
        private double yawErrorTolerance = 1;
        private LinearOpMode opMode;
        private RelativePosition currentTarget;
        private List<AprilTagDetection> currentDetections;

        private PIDController rangeController;
        private PIDController headingController;
        private PIDController yawController;
        private PIDCoeficients pidCoeficients;

        public static double RKp=0.033;
        public static double RKi=0.185;
        public static double RKd=0.005;//0.0135 use if tune

        public static double HKp=0.028;
        public static double HKi=0.024;
        public static double HKd=0.0001;

        public static double YKp=0.025;
        public static double YKi=0.09;
        public static double YKd =0.0001;


        public Navigate(LinearOpMode opMode) {
                this.opMode = opMode;
                this.driveTrain = new DriveTrain(opMode);
        }

        public Status perform(GlobalStore globalStore) {
                this.driveTrainConfig = (DriveTrainConfig) globalStore.getValue("DriveTrainConfig");

                this.currentTarget = (RelativePosition) globalStore.getValue("CurrentTarget");
                this.referenceTagId = currentTarget.referenceTagId;


                ErrorTolerances errorTolerances =(ErrorTolerances) globalStore.getValue("ErrorTolerances");
                this.headingErrorTolerance=errorTolerances.headingErrorTolerance;
                this.rangeErrorTolerance = errorTolerances.rangeErrorTolerance;
                this.yawErrorTolerance = errorTolerances.yawErrorTolerance;


                opMode.telemetry.addData("Navigation", "perform;******* currentTarget.referenceTagId = %d", currentTarget.referenceTagId);
                opMode.telemetry.update();

                getCurrentDetections(globalStore);
                setPIDControllers(globalStore);

               return navigateToRelativeLocation();

        }

        private void getCurrentDetections(GlobalStore globalStore) {
                this.currentDetections = (List<AprilTagDetection>) globalStore.getValue("CurrentDetections");

                this.referenceTag = currentDetections.stream().filter(detection -> detection.metadata != null && detection.id == this.referenceTagId).findFirst().orElse(null);

                if (referenceTag != null) {
                        opMode.telemetry.addData("Navigate", "Tag ID %d metadata.id %d name is %s\n", referenceTag.id, referenceTag.metadata.id, referenceTag.metadata.name);
                } else {
                        opMode.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", -1);
                }

                opMode.telemetry.update();
        }

        private void setPIDControllers(GlobalStore globalStore){
                this.pidCoeficients = (PIDCoeficients) globalStore.getValue("PIDCoeficients");
/*
                this.rangeController = new PIDController(this.pidCoeficients.RKp, this.pidCoeficients.RKi, this.pidCoeficients.RKd,this.opMode, "R");
                this.headingController = new PIDController(this.pidCoeficients.HKp,this.pidCoeficients.HKi,this.pidCoeficients.HKd,this.opMode, "H");
                this.yawController = new PIDController(this.pidCoeficients.YKp,this.pidCoeficients.YKi,this.pidCoeficients.YKd,this.opMode, "Y");
                */
               /* if(this.rangeController == null) {
                        this.rangeController = new PIDController(this.pidCoeficients.RKp, this.pidCoeficients.RKi, this.pidCoeficients.RKd, this.opMode, "R");
                }
                if(this.headingController == null) {
                        this.headingController = new PIDController(this.pidCoeficients.HKp, this.pidCoeficients.HKi, this.pidCoeficients.HKd, this.opMode, "H");
                }

                if(this.yawController == null) {
                        this.yawController = new PIDController(this.pidCoeficients.YKp, this.pidCoeficients.YKi, this.pidCoeficients.YKd, this.opMode, "Y");
                }
                */

                if(this.rangeController == null) {
                        this.rangeController = new PIDController(this.RKp, this.RKi, this.RKd, this.opMode, "R");
                }
                if(this.headingController == null) {
                        this.headingController = new PIDController(this.HKp, this.HKi, this.HKd, this.opMode, "H");
                }

                if(this.yawController == null) {
                        this.yawController = new PIDController(this.YKp, this.YKi, this.YKd, this.opMode, "Y");
                }

        }

        private Status navigateToRelativeLocation() {
                if (this.referenceTag == null || this.referenceTag.ftcPose == null) {
                        return Status.FAILURE;
                }

                double rangeError = referenceTag.ftcPose.range - this.currentTarget.range;
                double headingError = referenceTag.ftcPose.bearing - this.currentTarget.bearing;
                double yawError = referenceTag.ftcPose.yaw - this.currentTarget.yaw;

                opMode.telemetry.addData("Navigate1", "SetPoint range: %f heading: %f yaw: %f\n", currentTarget.range, currentTarget.bearing, currentTarget.yaw);
                opMode.telemetry.addData("Navigate1", "navigateToRelativeLocation rangeError: %f headingError: %f yawError: %f\n", rangeError, headingError, yawError);
                opMode.telemetry.update();






                if (Math.abs(rangeError) <= this.rangeErrorTolerance &&
                        Math.abs(headingError) <= this.headingErrorTolerance &&
                        Math.abs(yawError) <= this.yawErrorTolerance) {
                        opMode.telemetry.addData("Navigate0011", "Success rangeError: %f headingError: %f yawError: %f\n", rangeError, headingError, yawError);
                        opMode.telemetry.update();
                        return Status.SUCCESS;
                }
                opMode.telemetry.addData("Navigate2", "Reference range: %f bearing: %f yaw: %f\n", currentTarget.range, currentTarget.bearing, currentTarget.yaw);
                opMode.telemetry.update();



                drive = Range.clip(this.rangeController.output(this.currentTarget.range, referenceTag.ftcPose.range), -this.driveTrainConfig.maxAutoSpeed, this.driveTrainConfig.maxAutoSpeed);
                turn =   Range.clip(this.headingController.output(this.currentTarget.bearing, referenceTag.ftcPose.bearing), -this.driveTrainConfig.maxAutoTurn, this.driveTrainConfig.maxAutoTurn);
                strafe = Range.clip(-this.yawController.output(this.currentTarget.yaw, referenceTag.ftcPose.yaw), -this.driveTrainConfig.maxAutoStrafe, this.driveTrainConfig.maxAutoStrafe);

                //        drive = -Range.clip(Math.max (rangeError * this.driveTrainConfig.speedGain, rangeError * 0.1), -this.driveTrainConfig.maxAutoSpeed, this.driveTrainConfig.maxAutoSpeed);
         //       turn = -Range.clip(Math.max (headingError * this.driveTrainConfig.turnGain, headingError * 0.1), -this.driveTrainConfig.maxAutoTurn, this.driveTrainConfig.maxAutoTurn);
         //       strafe = -Range.clip(Math.max (-yawError * this.driveTrainConfig.strafeGain, -yawError * 0.1), -this.driveTrainConfig.maxAutoStrafe, this.driveTrainConfig.maxAutoStrafe);

               // opMode.telemetry.addData("Navigate3", "------navigateToRelativeLocation drive: %f strafe: %f turn: %f\n", drive, strafe, turn);
                opMode.telemetry.update();

                driveTrain.moveRobot(drive, strafe, turn);

                //this.opMode.sleep(2000);

                return Status.RUNNING;
        }
}
