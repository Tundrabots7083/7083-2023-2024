package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.DriveTrainConfig;
import org.firstinspires.ftc.teamcode.models.RelativePosition;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class NavigateRef implements ActionFunction {
    private DriveTrain driveTrain;

    private AprilTagDetection desiredTag = null;
    private double drive = 0; // Desired forward power/speed (-1 to +1)
    private double strafe = 0; // Desired strafe power/speed (-1 to +1)
    private double turn = 0; // Desired turning power/speed (-1 to +1)

    private DriveTrainConfig driveTrainConfig=null;
    private  double desiredTargetDistance=12.0;
    private  int desiredTagId=-1;

    private LinearOpMode opMode;
    private RelativePosition currentTarget;
    private List<AprilTagDetection> currentDetections;
    public NavigateRef(LinearOpMode opMode) {
        this.opMode=opMode;
        this.driveTrain = new DriveTrain(opMode);
    }

    public Status perform(GlobalStoreSingleton globalStore) {
        this.driveTrainConfig =(DriveTrainConfig) globalStore.getValue("DriveTrainConfig");
        this.desiredTargetDistance =(double) globalStore.getValue("DesiredTargetDistance");
        this.desiredTagId =(int) globalStore.getValue("DesiredTagId");
        this.currentTarget =(RelativePosition) globalStore.getValue("CurrentTarget");

        getCurrentDetections(globalStore);
        navigateToTarget();

        return Status.SUCCESS;
    }

    private void getCurrentDetections(GlobalStoreSingleton globalStore){
        this.currentDetections =  (List<AprilTagDetection>) globalStore.getValue("CurrentDetections");
        opMode.telemetry.addData("getCurrentDetections", "Desired tag id ID %d \n", this.desiredTagId);


        desiredTag = currentDetections.stream().filter(detection -> detection.metadata != null && detection.id == this.desiredTagId).findFirst( ).orElse(null);

        if (desiredTag != null) {
            opMode.telemetry.addData("Navigate", "Tag ID %d metadata.id %d name is %s\n", desiredTag.id, desiredTag.metadata.id, desiredTag.metadata.name);
        }else {
            opMode.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", -1);
        }
        opMode.telemetry.update();
    }
    private void navigateToCurrentTarget() {
        if(this.currentTarget == null || this.desiredTag == null){return;}


        // Determine heading, range and Yaw (tag image rotation)
         double rangeError = (desiredTag.ftcPose.range - this.currentTarget.range);
         double bearingError = (desiredTag.ftcPose.bearing - this.currentTarget.bearing);
         double yawError = (desiredTag.ftcPose.yaw - this.currentTarget.yaw);

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive = -Range.clip(rangeError * this.driveTrainConfig.speedGain, -this.driveTrainConfig.maxAutoSpeed, this.driveTrainConfig.maxAutoSpeed);
        turn = -Range.clip(bearingError * this.driveTrainConfig.turnGain, -this.driveTrainConfig.maxAutoTurn, this.driveTrainConfig.maxAutoTurn);
        strafe = -Range.clip(-yawError * this.driveTrainConfig.strafeGain, -this.driveTrainConfig.maxAutoStrafe, this.driveTrainConfig.maxAutoStrafe);

        driveTrain.moveRobot(drive, strafe, turn);

        opMode.sleep(10);
    }

    private void navigateToTarget() {
double test = Math.atan2(2.5,-3);
        // Determine heading, range and Yaw (tag image rotation) error so we can use
        // them to control the robot automatically.
        double rangeError = (desiredTag.ftcPose.range - this.desiredTargetDistance);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive = -Range.clip(rangeError * this.driveTrainConfig.speedGain, -this.driveTrainConfig.maxAutoSpeed, this.driveTrainConfig.maxAutoSpeed);
        turn = -Range.clip(headingError * this.driveTrainConfig.turnGain, -this.driveTrainConfig.maxAutoTurn, this.driveTrainConfig.maxAutoTurn);
        strafe = -Range.clip(-yawError * this.driveTrainConfig.strafeGain, -this.driveTrainConfig.maxAutoStrafe, this.driveTrainConfig.maxAutoStrafe);

        driveTrain.moveRobot(drive, strafe, turn);

        opMode.sleep(10);
    }
}
