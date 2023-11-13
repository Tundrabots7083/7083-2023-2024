package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.Position;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObject;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
//this is a place holder for an action that gets the robot's relative position from an april tag
// and calculates the robot's absolute position
public class CalculateLocation implements ActionFunction {
    private WorldModel worldModel;
    private AprilTagDetection currentDetections;
    private  int desiredTagId=-1;
    private AprilTagDetection referenceTag = null;

    private LinearOpMode opMode;
    public CalculateLocation(LinearOpMode opMode) {
        this.opMode=opMode;
    }



    public Status perform(GlobalStore globalStore) {
        WorldModel worldModel =(WorldModel) globalStore.getValue("WorldModel");
        int desiredTagId =(int)globalStore.getValue("DesiredTagId");

        WorldObject tagWorldObject =(WorldObject) worldModel.getValue(String.valueOf(desiredTagId));

        Position fixedAprilTagLocalization =tagWorldObject.position;
        getCurrentDetections(globalStore);

        double R = referenceTag.ftcPose.range;           // Range from AprilTag to robot
        double theta = Math.PI /referenceTag.ftcPose.bearing; // Bearing angle in radians
        double yaw = Math.PI / referenceTag.ftcPose.yaw;   // Yaw angle in radians

        Position robotLocalization = new Position(fixedAprilTagLocalization.X + R*Math.cos(theta+yaw),
                                                  fixedAprilTagLocalization.Y + R * Math.sin(theta + yaw) );


        globalStore.setValue("RobotLocation", robotLocalization);

        return Status.SUCCESS;
    }

    private void getCurrentDetections(GlobalStore globalStore){
        List<AprilTagDetection> currentDetections =  (List<AprilTagDetection>) globalStore.getValue("CurrentDetections");

        referenceTag = currentDetections.stream().filter(detection -> (detection.metadata != null) && (detection.id == this.desiredTagId)).findFirst( ).orElse(null);
        if (referenceTag != null) {
            opMode.telemetry.addData("Navigate", "Tag ID %d metadata.id %d name is %s\n", referenceTag.id, referenceTag.metadata.id, referenceTag.metadata.name);
        }else {
            opMode.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", referenceTag.id);
        }
    }

}