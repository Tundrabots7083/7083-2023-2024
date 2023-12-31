package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObject;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class LocalizeByAprilTag implements ActionFunction {
    private WorldModel worldModel;
    private AprilTagDetection currentDetections;
    private  int desiredTagId=-1;
    private AprilTagDetection referenceTag = null;

    private List<Pose2d> robotPoses = new ArrayList<Pose2d>();

    private LinearOpMode opMode;
    public LocalizeByAprilTag(LinearOpMode opMode) {
        this.opMode=opMode;
    }

    public Status perform(GlobalStore globalStore) {
       setRobotPoses(globalStore);

       //Calculate the estimated robot pose as the average of calculated poses
        Pose2d estPoseSum = robotPoses.stream().reduce(new Pose2d(0,0,0),(subtotal, element)->subtotal.plus(element));
        Pose2d estRobotPose = estPoseSum.div(robotPoses.size());

        globalStore.setValue("AprilTagRobotPose", estRobotPose);

        return Status.RUNNING;
    }

    private void setRobotPoses(GlobalStore globalStore){
        WorldModel worldModel =(WorldModel) globalStore.getValue("WorldModel");

        List<AprilTagDetection> currentDetections =  (List<AprilTagDetection>) globalStore.getValue("CurrentDetections");

        List<AprilTagDetection> validCurrentDetections = currentDetections.stream().filter(detection -> (detection.metadata != null)).collect(Collectors.toList());

        if (validCurrentDetections == null){
            return;
        }

        validCurrentDetections.forEach((AprilTagDetection robotDetection)->{
            WorldObject tagWorldObject = worldModel.getValue(String.valueOf(robotDetection.id));

            Pose2d aprilTagPose =tagWorldObject.position;

           robotPoses.add(calculateRobotPose(aprilTagPose, robotDetection));
        });
    }

    private Pose2d calculateRobotPose(Pose2d aprilTagPose, AprilTagDetection robotDetection){
        double degToRadRatio = Math.PI/180;

        double R = robotDetection.ftcPose.range;           // Range from AprilTag to robot
        double bearing = degToRadRatio * robotDetection.ftcPose.bearing; // Bearing angle in radians
        double yaw = degToRadRatio * robotDetection.ftcPose.yaw;   // Yaw angle in radians

        // Calculate relative position of the robot to the aprilTag
        double xRelative = R * Math.cos(bearing);
        double yRelative = R * Math.sin(bearing);

        opMode.telemetry.addData("LocalizeByAprilTag-", "calculateRobotPose Id: %d",robotDetection.id);
        opMode.telemetry.update();
        opMode.telemetry.addData("LocalizeByAprilTag-", "B: %f xRel: %f yRel: %f aprilTagPose Xr:%f; Yr:%f; Hr:%f;",bearing,xRelative,yRelative, aprilTagPose.getX(), aprilTagPose.getY(), aprilTagPose.getHeading());
        opMode.telemetry.update();

        double robotXAbsolute = aprilTagPose.getX() + xRelative * Math.cos(aprilTagPose.getHeading()*degToRadRatio) - yRelative * Math.sin(aprilTagPose.getHeading()*degToRadRatio);
        double robotYAbsolute = aprilTagPose.getY() + xRelative * Math.sin(aprilTagPose.getHeading()*degToRadRatio) + yRelative * Math.cos(aprilTagPose.getHeading()*degToRadRatio);
        double robotHeadingAbsolute = aprilTagPose.getHeading() + yaw;

        Pose2d robotPose = new Pose2d(robotXAbsolute,robotYAbsolute,robotHeadingAbsolute);


        return robotPose;
    }


}