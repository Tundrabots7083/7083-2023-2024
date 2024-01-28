package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;

import java.util.List;

//Implement localizer that takes robot poses from
//both AprilTag and from Odometry
//and provides the calculated value to the mech drive
public class CenterStageORAgregateLocalizer implements Localizer {
    private Pose2d poseEstimate;
    private Pose2d poseVelocity;

    private CenterStageOROdometryWheelLocalizer odometryWheelLocalizer;
    private Pose2d aprilTagRobotPose;
    private LinearOpMode opMode;
    private GlobalStoreSingleton globalStore;

    public CenterStageORAgregateLocalizer(GlobalStoreSingleton globalStore, LinearOpMode opMode, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels){
        this.globalStore = globalStore;
        this.opMode=opMode;

        odometryWheelLocalizer= new CenterStageOROdometryWheelLocalizer(opMode,  lastTrackingEncPositions, lastTrackingEncVels);

        aprilTagRobotPose = (Pose2d)globalStore.getValue("AprilTagRobotPose");
    }


    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        if(aprilTagRobotPose !=null){
            poseEstimate = aprilTagRobotPose;
        } else {
            poseEstimate=odometryWheelLocalizer.getPoseEstimate();
        }
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate=pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        poseVelocity = odometryWheelLocalizer.getPoseVelocity();
        return poseVelocity;
    }

    @Override
    public void update() {
        aprilTagRobotPose = (Pose2d)globalStore.getValue("AprilTagRobotPose");
        odometryWheelLocalizer.update();
    }
}
