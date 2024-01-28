package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.NavigationType;
import org.firstinspires.ftc.teamcode.subsystems.StandardTrajectoryBuilder;

public class NavigateRA1Trajectory  extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    public NavigateRA1Trajectory(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStoreSingleton globalStore) {

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        setTrajectory(globalStore);
        setNavigationType(globalStore);

        Status status = super.perform(globalStore);

        lastStatus = status;

        return status;

    }

    private void setTrajectory(GlobalStoreSingleton globalStore){
        StandardTrajectoryBuilder standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);

      Pose2d lastRobotPose = (Pose2d)globalStore.getValue("LastRobotPose");

      if(lastRobotPose==null){
          lastRobotPose = new Pose2d(0,0,Math.toRadians(90));

      }

      //  opMode.telemetry.addData("NavigateRA1Trajectory", lastRobotPose.toString());
       // opMode.telemetry.update();

        double DISTANCE = 20;

        Trajectory currentTrajectory1 = standardTrajectoryBuilder.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory currentTrajectory  = standardTrajectoryBuilder.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 20), Math.toRadians(90))
                .build();


        //opMode.telemetry.addData("NavigateRA1Trajectory currentTrajectory", currentTrajectory.toString());
        //opMode.telemetry.update();
        globalStore.setValue("CurrentTrajectory", currentTrajectory);
    }
    private void setNavigationType(GlobalStoreSingleton globalStore){

        globalStore.setValue("NavigationType", NavigationType.ABSOLUTE);
    }
}
