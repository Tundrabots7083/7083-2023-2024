package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.behaviorTrees.RedAudienceAutonomous;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;

@Autonomous(name="BT Drive To AprilTag", group="vision")
public class RedAudienceAutonomousOpMode extends LinearOpMode
{
    RedAudienceAutonomous behaviorTree = null;

    @Override
    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("BTAutoDriveToAprilTagOpMode", "runOpMode started");
        telemetry.update();
        initialize(this);
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("BTAutoDriveToAprilTagOpMode", "runOpMode while started");
            telemetry.update();
            Status result = this.behaviorTree.run();

            telemetry.addData("RedAudienceAutonomous", "Behavior tree result: %s",result);
            telemetry.update();

            if(result == Status.SUCCESS){
                telemetry.addData("BTAutoDriveToAprilTagOpMode", "runOpMode success");
                telemetry.update();
                //stop();
                requestOpModeStop();
            }
        }
    }

    private void initialize(LinearOpMode opMode){

        this.behaviorTree = new RedAudienceAutonomous(opMode);
    }

}
