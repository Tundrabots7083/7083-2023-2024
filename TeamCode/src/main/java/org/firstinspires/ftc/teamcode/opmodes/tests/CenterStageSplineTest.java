package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageORMechanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageORMechanumDriveSingleton;
import org.firstinspires.ftc.teamcode.subsystems.StandardTrajectoryBuilder;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class CenterStageSplineTest extends LinearOpMode {
    private Trajectory traj01;
    private Trajectory traj02;
    private Trajectory traj03;
    private Trajectory traj04;

    CenterStageORMechanumDriveSingleton drive = null;
    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive1 = new SampleMecanumDrive(hardwareMap);

        GlobalStoreSingleton globalStore = GlobalStoreSingleton.getInstance(this);

        if(this.drive == null) {
            this.drive = CenterStageORMechanumDriveSingleton.getInstance(hardwareMap, this, globalStore);
        }
        setTrajectory(globalStore);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj01);

        sleep(1000);

        drive.followTrajectory(traj02);

/*
        drive.followTrajectory(traj03);

        sleep(1000);

      //  drive.followTrajectory(traj04);
*/
    }

    private void setTrajectory(GlobalStoreSingleton globalStore){
        StandardTrajectoryBuilder standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);

        traj01 = standardTrajectoryBuilder.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 20), Math.toRadians(90))
                .build();

        traj02 = standardTrajectoryBuilder.trajectoryBuilder(traj01.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();


        traj03 = standardTrajectoryBuilder.trajectoryBuilder(new Pose2d(-36,-60,Math.toRadians(90)))
                .splineTo(new Vector2d(-40, -56), Math.toRadians(180))
                .build();

        traj03 = standardTrajectoryBuilder.trajectoryBuilder(traj03.end(), true)
                .splineTo(new Vector2d(-24, -60), Math.toRadians(0))
                .build();

    }
}

