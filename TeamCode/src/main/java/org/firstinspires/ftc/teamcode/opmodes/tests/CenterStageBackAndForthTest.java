package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageORMechanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageORMechanumDriveSingleton;
import org.firstinspires.ftc.teamcode.subsystems.StandardTrajectoryBuilder;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class CenterStageBackAndForthTest extends LinearOpMode {

    public static double DISTANCE = 20;

    Pose2d startingPose;
    private Trajectory traj01;
    private Trajectory traj02;
    private Trajectory traj03;
    private Trajectory traj04;

    CenterStageORMechanumDriveSingleton drive = null;
    SampleMecanumDrive drive1 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalStoreSingleton globalStore = GlobalStoreSingleton.getInstance(this);
        startingPose = new Pose2d(-5, 0, Math.toRadians(0));


        CenterStageORMechanumDriveSingleton.reset();
            this.drive = CenterStageORMechanumDriveSingleton.getInstance(hardwareMap, this, globalStore);
        //set drive starting pose
            this.drive.setPoseEstimate(startingPose);




      setTrajectory(globalStore);
        waitForStart();


        drive.followTrajectory(traj01);
      //  drive.followTrajectory(traj02);
      //  drive.followTrajectory(traj03);
      //  drive.followTrajectory(traj04);


    }

    private void driveRectangleWithTurns(GlobalStoreSingleton globalStore){

        setTurningTrajectory(globalStore);

        drive.followTrajectory(traj01);
        drive.turn(Math.toRadians(-90));

        /*
        drive.followTrajectory(traj02);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traj03);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traj04);

         */
    }

    private void setTrajectory(GlobalStoreSingleton globalStore){
        setStrafingTrajectory(globalStore);
        /*
        StandardTrajectoryBuilder standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);

        traj01 = standardTrajectoryBuilder.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        traj02 = standardTrajectoryBuilder.trajectoryBuilder(traj01.end())
                .strafeRight(20)
                .build();

        traj03 = standardTrajectoryBuilder.trajectoryBuilder(traj02.end())
                .back(DISTANCE)
                .build();


        traj04 = standardTrajectoryBuilder.trajectoryBuilder(traj03.end())
                .strafeLeft(20)
                .build();
*/
       // globalStore.setValue("CurrentTrajectory", currentTrajectory);
    }

    private void setStrafingTrajectory(GlobalStoreSingleton globalStore){
        StandardTrajectoryBuilder standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);

        traj01 = standardTrajectoryBuilder.trajectoryBuilder(this.startingPose)
                .forward(DISTANCE)
                .build();

        traj02 = standardTrajectoryBuilder.trajectoryBuilder(traj01.end())
                .strafeRight(20)
                .build();

        traj03 = standardTrajectoryBuilder.trajectoryBuilder(traj02.end())
                .back(DISTANCE)
                .build();


        traj04 = standardTrajectoryBuilder.trajectoryBuilder(traj03.end())
                .strafeLeft(20)
                .build();
    }

    private void setTurningTrajectory(GlobalStoreSingleton globalStore){
        StandardTrajectoryBuilder standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);



        traj01 = standardTrajectoryBuilder.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        traj02 = standardTrajectoryBuilder.trajectoryBuilder(traj01.end())
                .forward(20)
                .build();

        traj03 = standardTrajectoryBuilder.trajectoryBuilder(traj02.end())
                .forward(DISTANCE)
                .build();


        traj04 = standardTrajectoryBuilder.trajectoryBuilder(traj03.end())
                .forward(20)
                .build();
    }
}