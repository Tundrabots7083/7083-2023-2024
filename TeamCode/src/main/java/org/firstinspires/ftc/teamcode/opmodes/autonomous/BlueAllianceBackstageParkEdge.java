package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.BlueBackstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.PixelMover;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

@Autonomous(name="Blue Alliance Backstage Park Edge", group="Autonomous")
public class BlueAllianceBackstageParkEdge extends LinearOpMode {

    public static final Pose2d STARTING_POSE = new Pose2d(12, 63.5, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        VisionSensor visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"));
        visionSensor.initializeVisionPortal();

        AutoMecanumDrive drive = new AutoMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(STARTING_POSE);

        PixelMover pixelMover = new PixelMover("pixelMover", "Collects pixels and moves them", hardwareMap);
        Arm arm = new Arm("arm", "Arm", hardwareMap);

        // Wait for webcam to initialize
        while(!visionSensor.webcamInitialized()) {}

        telemetry.addData("Webcam", "Initialized");
        telemetry.update();

        waitForStart();

        TeamElementLocation element = visionSensor.getTeamElementLocation();
        telemetry.addData("Element", element);
        telemetry.update();
        visionSensor.close();

        telemetry.addLine("Lock the pixels");
        telemetry.update();
        pixelMover.start(telemetry, true);

        telemetry.addLine("Lower the pixel container");
        telemetry.update();
        // TODO: See if this code is needed
        // arm.setTarget(Arm.Position.Start);
        // arm.update();
        arm.setTarget(Arm.Position.Intake);
        arm.update();

        TrajectoryGenerator trajectoryGenerator = new BlueBackstageTrajectoryGenerator(element);

        Trajectory toSpikeMark = trajectoryGenerator.toSpikeMark(drive.trajectoryBuilder(STARTING_POSE));

        // Drive to the correct spike mark
        telemetry.addLine("Drive to spike mark");
        telemetry.update();
        drive.followTrajectory(toSpikeMark);

        telemetry.addLine("Drop off purple pixel");
        telemetry.update();
        // Deposit the purple pixel
        pixelMover.dropOffTopPixel(telemetry);

        telemetry.addLine("Lift arm");
        telemetry.update();
        // Drive to the backdrop
        Trajectory toArmLiftPosition = trajectoryGenerator.toArmLiftPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toArmLiftPosition);

        telemetry.addLine("Raise arm");
        arm.setTarget(Arm.Position.ScoreLow);
        while (!arm.isAtTarget()) {
            arm.update();
        }
        arm.stopArm();

        telemetry.addLine("Score yellow pixel on backdrop");
        telemetry.update();
        Trajectory toBackdropPosition = trajectoryGenerator.toArmLiftPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toArmLiftPosition);
        pixelMover.dropOffBottomPixel(telemetry);

        telemetry.addLine("Move away from backdrop");
        telemetry.update();
        Trajectory toArmRetractionPosition = trajectoryGenerator.toArmRetractionPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toArmRetractionPosition);

        telemetry.addLine("Lower arm");
        telemetry.update();
        arm.setTarget(Arm.Position.Intake);
        while (!arm.isAtTarget()) {
            arm.update();
        }
        arm.stopArm();

        telemetry.addLine("Drive to parking spot");
        telemetry.update();
        // Drive to the parking spot
        Trajectory toParkingSpot = trajectoryGenerator.toParkingSpotEdge(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toParkingSpot);
    }
}
