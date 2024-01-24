package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.BlueFrontstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PixelMover;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

@Autonomous(name="Blue Alliance Frontstage Park Edge", group="Autonomous")
public class BlueAllianceFrontstageParkEdge extends LinearOpMode {

    public static final Pose2d STARTING_POSE = new Pose2d(-36, 63.5, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {

        VisionSensor visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"));

        AutoMecanumDrive drive = new AutoMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(STARTING_POSE);

        PixelMover pixelMover = new PixelMover("pixelMover", "Collects pixels and moves them", hardwareMap);

        visionSensor.initializeVisionPortal();

        while(!visionSensor.webcamInitialized()) {}

        telemetry.addData("Webcam", "Initialized");
        telemetry.update();

        waitForStart();

        visionSensor.close();

        TeamElementLocation element = visionSensor.getTeamElementLocation();

        telemetry.addData("Element", element);
        telemetry.update();

        BlueFrontstageTrajectoryGenerator trajectoryGenerator = new BlueFrontstageTrajectoryGenerator(element);

        Trajectory toSpikeMark = trajectoryGenerator.toSpikeMark(drive.trajectoryBuilder(STARTING_POSE));

        // Drive to the correct spike mark
        telemetry.addLine("Driving to spike mark");
        telemetry.update();
        drive.followTrajectory(toSpikeMark);

        telemetry.addLine("Dropping off pixels");
        telemetry.update();
        // Deposit the purple pixel
        pixelMover.dropOffTopPixel(telemetry);

        sleep(3000);

        telemetry.addLine("Driving to parking spot");
        telemetry.update();

        // Drive to the parking spot
        Trajectory toParkingSpot = trajectoryGenerator.toParkingSpotEdge(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toParkingSpot);

        while (opModeIsActive()) {
            // Do nothing
            idle();
        }
    }
}
