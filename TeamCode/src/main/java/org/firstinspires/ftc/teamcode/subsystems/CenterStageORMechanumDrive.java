package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.kA;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.models.CenterStageORDriveConstants.kV;

/* roadrunner constants

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;

 */

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CenterStageORMechanumDrive extends Drive {
    private double trackWidth;
    private double wheelBase = trackWidth;
    private double lateralMultiplier = 1.0;


    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8.7, 0, 0.98);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8.7, 0, 0.98);

    public static double LATERAL_MULTIPLIER = 1.25;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    private LinearOpMode opMode;
    private DriveTrain driveTrain;

    public Localizer localizer;




    public CenterStageORMechanumDrive(HardwareMap hardwareMap, LinearOpMode opMode, GlobalStoreSingleton globalStore) {
        this.trackWidth = TRACK_WIDTH;
        this.wheelBase = TRACK_WIDTH;
        this.lateralMultiplier = LATERAL_MULTIPLIER;
        this.opMode = opMode;
        this.driveTrain = new DriveTrain(opMode);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        org.firstinspires.ftc.teamcode.util.LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //what does this code do?
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (RUN_USING_ENCODER) {
            driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            driveTrain.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID, batteryVoltageSensor.getVoltage());
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

       localizer = new CenterStageORAgregateLocalizer(globalStore,opMode, lastTrackingEncPositions, lastTrackingEncVels);
        // setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

       this.setLocalizer(localizer);

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

    }

    @NonNull
    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    @Override
    public void setLocalizer(@NonNull Localizer localizer) {
        this.localizer=localizer;
    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        List<Double>  powers = MecanumKinematics.robotToWheelVelocities(
                drivePower,
                1.0,
                1.0,
                lateralMultiplier
        );
        driveTrain.setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    @Override
    public void setDriveSignal(@NonNull DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(
                driveSignal.getVel(),
                trackWidth,
                wheelBase,
                lateralMultiplier
        );

        List<Double>  accelerations = MecanumKinematics.robotToWheelAccelerations(
                driveSignal.getAccel(),
                trackWidth,
                wheelBase,
                lateralMultiplier
        );

        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        driveTrain.setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
       waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
       waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }
    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }
}
