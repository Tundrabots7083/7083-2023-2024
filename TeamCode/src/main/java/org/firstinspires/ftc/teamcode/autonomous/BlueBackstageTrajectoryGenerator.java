package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

@Config
public class BlueBackstageTrajectoryGenerator implements TrajectoryGenerator {

    public static final Pose2d INNER_SPIKE_BASE = new Pose2d(16, 35, Math.toRadians(180));
    public static final int INNER_SPIKE_BASE_HEADING = 180;
    public static final Pose2d MIDDLE_SPIKE_BASE = new Pose2d(11, 41, Math.toRadians(-90));
    public static final int MIDDLE_SPIKE_BASE_HEADING = -90;
    public static final Pose2d OUTER_SPIKE_BASE = new Pose2d(23.5, 50, Math.toRadians(-90));
    public static final int OUTER_SPIKE_BASE_HEADING = -90;

    public static final Vector2d BACKDROP_INTERMEDIATE_POSITION = new Vector2d(40.5, 43);
    public static final Vector2d BACKDROP_EDGE_POSITION = new Vector2d(41.80, 44);
    public static final Vector2d BACKDROP_MIDDLE_POSITION = new Vector2d(39, 36.5);
    public static final Vector2d BACKDROP_CENTER_POSITION = new Vector2d(39.6, 30.5);
    public static double BACKDROP_MIDDLE_ROTATE = 0;
    public static double BACKDROP_CENTER_ROTATE = 5;
    public static double BACKDROP_EDGE_ROTATE = 0;

    public static double BACKDROP_FORWARD_DISTANCE = 1.5;
    public static double BACKDROP_BACKWARD_DISTANCE = 3.0;

    public static final Vector2d REVERSE_POSITION = new Vector2d(27, 59);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION_CENTER = new Vector2d(45, 15);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION_EDGE = new Vector2d(45, 63);
    public static final Vector2d PARKING_POSITION_CENTER = new Vector2d(59, 15);
    public static final Vector2d PARKING_POSITION_EDGE = new Vector2d(59, 63);

    private final TeamElementLocation targetLocation;

    public BlueBackstageTrajectoryGenerator(TeamElementLocation targetLocation) {
        this.targetLocation = targetLocation;
    }

    @Override
    public Trajectory toSpikeMark(TrajectoryBuilder builder) {

        // The first step is to drive the robot from the starting position to the correct spike mark.
        Pose2d spikePose;
        int spikeHeading;
        if (targetLocation == TeamElementLocation.LEFT) {
            spikePose = OUTER_SPIKE_BASE;
            spikeHeading = OUTER_SPIKE_BASE_HEADING;
        } else if (targetLocation == TeamElementLocation.MIDDLE) {
            spikePose = MIDDLE_SPIKE_BASE;
            spikeHeading = MIDDLE_SPIKE_BASE_HEADING;
        } else {
            spikePose = INNER_SPIKE_BASE;
            spikeHeading = INNER_SPIKE_BASE_HEADING;
        }
        return builder.splineToLinearHeading(spikePose, Math.toRadians(spikeHeading))
                .build();
    }

    @Override
    public Trajectory toArmLiftPosition(TrajectoryBuilder builder) {
        if (targetLocation != TeamElementLocation.RIGHT) {
            builder.splineTo(REVERSE_POSITION, Math.toRadians(0));
        }
        switch (targetLocation) {
            case LEFT:
                builder.splineToConstantHeading(BACKDROP_EDGE_POSITION, Math.toRadians(BACKDROP_EDGE_ROTATE));
                break;
            case MIDDLE:
                builder.splineToConstantHeading(BACKDROP_INTERMEDIATE_POSITION, Math.toRadians(0))
                        .splineToConstantHeading(BACKDROP_MIDDLE_POSITION, Math.toRadians(BACKDROP_MIDDLE_ROTATE));
                break;
            default:
                builder.splineToConstantHeading(BACKDROP_INTERMEDIATE_POSITION, Math.toRadians(0))
                        .splineToConstantHeading(BACKDROP_CENTER_POSITION, Math.toRadians(BACKDROP_CENTER_ROTATE));
        }
        return builder.build();
    }

    @Override
    public Trajectory toBackdropPosition(TrajectoryBuilder builder) {
        return toBackdropPosition(builder, BACKDROP_FORWARD_DISTANCE);
    }

    @Override
    public Trajectory toBackdropPosition(TrajectoryBuilder builder, double distance) {
        return builder.back(distance)
                .build();
    }

    @Override
    public Trajectory toArmRetractionPosition(TrajectoryBuilder builder) {
        return builder.forward(BACKDROP_BACKWARD_DISTANCE)
                .build();
    }

    @Override
    public Trajectory toParkingSpotCenter(TrajectoryBuilder builder) {
        return builder.splineToConstantHeading(INTERMEDIATE_PARKING_POSITION_CENTER, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_CENTER, Math.toRadians(0))
                .build();
    }

    @Override
    public Trajectory toParkingSpotEdge(TrajectoryBuilder builder) {
        return builder.splineToConstantHeading(INTERMEDIATE_PARKING_POSITION_EDGE, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_EDGE, Math.toRadians(0))
                .build();
    }
}
