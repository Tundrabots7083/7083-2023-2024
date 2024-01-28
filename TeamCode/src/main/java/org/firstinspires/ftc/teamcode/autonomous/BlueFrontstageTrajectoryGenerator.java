package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

public class BlueFrontstageTrajectoryGenerator implements TrajectoryGenerator {

    public static final Pose2d INNER_SPIKE_POSITION = new Pose2d(-39, 32, Math.toRadians(0));
    public static final int INNER_SPIKE_BASE_HEADING = 180;
    public static final Pose2d MIDDLE_SPIKE_POSITION = new Pose2d(-33.5, 41, Math.toRadians(-90));
    public static final int MIDDLE_SPIKE_BASE_HEADING = -90;
    public static final Pose2d OUTER_SPIKE_POSITION = new Pose2d(-49, 50, Math.toRadians(-90));
    public static final int OUTER_SPIKE_BASE_HEADING = -90;

    public static final Vector2d REVERSE_POSITION = new Vector2d(-27, 60);
    public static final Vector2d UNDER_STAGE_TARGET_POSITION = new Vector2d(-12, 60);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION = new Vector2d(20, 60);
    public static final Vector2d TURN_PARKING_POSITION = new Vector2d(40, 12);
    public static final Vector2d PARKING_POSITION_CENTER = new Vector2d(55, 10);
    public static final Vector2d PARKING_POSITION_EDGE = new Vector2d(59, 59);

    TeamElementLocation targetLocation;

    public BlueFrontstageTrajectoryGenerator(TeamElementLocation targetLocation) {
        this.targetLocation = targetLocation;
    }

    @Override
    public Trajectory toSpikeMark(TrajectoryBuilder builder) {
        // The first step is to drive the robot from the starting position to the correct spike mark.
        Pose2d spikePose;
        int spikeHeading;
        if (targetLocation == TeamElementLocation.LEFT) {
            spikePose = INNER_SPIKE_POSITION;
            spikeHeading = INNER_SPIKE_BASE_HEADING;
        } else if (targetLocation == TeamElementLocation.MIDDLE) {
            spikePose = MIDDLE_SPIKE_POSITION;
            spikeHeading = MIDDLE_SPIKE_BASE_HEADING;
        } else {
            spikePose = OUTER_SPIKE_POSITION;
            spikeHeading = OUTER_SPIKE_BASE_HEADING;
        }
        return builder.splineToLinearHeading(spikePose, Math.toRadians(spikeHeading))
                .build();
    }

    @Override
    public Trajectory toParkingSpotEdge(TrajectoryBuilder builder) {
        return builder.splineTo(REVERSE_POSITION, Math.toRadians(0))
                .splineTo(UNDER_STAGE_TARGET_POSITION, Math.toRadians(0))
                .splineTo(PARKING_POSITION_EDGE, Math.toRadians(0))
                .build();
    }

    public Trajectory toParkingSpotCenter(TrajectoryBuilder builder) {
        return builder.splineTo(REVERSE_POSITION, Math.toRadians(0))
                .splineToConstantHeading(UNDER_STAGE_TARGET_POSITION, Math.toRadians(0))
                .splineToConstantHeading(INTERMEDIATE_PARKING_POSITION, Math.toRadians(0))
                .splineTo(TURN_PARKING_POSITION, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_CENTER, Math.toRadians(0))
                .build();
    }
}
