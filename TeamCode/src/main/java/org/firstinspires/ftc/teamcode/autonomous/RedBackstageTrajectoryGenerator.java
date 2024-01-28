package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

@Config
public class RedBackstageTrajectoryGenerator implements TrajectoryGenerator {
    public static final Pose2d INNER_SPIKE_BASE = new Pose2d(16, -35, Math.toRadians(-180));
    public static final int INNER_SPIKE_BASE_HEADING = 90;
    public final Pose2d MIDDLE_SPIKE_BASE = new Pose2d(11, -42, Math.toRadians(90));
    public static int MIDDLE_SPIKE_BASE_HEADING = 0;
    public static final Pose2d OUTER_SPIKE_BASE = new Pose2d(25, -50, Math.toRadians(90));
    public static final int OUTER_SPIKE_BASE_HEADING = 0;

    public static final Vector2d REVERSE_POSITION = new Vector2d(27, -55);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION = new Vector2d(50, -15);
    public static final Vector2d PARKING_POSITION_CENTER = new Vector2d(59, -15);
    public static final Vector2d PARKING_POSITION_EDGE = new Vector2d(59, -63);

    TeamElementLocation targetLocation;

    public RedBackstageTrajectoryGenerator(TeamElementLocation targetLocation) {
        this.targetLocation = targetLocation;
    }

    @Override
    public Trajectory toSpikeMark(TrajectoryBuilder builder) {

        // The first step is to drive the robot from the starting position to the correct spike mark.
        Pose2d spikePose;
        int spikeHeading;
        if (targetLocation == TeamElementLocation.LEFT) {
            spikePose = INNER_SPIKE_BASE;
            spikeHeading = INNER_SPIKE_BASE_HEADING;
        } else if (targetLocation == TeamElementLocation.MIDDLE) {
            spikePose = MIDDLE_SPIKE_BASE;
            spikeHeading = MIDDLE_SPIKE_BASE_HEADING;
        } else {
            spikePose = OUTER_SPIKE_BASE;
            spikeHeading = OUTER_SPIKE_BASE_HEADING;
        }
        return builder.splineToLinearHeading(spikePose, Math.toRadians(spikeHeading))
                .build();
    }

    @Override
    public Trajectory toParkingSpotCenter(TrajectoryBuilder builder) {
        if (targetLocation != TeamElementLocation.LEFT) {
            builder.splineTo(REVERSE_POSITION, Math.toRadians(0));
        }
        return builder.splineTo(INTERMEDIATE_PARKING_POSITION, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_CENTER, Math.toRadians(0))
                .build();
    }

    @Override
    public Trajectory toParkingSpotEdge(TrajectoryBuilder builder) {
        return builder.splineTo(REVERSE_POSITION, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_EDGE, Math.toRadians(0))
                .build();
    }
}
