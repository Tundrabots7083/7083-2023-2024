package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

public class BlueBackstageParkCenterTrajectoryGenerator implements TrajectoryGenerator {


    public static final Pose2d BACK_STAGE_INNER_SPIKE_BASE = new Pose2d(16, 32, Math.toRadians(180));
    public static final int INNER_SPIKE_BASE_HEADING = 180;
    public static final Pose2d BACK_STAGE_MIDDLE_SPIKE_BASE = new Pose2d(11, 43, Math.toRadians(-90));
    public static final int MIDDLE_SPIKE_BASE_HEADING = -90;
    public static final Pose2d BACK_STAGE_OUTER_SPIKE_BASE = new Pose2d(23, 49, Math.toRadians(-90));
    public static final int OUTER_SPIKE_BASE_HEADING = -90;

    public static final Pose2d BACK_STAGE_REVERSE_POSITION = new Pose2d(27, 55, Math.toRadians(180));
    public static final Pose2d BACK_STAGE_INTERMEDIATE_PARKING_POSITION = new Pose2d(50, 15, Math.toRadians(180));
    public static final Pose2d BACK_STAGE_PARKING_POSITION = new Pose2d(59, 15, Math.toRadians(180));

    TeamElementLocation targetLocation;

    public BlueBackstageParkCenterTrajectoryGenerator(TeamElementLocation targetLocation) {
        this.targetLocation = targetLocation;
    }

    @Override
    public Trajectory toSpikeMark(TrajectoryBuilder builder) {

        // The first step is to drive the robot from the starting position to the correct spike mark.
        Pose2d spikePose;
        int spikeHeading = 0;
        if (targetLocation == TeamElementLocation.LEFT) {
            spikePose = BACK_STAGE_OUTER_SPIKE_BASE;
            spikeHeading = OUTER_SPIKE_BASE_HEADING;
        } else if (targetLocation == TeamElementLocation.MIDDLE) {
            spikePose = BACK_STAGE_MIDDLE_SPIKE_BASE;
            spikeHeading = MIDDLE_SPIKE_BASE_HEADING;
        } else {
            spikePose = BACK_STAGE_INNER_SPIKE_BASE;
            spikeHeading = INNER_SPIKE_BASE_HEADING;
        }
        return builder.splineToLinearHeading(spikePose, Math.toRadians(spikeHeading))
                .build();
    }

    public Trajectory toReversePosition(TrajectoryBuilder builder) {
        return builder.splineToLinearHeading(BACK_STAGE_REVERSE_POSITION, Math.toRadians(0))
            .build();
    }

    public Trajectory toIntermediateParkingSpot(TrajectoryBuilder builder) {
        return builder.splineToLinearHeading(BACK_STAGE_INTERMEDIATE_PARKING_POSITION, Math.toRadians(0))
        .build();
    }

    @Override
    public Trajectory toParkingSpot(TrajectoryBuilder builder) {
        return builder.splineToLinearHeading(BACK_STAGE_PARKING_POSITION, Math.toRadians(0))
                .build();
    }
}
