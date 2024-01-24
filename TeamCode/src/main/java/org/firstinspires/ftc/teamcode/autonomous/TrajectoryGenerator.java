package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

public interface TrajectoryGenerator {

    Trajectory toSpikeMark(TrajectoryBuilder builder);

    Trajectory toParkingSpotCenter(TrajectoryBuilder builder);

    Trajectory toParkingSpotEdge(TrajectoryBuilder builder);
}
