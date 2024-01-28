package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class CenterStageOROdometryWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV =8192;// 0;
    public static double WHEEL_RADIUS = 1.1811;// 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 7.585824; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4.5; // in; offset of the lateral wheel

    public static double X_MULTIPLIER =1;
    public static double Y_MULTIPLIER =1.00515084;
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;
    private LinearOpMode opMode;


    public CenterStageOROdometryWheelLocalizer(LinearOpMode opMode, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        this.opMode = opMode;

        leftEncoder = new Encoder(this.opMode.hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(this.opMode.hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(this.opMode.hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public void update(){
        super.update();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos)*X_MULTIPLIER,
                encoderTicksToInches(rightPos)*X_MULTIPLIER,
                encoderTicksToInches(frontPos)*Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel)*X_MULTIPLIER,
                encoderTicksToInches(rightVel)*X_MULTIPLIER,
                encoderTicksToInches(frontVel)*Y_MULTIPLIER
        );
    }
}
