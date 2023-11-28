package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class DriveTrain {
    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel

    private DcMotorEx specialMotor = null;
private LinearOpMode opMode;
    public DriveTrain(LinearOpMode opMode){
        this.opMode=opMode;
        initDriveTrain();
    }

    private void initDriveTrain(){
        //back camera
        leftFrontDrive  = opMode.hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "leftRear");
        leftBackDrive  = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");



        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

//front camera
        /*
        leftFrontDrive  = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackDrive  = opMode.hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, "rightRear");
        */
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
    }
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeft    =  x -y -yaw;
        double frontRight   =  x +y +yaw;
        double rearLeft     =  x +y -yaw;
        double rearRight    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        max = Math.max(max, Math.abs(rearLeft));
        max = Math.max(max, Math.abs(rearRight));

        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            rearLeft /= max;
            rearRight /= max;
        }

        // Send powers to the wheels.
         leftFrontDrive.setPower(frontLeft);
         rightFrontDrive.setPower(frontRight);
         leftBackDrive.setPower(rearLeft);
         rightBackDrive.setPower(rearRight);
    }

    public void setMotorPowers(double frontLeft, double rearLeft, double rearRight, double frontRight ){
        leftFrontDrive.setPower(frontLeft);
        leftBackDrive.setPower(rearLeft);
        rightBackDrive.setPower(rearRight);
        rightFrontDrive.setPower(frontRight);
    }

    public void setMode(DcMotor.RunMode runMode) {
        leftFrontDrive.setMode(runMode);
        leftBackDrive.setMode(runMode);
        rightBackDrive.setMode(runMode);
        rightFrontDrive.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFrontDrive.setZeroPowerBehavior(zeroPowerBehavior);
        leftBackDrive.setZeroPowerBehavior(zeroPowerBehavior);
        rightBackDrive.setZeroPowerBehavior(zeroPowerBehavior);
        rightFrontDrive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients, double voltage) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / voltage
        );

        leftFrontDrive.setPIDFCoefficients(runMode, compensatedCoefficients);
        leftBackDrive.setPIDFCoefficients(runMode, compensatedCoefficients);
        rightBackDrive.setPIDFCoefficients(runMode, compensatedCoefficients);
        rightFrontDrive.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

}