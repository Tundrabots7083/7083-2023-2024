package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        /*
        leftFrontDrive  = opMode.hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "leftRear");
        leftBackDrive  = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        */

//front camera
        leftFrontDrive  = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackDrive  = opMode.hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
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
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
         leftFrontDrive.setPower(leftFrontPower);
         rightFrontDrive.setPower(rightFrontPower);
         leftBackDrive.setPower(leftBackPower);
         rightBackDrive.setPower(rightBackPower);
    }
}