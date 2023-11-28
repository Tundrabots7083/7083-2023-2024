package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.ErrorTolerances;
import org.firstinspires.ftc.teamcode.models.PIDCoeficients;
import org.firstinspires.ftc.teamcode.models.RelativePosition;

public class NavigateRA1 extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    public NavigateRA1(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStore globalStore) {

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        setRelativePositionTargetParameters(globalStore);
        setPIDCoeficients(globalStore);
        setErrorTolerances(globalStore);

        Status status = super.perform(globalStore);

        lastStatus = status;

        return status;
    }

    private void setRelativePositionTargetParameters(GlobalStore globalStore){
        RelativePosition currentTarget = new RelativePosition(7,30,0,0);
        globalStore.setValue("CurrentTarget", currentTarget);

        globalStore.setValue("ReferenceAprilTagId",7);
        globalStore.setValue("YawStep",-0.25);
    }
    private void setAbsolutePositionTargetParameters(GlobalStore globalStore){
        //this is a place holder method
        /*
        RelativePosition currentTarget = new RelativePosition(7,30,0,0);
        globalStore.setValue("CurrentTarget", currentTarget);

        globalStore.setValue("ReferenceAprilTagId",7);
        */
    }
    private void setPIDCoeficients(@NonNull GlobalStore globalStore){
        PIDCoeficients pidCoeficients = new PIDCoeficients();
        pidCoeficients.HKd=0.0;
        pidCoeficients.HKi=0.022;
        pidCoeficients.HKp=0.028;

        pidCoeficients.RKd=0;
        pidCoeficients.RKi=0.15;
        pidCoeficients.RKp=0.023;

        pidCoeficients.YKd=0.01;
        pidCoeficients.YKi=0.01;
        pidCoeficients.YKp=0.015;

        globalStore.setValue("PIDCoeficients", pidCoeficients);
    }

    private void setErrorTolerances(GlobalStore globalStore){
        ErrorTolerances errorTolerances = new ErrorTolerances();
        errorTolerances.headingErrorTolerance=0.75;
        errorTolerances.rangeErrorTolerance=0.75;
        errorTolerances.yawErrorTolerance=0.75;

        globalStore.setValue("ErrorTolerances", errorTolerances);

    }
}
