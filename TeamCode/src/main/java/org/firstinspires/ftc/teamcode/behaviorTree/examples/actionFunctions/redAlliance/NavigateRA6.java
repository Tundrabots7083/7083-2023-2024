package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.ErrorTolerances;
import org.firstinspires.ftc.teamcode.models.PIDCoeficients;
import org.firstinspires.ftc.teamcode.models.RelativePosition;

public class NavigateRA6 extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    public NavigateRA6(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStore globalStore) {

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        RelativePosition currentTarget = new RelativePosition(4,16.3,-17,25);
        globalStore.setValue("CurrentTarget", currentTarget);

        globalStore.setValue("ReferenceAprilTagId",4);
        globalStore.setValue("YawStep",0.35);

        setPIDCoeficients(globalStore);
        setErrorTolerances(globalStore);


        Status status = super.perform(globalStore);

        lastStatus = status;

        return status;
    }

    private void setPIDCoeficients(GlobalStore globalStore){
        PIDCoeficients pidCoeficients = new PIDCoeficients();
        pidCoeficients.HKd=0.0;
        pidCoeficients.HKi=0.012;
        pidCoeficients.HKp=0.028;

        pidCoeficients.RKd=0;
        pidCoeficients.RKi=0.010;
        pidCoeficients.RKp=0.013;

        pidCoeficients.YKd=0;
        pidCoeficients.YKi=0.01;
        pidCoeficients.YKp=0.01;

        globalStore.setValue("PIDCoeficients", pidCoeficients);
    }

    private void setErrorTolerances(GlobalStore globalStore){
        ErrorTolerances errorTolerances = new ErrorTolerances();
        errorTolerances.headingErrorTolerance=10;
        errorTolerances.rangeErrorTolerance=5;
        errorTolerances.yawErrorTolerance=10;

        globalStore.setValue("ErrorTolerances", errorTolerances);

    }
}
