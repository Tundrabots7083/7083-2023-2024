package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.ErrorTolerances;
import org.firstinspires.ftc.teamcode.models.PIDCoeficients;
import org.firstinspires.ftc.teamcode.models.RelativePosition;

public class NavigateRA4 extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    public NavigateRA4(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStore globalStore) {

        if(lastStatus == Status.SUCCESS){
            opMode.sleep(30);//allow the system time to change the referenceTargetId

            return lastStatus;
        }

        RelativePosition currentTarget = new RelativePosition(7,95,0,-1);
        globalStore.setValue("CurrentTarget", currentTarget);

        globalStore.setValue("ReferenceAprilTagId",7);
        globalStore.setValue("YawStep",-0.25);

        setPIDCoeficients(globalStore);
        setErrorTolerances(globalStore);

        Status status = super.perform(globalStore);

        lastStatus = status;



        return status;
    }

    private void setPIDCoeficients(GlobalStore globalStore){
        PIDCoeficients pidCoeficients = new PIDCoeficients();
        pidCoeficients.HKd=0.0;
        pidCoeficients.HKi=0;
        pidCoeficients.HKp=0.012;

        pidCoeficients.RKd=0.0045;
        pidCoeficients.RKi=0.14;
        pidCoeficients.RKp=0.033;

        pidCoeficients.YKd=0;
        pidCoeficients.YKi=0;
        pidCoeficients.YKp=0.012;

        globalStore.setValue("PIDCoeficients", pidCoeficients);

    }

    private void setErrorTolerances(GlobalStore globalStore){
        ErrorTolerances errorTolerances = new ErrorTolerances();
        errorTolerances.headingErrorTolerance=2;
        errorTolerances.rangeErrorTolerance=2.5;
        errorTolerances.yawErrorTolerance=3;

        globalStore.setValue("ErrorTolerances", errorTolerances);

    }
}

