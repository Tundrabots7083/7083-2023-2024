package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;


/*---------------------------------------------------------
This is a place holder class for scoring the spike pixel
------------------------------------------------------- */

public class ScoreSpikePixel implements ActionFunction {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;

    ElapsedTime timer = new ElapsedTime(); //used for simulating scoring time
    double lastTime=0;
    public ScoreSpikePixel(LinearOpMode opMode){
        this.opMode = opMode;
        timer = new ElapsedTime();

    }

    public Status perform(GlobalStoreSingleton globalStore) {
        globalStore.setValue("ReferenceAprilTagId",-1);

        opMode.telemetry.addData("SearchSpikePixel", "perform;---lastStatus = %d", lastStatus);
        opMode.telemetry.update();

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        Status status = Status.RUNNING;


        if(timer.seconds() - lastTime <= 3){
            status = Status.RUNNING;
        } else {
            status = Status.SUCCESS;
        }
        lastTime=timer.seconds();


        lastStatus = status;

       // opMode.sleep(20);

        return status;
    }
}
