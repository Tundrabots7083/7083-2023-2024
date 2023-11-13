package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.RelativePosition;

public class ScoreBoardPixel implements ActionFunction {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;

    ElapsedTime timer = new ElapsedTime(); //used for simulating scoring time
    static double lastTime=0;
    public ScoreBoardPixel(LinearOpMode opMode){
        this.opMode = opMode;
        timer = new ElapsedTime();

    }

    public Status perform(GlobalStore globalStore) {
        globalStore.setValue("ReferenceAprilTagId",-1);

        opMode.telemetry.addData("SearchBoarPixel", "perform;---lastStatus = %d", lastStatus);
        opMode.telemetry.update();

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }

        Status status = Status.RUNNING;


        if(timer.seconds() - lastTime <= 3){
            status = Status.RUNNING;
        }
        lastTime=timer.seconds();


        lastStatus = status;

        opMode.sleep(20);

        return status;
    }
}
