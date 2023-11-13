package org.firstinspires.ftc.teamcode.behaviorTree.general;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

public class Action extends Node {
    private ActionFunction actionFunction;
    protected LinearOpMode opMode;


    public Action(ActionFunction actionFunction, LinearOpMode opMode) {
        this.actionFunction = actionFunction;
        this.opMode = opMode;
    }

    @Override
    public Status execute(GlobalStore globalStore) {

        Status status = actionFunction.perform(globalStore);



         return status;
    }
}
