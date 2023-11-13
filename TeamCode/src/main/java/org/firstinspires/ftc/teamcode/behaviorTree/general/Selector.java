package org.firstinspires.ftc.teamcode.behaviorTree.general;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

public class Selector extends Node {
    private List<Node> children;
    protected LinearOpMode opMode;

    public Selector(List<Node> children, LinearOpMode opMode) {
        this.children = children;
        this.opMode = opMode;

    }

    @Override
    public Status execute(GlobalStore globalStore) {
        for (Node child : children) {
            Status status = child.execute(globalStore);
            opMode.telemetry.addData("Selector", "Selector status: %b",status == Status.SUCCESS);
            opMode.telemetry.update();
            if (status == Status.SUCCESS) {
                return Status.SUCCESS;
            } else if (status == Status.RUNNING) {
                return Status.RUNNING;
            }
        }
        return Status.FAILURE;
    }
}
