package org.firstinspires.ftc.teamcode.behaviorTree.general;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

public class Sequence extends Node {
    private List<Node> children;
    protected LinearOpMode opMode;

    public Sequence(List<Node> children, LinearOpMode opMode) {
        this.children = children;
        this.opMode = opMode;
    }

    @Override
    public Status execute(GlobalStore globalStore) {
        for (Node child : children) {
            Status status = child.execute(globalStore);
          //  opMode.telemetry.addData("Sequence", "Sequence execute result: %b num children = %d",status==Status.SUCCESS, children.stream().count());
          //  opMode.telemetry.update();
            if (status == Status.FAILURE) {
                return Status.FAILURE;
            } else if (status == Status.RUNNING) {
                return Status.RUNNING;
            }
        }
       // opMode.telemetry.addData("Sequence", "Sequence execute ---------------");
       // opMode.telemetry.update();
        return Status.SUCCESS;
    }
}
