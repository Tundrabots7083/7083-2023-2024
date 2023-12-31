package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;

public class LocalizeByOdometry  implements ActionFunction {

    private LinearOpMode opMode;
    public LocalizeByOdometry(LinearOpMode opMode) {
        this.opMode = opMode;
        //initialize mech drive
        //initialize localizer
        //set localizer to mech drive
    }

    public Status perform(GlobalStore globalStore) {
        return Status.SUCCESS;
    }
}
