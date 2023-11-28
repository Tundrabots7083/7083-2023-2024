package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DetectDeadWheelsLocalization  implements ActionFunction {
    private LinearOpMode opMode;

    public DetectDeadWheelsLocalization(LinearOpMode opMode) {
        this.opMode = opMode;
        //initialize dead wheel subsystem
    }

    public Status perform(GlobalStore globalStore) {
        //read values from the dead wheels subsystem
        //interpret the values
        // calculate localization
        // push/save localization to globalStore
        return Status.SUCCESS;
    }
}
