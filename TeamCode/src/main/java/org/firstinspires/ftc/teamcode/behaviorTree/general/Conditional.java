package org.firstinspires.ftc.teamcode.behaviorTree.general;

import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

public class Conditional extends Node {
    private Condition condition;

    public Conditional(Condition condition) {
        this.condition = condition;
    }

    @Override
    public Status execute(GlobalStoreSingleton globalStore) {
        boolean result = condition.check();
        System.out.println("Conditional check result: " + result);
        if (result == true) {
            return Status.SUCCESS;
        } else {
            return Status.FAILURE;
        }

        // return condition.check() ? Status.SUCCESS : Status.FAILURE;
    }
}
