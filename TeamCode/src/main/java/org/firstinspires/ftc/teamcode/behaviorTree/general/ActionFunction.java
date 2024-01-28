package org.firstinspires.ftc.teamcode.behaviorTree.general;

import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

public interface ActionFunction {
    public Status perform(GlobalStoreSingleton globalStore);
}