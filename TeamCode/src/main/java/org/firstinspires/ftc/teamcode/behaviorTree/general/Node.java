package org.firstinspires.ftc.teamcode.behaviorTree.general;

import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

public abstract class Node {
    public abstract Status execute(GlobalStoreSingleton globalStore);
}

