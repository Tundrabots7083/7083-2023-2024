package org.firstinspires.ftc.teamcode.behaviorTree.general;


import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

public class BehaviorTree {
    private Node root;
    private GlobalStoreSingleton globalStore;

    public BehaviorTree(Node root, GlobalStoreSingleton globalStore) {
        this.root = root;
        this.globalStore = globalStore;
    }

    public Status run() {
        return root.execute(globalStore);
    }
}
