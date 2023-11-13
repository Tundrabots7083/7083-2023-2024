package org.firstinspires.ftc.teamcode.behaviorTree.general;


import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

public class BehaviorTree {
    private Node root;
    private GlobalStore globalStore;

    public BehaviorTree(Node root, GlobalStore globalStore) {
        this.root = root;
        this.globalStore = globalStore;
    }

    public Status run() {
        return root.execute(globalStore);
    }
}
