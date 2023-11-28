package org.firstinspires.ftc.teamcode.behaviorTree.examples.behaviorTrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.CalculateLocation;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.DetectAprilTags;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.DetectDeadWheelsLocalization;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.SearchTarget;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance.NavigateRA1;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance.NavigateRA2;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance.NavigateRA4;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance.NavigateRA5;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions.redAlliance.NavigateRA6;
import org.firstinspires.ftc.teamcode.behaviorTree.examples.worldModels.CenterStageWorldModel;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Action;
import org.firstinspires.ftc.teamcode.behaviorTree.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Node;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Selector;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Sequence;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.DriveTrainConfig;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

import java.util.Arrays;

public class RedAudienceAutonomousWithDeadWheels
{
    private BehaviorTree tree;
    private Node root;
    private GlobalStore globalStore;
    private WorldModel worldModel;
    protected LinearOpMode opMode;

    public RedAudienceAutonomousWithDeadWheels(LinearOpMode opMode) {
        this.opMode = opMode;

        opMode.telemetry.addData("RedAudienceAutonomous", "RedAudienceAutonomous started");
        opMode.telemetry.update();
        Init();
    }

    private void Init() {
        this.worldModel = new CenterStageWorldModel();

        this.globalStore = new GlobalStore();
        this.initializeGlobalStore();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new DetectDeadWheelsLocalization(this.opMode),this.opMode),
                        new Action(new DetectAprilTags(this.opMode),this.opMode),
                        new Action(new CalculateLocation(this.opMode),this.opMode),


                        new Action(new NavigateRA1(this.opMode),this.opMode),
                        //new Action(new ScoreSpikePixel(this.opMode),this.opMode),
                        new Action(new NavigateRA2(this.opMode),this.opMode),
                        //  new Action(new NavigateRA3(this.opMode),this.opMode),/**/
                        new Action(new NavigateRA4(this.opMode),this.opMode),
                        new Action(new NavigateRA5(this.opMode),this.opMode),
                        //   new Action(new ScoreBoardPixel(this.opMode),this.opMode),
                        new Action(new NavigateRA6(this.opMode),this.opMode)/**/

                ),this.opMode
        );

        this.tree = new BehaviorTree(root, globalStore);
    }
    private void initializeGlobalStore(){
        DriveTrainConfig driveTrainConfig = new DriveTrainConfig();
        driveTrainConfig.speedGain =0.033;
        driveTrainConfig.strafeGain=0.02;
        driveTrainConfig.turnGain=0.04;

        this.globalStore.setValue("DriveTrainConfig", driveTrainConfig);
        this.globalStore.setValue("YawStep", 0.25);
        this.globalStore.setValue("WorldModel", this.worldModel);

        this.globalStore.setValue("ReferenceAprilTagId",7);
        this.globalStore.setValue("YawStep",-0.25);
    }
    public Status run() {
        // Run the behavior tree
        Status result = tree.run();
        opMode.telemetry.addData("RedAudienceAutonomous", "Run - Behavior tree result: %s",result);
        opMode.telemetry.update();

        return result;
    }
}
