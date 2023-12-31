package org.firstinspires.ftc.teamcode.behaviorTree.examples.worldModels;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObject;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObjectSize;
public class CenterStageWorldModel extends WorldModel {
    public CenterStageWorldModel() {
        this.Init();
    }

    private void Init() {
        //blue alliance backdrop board
        WorldObject blueAllianceBackdrop = new WorldObject("BlueAllianceBackdrop","BlueBackdrop", new Pose2d(60.75, 49.5), new WorldObjectSize(48, 35.0));

        this.setValue(blueAllianceBackdrop);

        //blue alliance backdrop board tags
        WorldObject blueAllianceLeft = new WorldObject("BlueAllianceLeft","1", new Pose2d(62, 42, 180), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceCenter = new WorldObject("BlueAllianceCenter","2", new Pose2d(62, 36, 180), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceRight = new WorldObject("BlueAllianceRight","3", new Pose2d(62, 30, 180), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceLeft);
        this.setValue(blueAllianceCenter);
        this.setValue(blueAllianceRight);

        //red alliance backdrop board tags
        WorldObject redAllianceLeft = new WorldObject("RedAllianceLeft","4", new Pose2d(62, -30.0,180), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceCenter = new WorldObject("RedAllianceCenter","5", new Pose2d(62, -36.0, 180), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceRight = new WorldObject("RedAllianceRight","6", new Pose2d(62, -42.0, 180), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceLeft);
        this.setValue(redAllianceCenter);
        this.setValue(redAllianceRight);


        //blue alliance backdrop board
        WorldObject redAllianceBackdrop = new WorldObject("RedAllianceBackdrop","RedBackdrop", new Pose2d(60.75, -12.75), new WorldObjectSize(48, 35.0));

        this.setValue(redAllianceBackdrop);

        //red alliance wall target tags
        WorldObject redAllianceWallTargetLarge = new WorldObject("RedAllianceWallTargetLarge","7", new Pose2d(-72.0, -41.5, 0), new WorldObjectSize(5.0, 5.0));
        WorldObject redAllianceWallTargetSmall = new WorldObject("RedAllianceWallTargetSmall","8", new Pose2d(-72.0, -36.0, 0), new WorldObjectSize(2.0, 2.0));

        this.setValue(redAllianceWallTargetLarge);
        this.setValue(redAllianceWallTargetSmall);

        //blue alliance wall target tags
        WorldObject blueAllianceWallTargetLarge = new WorldObject("BlueAllianceWallTargetLarge","10", new Pose2d(-72.0, 41.5, 0), new WorldObjectSize(5.0, 5.0));
        WorldObject blueAllianceWallTargetSmall = new WorldObject("BlueAllianceWallTargetSmall","9", new Pose2d(-72.0, 36, 0), new WorldObjectSize(2.0, 2.0));

        this.setValue(blueAllianceWallTargetLarge);
        this.setValue(blueAllianceWallTargetSmall);


        //blue alliance audience spike positions
        WorldObject blueAllianceAudienceSpikeLeft = new WorldObject("BlueAllianceAudienceSpikeLeft","BlueAllianceAudienceSpikeLeft", new Pose2d(-24, 30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceAudienceSpikeCenter = new WorldObject("BlueAllianceAudienceSpikeCenter","BlueAllianceAudienceSpikeCenter", new Pose2d(-36, 24), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceAudienceSpikeRight = new WorldObject("BlueAllianceAudienceSpikeRight","BlueAllianceAudienceSpikeRight", new Pose2d(-48, 30), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceAudienceSpikeLeft);
        this.setValue(blueAllianceAudienceSpikeCenter);
        this.setValue(blueAllianceAudienceSpikeRight);


        //blue alliance backstage spike positions
        WorldObject blueAllianceBackstageSpikeLeft = new WorldObject("BlueAllianceBackstageSpikeLeft","BlueAllianceBackstageSpikeLeft", new Pose2d(24, 30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceBackstageSpikeCenter = new WorldObject("BlueAllianceBackstageSpikeCenter","BlueAllianceBackstageSpikeCenter", new Pose2d(12, 24), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceBackstageSpikeRight = new WorldObject("BlueAllianceBackstageSpikeRight","BlueAllianceBackstageSpikeRight", new Pose2d(0, 30), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceBackstageSpikeLeft);
        this.setValue(blueAllianceBackstageSpikeCenter);
        this.setValue(blueAllianceBackstageSpikeRight);


        //red alliance audience spike positions
        WorldObject redAllianceAudienceSpikeLeft = new WorldObject("RedAllianceAudienceSpikeLeft","RedAllianceAudienceSpikeLeft", new Pose2d(-48.0, -30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceAudienceSpikeCenter = new WorldObject("RedAllianceAudienceSpikeCenter","RedAllianceAudienceSpikeCenter", new Pose2d(-36, -24), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceAudienceSpikeRight = new WorldObject("RedAllianceAudienceSpikeRight","RedAllianceAudienceSpikeRight", new Pose2d(-24, -30), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceAudienceSpikeLeft);
        this.setValue(redAllianceAudienceSpikeCenter);
        this.setValue(redAllianceAudienceSpikeRight);


        //red alliance backstage spike positions
        WorldObject redAllianceBackstageSpikeLeft = new WorldObject("RedAllianceBackstageSpikeLeft","RedAllianceBackstageSpikeLeft", new Pose2d(0, -30), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceBackstageSpikeCenter = new WorldObject("RedAllianceBackstageSpikeCenter","RedAllianceBackstageSpikeCenter", new Pose2d(12.0, -24), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceBackstageSpikeRight = new WorldObject("RedAllianceBackstageSpikeRight","RedAllianceBackstageSpikeRight", new Pose2d(24.0, -30), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceBackstageSpikeLeft);
        this.setValue(redAllianceBackstageSpikeCenter);
        this.setValue(redAllianceBackstageSpikeRight);

    }
}
