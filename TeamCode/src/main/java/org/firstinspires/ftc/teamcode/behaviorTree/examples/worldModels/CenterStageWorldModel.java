package org.firstinspires.ftc.teamcode.behaviorTree.examples.worldModels;

import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObject;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObjectSize;
import org.firstinspires.ftc.teamcode.models.Position;
public class CenterStageWorldModel extends WorldModel {
    public CenterStageWorldModel() {
        this.Init();
    }

    private void Init() {
        //blue alliance backdrop board
        WorldObject blueAllianceBackdrop = new WorldObject("BlueAllianceBackdrop","BlueBackdrop", new Position(60.75, 49.5), new WorldObjectSize(48, 35.0));

        this.setValue(blueAllianceBackdrop);

        //blue alliance backdrop board tags
        WorldObject blueAllianceLeft = new WorldObject("BlueAllianceLeft","1", new Position(1.0, 3.0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceCenter = new WorldObject("BlueAllianceCenter","2", new Position(2.0, 6.0), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceRight = new WorldObject("BlueAllianceRight","3", new Position(3.0, 9.0), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceLeft);
        this.setValue(blueAllianceCenter);
        this.setValue(blueAllianceRight);

        //red alliance backdrop board tags
        WorldObject redAllianceLeft = new WorldObject("RedAllianceLeft","4", new Position(1.0, 3.0), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceCenter = new WorldObject("RedAllianceCenter","5", new Position(2.0, 6.0), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceRight = new WorldObject("RedAllianceRight","6", new Position(3.0, 9.0), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceLeft);
        this.setValue(redAllianceCenter);
        this.setValue(redAllianceRight);


        //blue alliance backdrop board
        WorldObject redAllianceBackdrop = new WorldObject("RedAllianceBackdrop","RedBackdrop", new Position(60.75, -12.75), new WorldObjectSize(48, 35.0));

        this.setValue(redAllianceBackdrop);

        //red alliance wall target tags
        WorldObject redAllianceWallTargetLarge = new WorldObject("RedAllianceWallTargetLarge","7", new Position(-72.0, -29.5), new WorldObjectSize(5.0, 5.0));
        WorldObject redAllianceWallTargetSmall = new WorldObject("RedAllianceWallTargetSmall","8", new Position(-72.0, -24.0), new WorldObjectSize(2.0, 2.0));

        this.setValue(redAllianceWallTargetLarge);
        this.setValue(redAllianceWallTargetSmall);

        //blue alliance wall target tags
        WorldObject blueAllianceWallTargetLarge = new WorldObject("BlueAllianceWallTargetLarge","10", new Position(-72.0, 29.5), new WorldObjectSize(5.0, 5.0));
        WorldObject blueAllianceWallTargetSmall = new WorldObject("BlueAllianceWallTargetSmall","9", new Position(-72.0, 24.0), new WorldObjectSize(2.0, 2.0));

        this.setValue(blueAllianceWallTargetLarge);
        this.setValue(blueAllianceWallTargetSmall);


        //blue alliance audience spike positions
        WorldObject blueAllianceAudienceSpikeLeft = new WorldObject("BlueAllianceAudienceSpikeLeft","BlueAllianceAudienceSpikeLeft", new Position(-24, 30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceAudienceSpikeCenter = new WorldObject("BlueAllianceAudienceSpikeCenter","BlueAllianceAudienceSpikeCenter", new Position(-36, 24), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceAudienceSpikeRight = new WorldObject("BlueAllianceAudienceSpikeRight","BlueAllianceAudienceSpikeRight", new Position(-48, 30), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceAudienceSpikeLeft);
        this.setValue(blueAllianceAudienceSpikeCenter);
        this.setValue(blueAllianceAudienceSpikeRight);


        //blue alliance backstage spike positions
        WorldObject blueAllianceBackstageSpikeLeft = new WorldObject("BlueAllianceBackstageSpikeLeft","BlueAllianceBackstageSpikeLeft", new Position(24, 30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceBackstageSpikeCenter = new WorldObject("BlueAllianceBackstageSpikeCenter","BlueAllianceBackstageSpikeCenter", new Position(12, 24), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceBackstageSpikeRight = new WorldObject("BlueAllianceBackstageSpikeRight","BlueAllianceBackstageSpikeRight", new Position(0, 30), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceBackstageSpikeLeft);
        this.setValue(blueAllianceBackstageSpikeCenter);
        this.setValue(blueAllianceBackstageSpikeRight);


        //red alliance audience spike positions
        WorldObject redAllianceAudienceSpikeLeft = new WorldObject("RedAllianceAudienceSpikeLeft","RedAllianceAudienceSpikeLeft", new Position(-48.0, -30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceAudienceSpikeCenter = new WorldObject("RedAllianceAudienceSpikeCenter","RedAllianceAudienceSpikeCenter", new Position(-36, -24), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceAudienceSpikeRight = new WorldObject("RedAllianceAudienceSpikeRight","RedAllianceAudienceSpikeRight", new Position(-24, -30), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceAudienceSpikeLeft);
        this.setValue(redAllianceAudienceSpikeCenter);
        this.setValue(redAllianceAudienceSpikeRight);


        //red alliance backstage spike positions
        WorldObject redAllianceBackstageSpikeLeft = new WorldObject("RedAllianceBackstageSpikeLeft","RedAllianceBackstageSpikeLeft", new Position(0, -30), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceBackstageSpikeCenter = new WorldObject("RedAllianceBackstageSpikeCenter","RedAllianceBackstageSpikeCenter", new Position(12.0, -24), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceBackstageSpikeRight = new WorldObject("RedAllianceBackstageSpikeRight","RedAllianceBackstageSpikeRight", new Position(24.0, -30), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceBackstageSpikeLeft);
        this.setValue(redAllianceBackstageSpikeCenter);
        this.setValue(redAllianceBackstageSpikeRight);

    }
}
