package org.firstinspires.ftc.teamcode.behaviorTree.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStore;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.RelativePosition;
import org.firstinspires.ftc.teamcode.subsystems.SpikeElementDetector;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class DetectSpikeElement  implements ActionFunction {
    private LinearOpMode opMode;
private SpikeElementDetector spikeElementDetector;
    private AprilTagProcessor spikeElementProcessor;  //TODO change type to get tensor flow processor

    public DetectSpikeElement(LinearOpMode opMode) {
        opMode.telemetry.addData("DetectSpikeElement", "DetectSpikeElement started");
        opMode.telemetry.update();
        this.opMode=opMode;
        this.spikeElementDetector = new SpikeElementDetector(this.opMode);


    }

    public Status perform(GlobalStore globalStore) {
        opMode.telemetry.addData("DetectSpikeElement "," perform");
        opMode.telemetry.update();

        RelativePosition spikePosition = new RelativePosition(7,30,0,0);
        //fake center position
        globalStore.setValue("CurentSpikeElementPosition", spikePosition);
        return Status.SUCCESS;
/*
        List<AprilTagDetection> currentDetections = spikeElementProcessor.getDetections();
        if (currentDetections != null) {
            opMode.telemetry.addData("DetectSpikeElement ","Found %d targets", currentDetections.stream().count());
            opMode.telemetry.update();
            globalStore.setValue("DetectSpikeElement", currentDetections);
            return true;
        } else {
            opMode.telemetry.addData("DetectSpikeElement "," no tag found");
            opMode.telemetry.update();
            globalStore.removeValue("CurrentDetections");
            return false;
        }

        */
    }
}
