package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.beta.VuforiaCamera;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;

@TeleOp(group = "Hardware tests")
public class TensorFlowDetection extends OpMode {
    VuforiaCamera vuforiaCamera = new VuforiaCamera();
    SleeveDetection.ParkingPosition parkingPosition;
    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        vuforiaCamera.init(hardwareMap, telemetry);

    }

    @Override
    public void init_loop(){
        parkingPosition = vuforiaCamera.returnZoneEnumerated();
        telemetry.addData("Zone: ", parkingPosition);
    }

    @Override
    public void loop(){
        parkingPosition = vuforiaCamera.returnZoneEnumerated();
        telemetry.addData("Zone: ", parkingPosition);
    }
}
