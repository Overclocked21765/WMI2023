package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Slide;
import org.firstinspires.ftc.teamcode.util.Constants;

@Config
@TeleOp
public class PIDFTest extends OpMode {
    Slide slide = new Slide();

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide.init(hardwareMap, telemetry);

    }

    @Override
    public void loop(){
        slide.setSlidePosition(Slide.test_target);
        slide.update();
        telemetry.addData("pos:", slide.getSlidePos());
        telemetry.addData("target", Slide.test_target);
    }

}
