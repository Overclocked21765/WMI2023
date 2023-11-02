package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoopTimer extends ElapsedTime {

    double maxLoop, minLoop;
    Telemetry telemetry;

    public LoopTimer(Telemetry telemetry){
        super();
        this.telemetry = telemetry;
    }

    public void updateLoop(){
        double currentLoop = this.time();
        this.reset();
        maxLoop = Math.max(currentLoop, maxLoop);
        minLoop = (minLoop == 0) ? currentLoop : Math.min(currentLoop, minLoop);
        telemetry.addData("Current loop: ", currentLoop);
        telemetry.addData("Max loop: ", maxLoop);
        telemetry.addData("Min loop: ", minLoop);
    }

    public void endLoop(){
        telemetry.addData("Max loop: ", maxLoop);
        telemetry.addData("Min loop: ", minLoop);
    }
}
