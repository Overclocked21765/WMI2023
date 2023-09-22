package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkReader {
    List<LynxModule> modules;

    public void init(HardwareMap hardwareMap){
        modules = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : modules){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public void update(){
        for (LynxModule hub: modules){
            hub.clearBulkCache();
        }
    }
}
