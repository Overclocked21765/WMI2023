package org.firstinspires.ftc.teamcode.mechanisms.beta.command.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public DriveSubsystem(HardwareMap hardwareMap, String name){
        frontLeft = hardwareMap.get(DcMotorEx.class, name);
        frontRight = hardwareMap.get(DcMotorEx.class, name);
        backLeft = hardwareMap.get(DcMotorEx.class, name);
        backRight = hardwareMap.get(DcMotorEx.class, name);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
