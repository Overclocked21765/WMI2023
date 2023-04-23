package org.firstinspires.ftc.teamcode.mechanisms.beta;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Vector;

public class SwerveModule {
    DcMotorEx drive;
    Servo turn;
    public static double ticksPerDeg;
    public static double maxPower;

    public void init(HardwareMap hardwareMap, String motorName, String servoName){
        drive = hardwareMap.get(DcMotorEx.class, motorName);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turn = hardwareMap.get(Servo.class, servoName);
        turn.setPosition(0);
    }

    public void setState(Vector vector){
        drive.setPower(maxPower * vector.getR());
        turn.setPosition((vector.getTheta() - 90) * ticksPerDeg);
    }
}
