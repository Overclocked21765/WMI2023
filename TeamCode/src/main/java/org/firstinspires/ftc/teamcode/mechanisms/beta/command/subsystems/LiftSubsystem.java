package org.firstinspires.ftc.teamcode.mechanisms.beta.command.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDController;

public class LiftSubsystem extends SubsystemBase {
    DcMotorEx liftMotor;
    PIDController controller;
    public double target;


    public static double kP, kI, kD, kF;

    public LiftSubsystem(HardwareMap hardwareMap, String name) {
        liftMotor = hardwareMap.get(DcMotorEx.class, name);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller = new PIDController(kP, kI, kD);
    }

    public void setTarget(double target){
        this.target = target;
    }




    @Override
    public void periodic(){
        controller.setPID(kP, kI, kD);
        double pid = controller.calculate(liftMotor.getCurrentPosition(), this.target);
        liftMotor.setPower(kF + pid);
    }
}
