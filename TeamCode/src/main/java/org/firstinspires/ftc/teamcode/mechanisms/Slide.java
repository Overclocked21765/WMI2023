package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;

//if motor direction is FORWARD:
//negative power is extend, positive is retract

@Config
public class Slide {
    private DcMotorEx linearSlideMotor;
    private Servo slideServo;
    private double ticksPerRevolution;
    private boolean hasBeenToldToRotate;
    private double target;
    private double slidePos;

    public static double kF = 0.21;
    public static double kP = 0.05;
    public static double kI = 0;
    public static double kD = 0.23;

    public static int test_target = 0;

    private PIDController controller;

    Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry){
        linearSlideMotor = hwMap.get(DcMotorEx.class, "Slide_Motor");
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideServo = hwMap.get(Servo.class, "Slide_Servo");
        slideServo.setDirection(Servo.Direction.FORWARD);

        controller = new PIDController(kP, kI, kD, 0);

        this.telemetry = telemetry;

        ticksPerRevolution = linearSlideMotor.getMotorType().getTicksPerRev();
        hasBeenToldToRotate = true;
        //TEMP 2/1
        //This will set the slide servo to sit at the back of the robot (where we put the claw when picking up from ground)
        slideServo.setPosition(Constants.SLIDE_SERVO_ZERO_POSITION);
    }

    public void init(HardwareMap hwMap){
        linearSlideMotor = hwMap.get(DcMotorEx.class, "Slide_Motor");
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideServo = hwMap.get(Servo.class, "Slide_Servo");
        slideServo.setDirection(Servo.Direction.FORWARD);

        controller = new PIDController(kP, kI, kD);

        ticksPerRevolution = linearSlideMotor.getMotorType().getTicksPerRev();
        hasBeenToldToRotate = true;
        //TEMP 2/1
        //This will set the slide servo to sit at the back of the robot (where we put the claw when picking up from ground)
        slideServo.setPosition(Constants.SLIDE_SERVO_ZERO_POSITION);
    }

    /*
    public void setSlidePosition(int encoderPosition){
        linearSlideMotor.setTargetPosition(encoderPosition);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(Constants.MOTOR_SLIDE_POWER);
    }

     */

    public void moveSlide(double speed){
        if (linearSlideMotor.getCurrentPosition() >= (Constants.LINEAR_SLIDE_MINIMUM - Constants.LINEAR_SLIDE_MARGIN_ERROR) && linearSlideMotor.getCurrentPosition() <= (Constants.LINEAR_SLIDE_MAXIMUM - Constants.LINEAR_SLIDE_MARGIN_ERROR)){
            linearSlideMotor.setPower(speed * Constants.MOTOR_SLIDE_POWER);
        }
    }

    public void setLinearSlideMotorRunMode(){
        linearSlideMotor.setPower(0);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLinearSlideMotorPositionMode(){
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(Constants.MOTOR_SLIDE_POWER);
    }

    public int getSlidePosition(){
        return (int) slidePos;
    }

    public void rotateServo(){
        if (Math.abs(slideServo.getPosition() - Constants.SLIDE_SERVO_ZERO_POSITION) < 0.02){
            slideServo.setPosition(Constants.SLIDE_SERVO_ROTATED_POSITION);
        } else {
            slideServo.setPosition(Constants.SLIDE_SERVO_ZERO_POSITION);
        }
    }

    public void moveSlideNoLimitations(double power){
        linearSlideMotor.setPower(power * Constants.MOTOR_SLIDE_POWER);
    }

    public void setSlidePosition(double target){
        this.target = target;
    }

    public void update(){
        controller.setPID(kP, kI, kD);
        slidePos = linearSlideMotor.getCurrentPosition();
        double power = controller.calculate(slidePos, target);
        linearSlideMotor.setPower(kF + power);
        telemetry.addData("Target: ", target);
        telemetry.addData("Current position: ", linearSlideMotor.getCurrentPosition());
    }

    public boolean atTarget(){
        if (Math.abs(linearSlideMotor.getCurrentPosition() - target) < 10){
            return true;
        }
        return false;
    }

    public int getTargetPos(){
        return (int) target;
    }
    public int getSlidePos(){
        return (int) slidePos;
    }

    public double getServoPosition(){
        return slideServo.getPosition();
    }

    public void stopAndReset(){
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void pidReset(double currentState){
        controller = new PIDController(kP, kI, kD, 0, currentState);
        setSlidePosition(currentState);
    }

    public int i2cCall(){
        slidePos = linearSlideMotor.getCurrentPosition();
        return (int) slidePos;
    }
}
