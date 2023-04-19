package org.firstinspires.ftc.teamcode.opmodes.auto.beta;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "WMI")
public class WMI extends OpMode {
    SampleMecanumDrive drive;
    Claw claw = new Claw();
    Slide slide = new Slide();
    Camera camera = new Camera();

    

    @Override
    public void init(){
        drive = new SampleMecanumDrive(hardwareMap);
        claw.init(hardwareMap);
        slide.init(hardwareMap);
        camera.init(hardwareMap);


    }

    @Override
    public void loop(){

    }
}
