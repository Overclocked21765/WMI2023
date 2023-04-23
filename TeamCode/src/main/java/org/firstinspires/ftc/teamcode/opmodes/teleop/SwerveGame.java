package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.beta.SwerveBase;

@TeleOp(name = "swerve game")
public class SwerveGame extends OpMode {
    SwerveBase drive = new SwerveBase();
    boolean yAlreadyPressed;

    @Override
    public void init(){
        yAlreadyPressed = false;
        drive.init(hardwareMap);
    }

    @Override
    public void loop(){
        drive.drive(gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x);

        if (gamepad1.y && !yAlreadyPressed){
            drive.resetHeading();
        }
        yAlreadyPressed = gamepad1.y;
    }
}
