package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.beta.GenericDetector;

@TeleOp(group = "Hardware tests")
public class MHSDetectorOp extends LinearOpMode {
    private GenericDetector rf = null;
    private String result = "";


    @Override
    public void runOpMode(){
        try {
            try {
                //initialize the bot

                //initialize the detector. It will run on its own thread continuously
                rf = new GenericDetector(this.hardwareMap,  this,  telemetry);
                Thread detectThread = new Thread(rf);
                detectThread.start();
                telemetry.update();
            } catch (Exception ex) {
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                telemetry.update();
                sleep(5000);
                return;
            }

            // Wait for the game to start (driver presses PLAY)
            telemetry.update();
            waitForStart();

            rf.stopDetection();

            result = rf.getResult();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addLine(result);
            }
        } catch (Exception ex) {
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        } finally {
            if (rf != null) {
                rf.stopDetection();
            }
        }

    }
}
