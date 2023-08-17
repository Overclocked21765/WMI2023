package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Constants;

@Config
public class HeadingPID extends DriveTrain {
    public static double kP = 0.015;
    public static double kI = 0;
    public static double kD = 0;
    public static double margin = 1.5;
    public static double maxAllowedError = 350;
    protected double targetHeading;
    protected boolean firstTimeAfterStickRelease;
    PIDController controller;

    double maxError;

    public enum States{
        PID,
        ROTATING,
        SETTING
    }

    States state;

    public HeadingPID(double heading){
        targetHeading = heading;
        state = States.PID;
        firstTimeAfterStickRelease = false;
        controller = new PIDController(kP, kI, kD);
        maxError = 0;
    }

    public void update(double leftX, double leftY, double rightX){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double error = heading - targetHeading;
        if (error > maxAllowedError){
            error -= 360;
        } else if (error < -maxAllowedError){
            error += 360;
        }

        if (Math.abs(error) > maxError){
            maxError = Math.abs(error);
        }

        if (rightX == 0) {
            if (firstTimeAfterStickRelease){
                targetHeading = heading;
                error = heading - targetHeading;
                firstTimeAfterStickRelease = false;
            }
            if (Math.abs(error) > margin) {
                drive(controller.calculate(heading, targetHeading), leftX, leftY, heading, Constants.DRIVE_POWER_MODIFIER);
            } else {
                drive(0, leftX, leftY, heading, Constants.DRIVE_POWER_MODIFIER);
            }
            telemetry.addData("Mode: ", "PID");
        } else {
            drive(rightX, leftX, leftY, heading, Constants.DRIVE_POWER_MODIFIER);
            firstTimeAfterStickRelease = true;
            telemetry.addData("Mode: ", "Rotating");
        }


        telemetry.addData("Heading: ", heading);
        telemetry.addData("Error: ", error);
        telemetry.addData("Target: ", targetHeading);
        telemetry.addData("Max Error: ", maxError);

    }

    @Override
    public void resetYaw(){
        imu.resetYaw();
        targetHeading = 0;
    }
}
