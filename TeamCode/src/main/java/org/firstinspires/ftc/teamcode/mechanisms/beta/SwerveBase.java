package org.firstinspires.ftc.teamcode.mechanisms.beta;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Vector;

import static org.firstinspires.ftc.teamcode.util.Algorithms.mapJoystick;
import static org.firstinspires.ftc.teamcode.util.Algorithms.returnSwerve;

public class SwerveBase {
    SwerveModule frontLeft = new SwerveModule();
    SwerveModule frontRight = new SwerveModule();
    SwerveModule backLeft = new SwerveModule();
    SwerveModule backRight = new SwerveModule();
    IMU imu;

    public void init(HardwareMap hardwareMap){
        frontRight.init(hardwareMap, "FLDrive", "FLTurn");
        frontRight.init(hardwareMap, "FRDrive", "FRTurn");
        backLeft.init(hardwareMap, "BLDrive", "BLTurn");
        backRight.init(hardwareMap, "BRDrive", "BRTurn");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }

    public void setAllFourStates(Vector[] vectors){
        frontLeft.setState(vectors[0]);
        frontRight.setState(vectors[1]);
        backLeft.setState(vectors[2]);
        backRight.setState(vectors[3]);
    }

    public void drive(double x, double y, double rotation){
        double rescaleX = mapJoystick(x, y)[0];
        double rescaleY = mapJoystick(x, y)[1];

        setAllFourStates(returnSwerve(new Vector(new Vector.VectorRectangular(rescaleX, rescaleY)), rotation, getHeading()));


    }

    public double getHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading(){
        imu.resetYaw();
    }


}
