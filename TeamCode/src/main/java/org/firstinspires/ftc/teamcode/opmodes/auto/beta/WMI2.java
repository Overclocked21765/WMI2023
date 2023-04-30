package org.firstinspires.ftc.teamcode.opmodes.auto.beta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoCycleNewPathsRight;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;

@Config
@Autonomous(name = "WMI Right")
public class WMI2 extends OpMode {

    AutoCycleNewPathsRight.States state;

    SampleMecanumDrive drive;
    Claw claw = new Claw();
    Slide slide = new Slide();
    Camera camera = new Camera();

    ElapsedTime clawTimer = new ElapsedTime();
    ElapsedTime slideTImer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    SleeveDetection.ParkingPosition parkingPosition;

    TrajectorySequence startTrajectory;
    TrajectorySequence junctionToStack;
    TrajectorySequence stackToJunction;
    TrajectorySequence zoneTwo;
    TrajectorySequence zoneOne;


    boolean wantToPark;

    public static double timeToPark = 24;
    public static int coneIndex;

    public static double startX = 36;
    public static double startY = -60;
    public static double signalX = -5;
    public static double scoreX = 28;
    public static double scoreY = -18;
    public static double tangentX = 44;
    public static double coneX = -60;
    public static double coneY = -12;

    public static double parkX = -12;
    public static double zone2X = 36;
    public static double zoneOneX = 12;

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        coneIndex = 0;
        wantToPark = false;
        state = AutoCycleNewPathsRight.States.HEADING_TO_JUNCTION;

        drive = new SampleMecanumDrive(hardwareMap);
        claw.init(hardwareMap);
        slide.init(hardwareMap, telemetry);
        camera.init(hardwareMap);

        drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(90)));

        startTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(startX, signalX))
                .setReversed(true)
                .splineTo(new Vector2d(scoreX, scoreY), Math.toRadians(222))
                .build();

        junctionToStack = drive.trajectorySequenceBuilder(startTrajectory.end())
                .splineTo(new Vector2d(tangentX, coneY), Math.toRadians(180))
                .splineTo(new Vector2d(coneX, coneY), Math.toRadians(180))
                .build();

        stackToJunction = drive.trajectorySequenceBuilder(junctionToStack.end())
                .setReversed(true)
                .splineTo(new Vector2d(tangentX, coneY), Math.toRadians(0))
                .splineTo(new Vector2d(scoreX, scoreY), Math.toRadians(222))
                .build();

        zoneTwo = drive.trajectorySequenceBuilder(stackToJunction.end())
                .lineToLinearHeading(new Pose2d(zone2X, parkX, Math.toRadians(90)))
                .build();

        zoneOne = drive.trajectorySequenceBuilder(stackToJunction.end())
                .lineToLinearHeading(new Pose2d(zone2X, parkX, Math.toRadians(90)))
                .lineTo(new Vector2d(zoneOneX, parkX))
                .build();


    }

    @Override
    public void init_loop(){
        parkingPosition = camera.returnZoneEnumerated();
        claw.grab();
        telemetry.addData("Zone", parkingPosition);
    }

    @Override
    public void start(){
        drive.followTrajectorySequenceAsync(startTrajectory);
        slide.setSlidePosition(Constants.MEDIUM_POSITION);
        slide.update();
        runtime.reset();
    }

    @Override
    public void loop(){
        double[] coneStackValues = {Constants.CONE_FOUR, Constants.CONE_THREE, Constants.CONE_TWO, Constants.CONE_ONE, Constants.GROUND_POSITION};
        switch (state){
            case HEADING_TO_JUNCTION:
                claw.grab();
                if (!drive.isBusy()){
                    claw.release();
                    clawTimer.reset();
                    if (runtime.time() >= timeToPark){
                        wantToPark = true;
                        if (parkingPosition == SleeveDetection.ParkingPosition.RIGHT){
                            drive.followTrajectorySequenceAsync(junctionToStack);
                        } else if (parkingPosition == SleeveDetection.ParkingPosition.LEFT){
                            drive.followTrajectorySequenceAsync(zoneOne);
                        } else {
                            drive.followTrajectorySequenceAsync(zoneTwo);
                        }
                        state = AutoCycleNewPathsRight.States.PARKING;
                        slide.setSlidePosition(Constants.GROUND_POSITION);

                    }
                    if (!wantToPark){
                        drive.followTrajectorySequenceAsync(junctionToStack);
                        state = AutoCycleNewPathsRight.States.HEADING_TO_CONES_1;
                    }

                }
                break;
            case HEADING_TO_CONES_1:
                if (clawTimer.time() > Constants.TIME_FOR_RELEASE_CLAW){
                    slide.rotateServo();
                    slideTImer.reset();
                    state = AutoCycleNewPathsRight.States.HEADING_TO_CONES_2;
                }
                break;
            case HEADING_TO_CONES_2:
                if (slideTImer.time() > Constants.SERVO_ROTATE_TIME){
                    slide.setSlidePosition(coneStackValues[coneIndex]);
                    coneIndex++;
                    state = AutoCycleNewPathsRight.States.HEADING_TO_CONES_3;
                }
                break;
            case HEADING_TO_CONES_3:
                if (!drive.isBusy()){
                    claw.grab();
                    clawTimer.reset();
                    state = AutoCycleNewPathsRight.States.GRABBING_1;
                }
                break;
            case GRABBING_1:
                if (clawTimer.time() > Constants.TIME_FOR_RELEASE_CLAW){
                    slide.setSlidePosition(Constants.RED_ZONE);
                    state = AutoCycleNewPathsRight.States.GRABBING_2;
                }
                break;
            case GRABBING_2:
                if (slide.atTarget()){
                    slide.setSlidePosition(Constants.MEDIUM_POSITION);
                    slide.rotateServo();
                    state = AutoCycleNewPathsRight.States.HEADING_TO_JUNCTION;
                    drive.followTrajectorySequenceAsync(stackToJunction);
                }
                break;
            case PARKING:
                if (!drive.isBusy()){
                    state = AutoCycleNewPathsRight.States.STILL_1;
                }
        }
        drive.update();
        slide.update();
        telemetry.addData("State: ", state);
        telemetry.addData("Cone index: ", coneIndex);
    }
}
