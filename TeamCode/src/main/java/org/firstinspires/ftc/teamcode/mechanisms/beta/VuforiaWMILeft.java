package org.firstinspires.ftc.teamcode.mechanisms.beta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoCycleNewPathsRight;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;


@Autonomous
public class VuforiaWMILeft extends LinearOpMode {
    private GenericDetector rf = null;
    private String result = "";


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
    TrajectorySequence zoneThree;
    TrajectorySequence stackToJunction1;
    TrajectorySequence junctionToStack1;
    TrajectorySequence stackToJunction2;
    TrajectorySequence junctionToStack2;



    boolean wantToPark;

    public static double timeToPark = 24;
    public static int coneIndex;

    public static double startX = -35;
    public static double startY = -63;
    public static double signalY = -5;
    public static double scoreX = -28;
    public static double scoreY = -21;
    public static double tangentX = -44;
    public static double coneX = -63.5;
    public static double coneY = -11.5;

    public static double parkX = -12;
    public static double zone2X = -36;
    public static double zoneThreeX = -12;
    @Override
    public void runOpMode(){
        try {
            try {
                //initialize the bot
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
                        .lineTo(new Vector2d(startX, signalY))
                        .setReversed(true)
                        .splineTo(new Vector2d(scoreX, scoreY), Math.toRadians(-42))
                        .build();

                junctionToStack = drive.trajectorySequenceBuilder(startTrajectory.end())
                        .splineTo(new Vector2d(tangentX, coneY), Math.toRadians(180))
                        .splineTo(new Vector2d(coneX, coneY), Math.toRadians(180))
                        .build();

                stackToJunction = drive.trajectorySequenceBuilder(junctionToStack.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(tangentX, coneY), Math.toRadians(0))
                        .splineTo(new Vector2d(scoreX, scoreY), Math.toRadians(-42))
                        .build();

                junctionToStack1 = drive.trajectorySequenceBuilder(stackToJunction.end())
                        .splineTo(new Vector2d(tangentX, coneY), Math.toRadians(180))
                        .splineTo(new Vector2d(coneX - 1, coneY), Math.toRadians(180))
                        .build();

                stackToJunction1 = drive.trajectorySequenceBuilder(junctionToStack.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(tangentX, coneY), Math.toRadians(0))
                        .splineTo(new Vector2d(scoreX - 1, scoreY), Math.toRadians(-42))
                        .build();

                junctionToStack2 =  drive.trajectorySequenceBuilder(stackToJunction1.end())
                        .splineTo(new Vector2d(tangentX, coneY), Math.toRadians(180))
                        .splineTo(new Vector2d(coneX - 2, coneY), Math.toRadians(180))
                        .build();

                stackToJunction2 = drive.trajectorySequenceBuilder(junctionToStack2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(tangentX, coneY), Math.toRadians(0))
                        .splineTo(new Vector2d(scoreX - 2.5, scoreY), Math.toRadians(-42))
                        .build();



                zoneTwo = drive.trajectorySequenceBuilder(stackToJunction2.end())
                        .lineToLinearHeading(new Pose2d(zone2X, parkX, Math.toRadians(90)))
                        .build();

                zoneThree = drive.trajectorySequenceBuilder(stackToJunction2.end())
                        .lineToLinearHeading(new Pose2d(zone2X, parkX, Math.toRadians(90)))
                        .lineTo(new Vector2d(zoneThreeX, parkX))
                        .build();

                claw.grab();


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

            if (result.equals("2 PP")){
                parkingPosition = SleeveDetection.ParkingPosition.CENTER;
            } else if (result.equals("3 FTC")){
                parkingPosition = SleeveDetection.ParkingPosition.RIGHT;
            } else {
                parkingPosition = SleeveDetection.ParkingPosition.LEFT;
            }

            drive.followTrajectorySequenceAsync(startTrajectory);
            slide.setSlidePosition(Constants.MEDIUM_POSITION);
            slide.update();
            runtime.reset();
            claw.grab();


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double[] coneStackValues = {Constants.CONE_FOUR, Constants.CONE_THREE, Constants.CONE_TWO, Constants.CONE_ONE, Constants.GROUND_POSITION};
                TrajectorySequence[] junctionsToStack = {junctionToStack, junctionToStack1, junctionToStack2};
                TrajectorySequence[] stacsksToJunction = {stackToJunction, stackToJunction1, stackToJunction2};
                switch (state){
                    case HEADING_TO_JUNCTION:
                        claw.grab();
                        if (!drive.isBusy()){
                            claw.release();
                            clawTimer.reset();
                            if (runtime.time() >= timeToPark){
                                wantToPark = true;
                                if (parkingPosition == SleeveDetection.ParkingPosition.LEFT){
                                    drive.followTrajectorySequenceAsync(junctionToStack2);
                                } else if (parkingPosition == SleeveDetection.ParkingPosition.RIGHT){
                                    drive.followTrajectorySequenceAsync(zoneThree);
                                } else {
                                    drive.followTrajectorySequenceAsync(zoneTwo);
                                }
                                state = AutoCycleNewPathsRight.States.PARKING;


                            }
                            if (!wantToPark){
                                drive.followTrajectorySequenceAsync(junctionsToStack[coneIndex]);
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
                            claw.grab();
                            clawTimer.reset();
                            state = AutoCycleNewPathsRight.States.HEADING_TO_CONES_3;
                        }
                        break;
                    case HEADING_TO_CONES_3:
                        if (clawTimer.time() > Constants.TIME_FOR_RELEASE_CLAW){
                            slide.setSlidePosition(coneStackValues[coneIndex]);

                            state = AutoCycleNewPathsRight.States.HEADING_TO_CONES_4;
                        }
                        break;
                    case HEADING_TO_CONES_4:
                        if (slide.getSlidePos() < Constants.RED_ZONE){
                            claw.release();
                            state = AutoCycleNewPathsRight.States.HEADING_TO_CONES_5;
                        }
                        break;

                    case HEADING_TO_CONES_5:
                        if (!drive.isBusy()){
                            claw.grab();
                            clawTimer.reset();
                            coneIndex++;
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
                        if (slide.getSlidePos() > Constants.RED_ZONE){
                            slide.setSlidePosition(Constants.MEDIUM_POSITION);
                            slide.rotateServo();
                            state = AutoCycleNewPathsRight.States.HEADING_TO_JUNCTION;
                            drive.followTrajectorySequenceAsync(stacsksToJunction[coneIndex - 1]);
                        }
                        break;
                    case PARKING:
                        if (clawTimer.time() >  Constants.TIME_FOR_RELEASE_CLAW + 0.3){
                            claw.grab();
                            clawTimer.reset();
                            state = AutoCycleNewPathsRight.States.PARKING_2;
                        }
                    case PARKING_2:
                        if (clawTimer.time() > Constants.TIME_FOR_RELEASE_CLAW){
                            slide.setSlidePosition(Constants.GROUND_POSITION);
                            state = AutoCycleNewPathsRight.States.PARKING_3;
                        }
                    case PARKING_3:
                        if (!drive.isBusy()){
                            state = AutoCycleNewPathsRight.States.STILL_1;
                        }
                }
                drive.update();
                slide.update();
                telemetry.addData("State: ", state);
                telemetry.addData("Cone index: ", coneIndex);
                telemetry.update();
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
