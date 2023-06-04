package org.firstinspires.ftc.teamcode.mechanisms.beta;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;

import java.util.List;

public class VuforiaCamera{
    private static final String TFOD_MODEL_ASSET = "wmi.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "LEFT",
            "CENTER",
            "RIGHT"
    };

    private static final String VUFORIA_KEY = "temp";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    Telemetry telemetry;

    String lastPosition;
    SleeveDetection.ParkingPosition trueLastPosition;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        if (tfod != null){
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        lastPosition = "CENTER";
        trueLastPosition = SleeveDetection.ParkingPosition.CENTER;
    }

    public SleeveDetection.ParkingPosition returnZoneEnumerated(){
        String name = lastPosition;
        double max = 0;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number

                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;



                    telemetry.addData(""," ");
                    double confidence = recognition.getConfidence();
                    String label = recognition.getLabel();
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", label, confidence * 100 );
                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                    if (confidence > max){
                        max = confidence;
                        name = label;
                    }



                }
                if (name.equals(LABELS[0])){
                    lastPosition = LABELS[0];
                    trueLastPosition = SleeveDetection.ParkingPosition.LEFT;
                    return trueLastPosition;
                } else if (name.equals(LABELS[1])){
                    lastPosition = LABELS[1];
                    trueLastPosition = SleeveDetection.ParkingPosition.CENTER;
                    return trueLastPosition;
                } else if (name.equals(LABELS[2])){
                    lastPosition = LABELS[2];
                    trueLastPosition = SleeveDetection.ParkingPosition.RIGHT;
                    return trueLastPosition;
                }
            }
        }
        return trueLastPosition;

    }

    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
