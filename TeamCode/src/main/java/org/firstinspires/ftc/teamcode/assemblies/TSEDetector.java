package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.List;

public class TSEDetector {

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            "Aek7e0f/////AAABmQVqFuJnNEFosHXJFebMMqiNWhdbJ0N0rL1VvGfQVwrRgnvjoNhY8h5V5vgtui/tirazPIaHKxq5qvd5FVFA9HygGZF4G5rDWkOg7r0FdpKlFr2lDArT2XPAcBCnTCVdqkbY6ri0/xsLoWpHpQY6pYrRtDpOxLdso0xmPdGPzO3XHkEvcLuYhb/QJyAv/Svx7oSuxfcE+mv5LHQcdsdC1C3WEhWLcF8QrmQFzKR4yQRjl2ieRaQkYyLr/ATIBvkIZdK5aYfGhW4aCRZWoBg5xoifJITuO68EK9up2mDb2ETM+BuXS4d0RqWKpuVI7u6f0RiOrrlmUHtQhhuHPxkjdPG6812jKMnH+qDrBlLTJlUV";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public TSEDetector(){
        teamUtil.log("Constructing Duck Detector");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }
    public void writeTelemetry(){
    }
    public void initialize() {
        teamUtil.log("Initializing TSEDetector");

        initVuforia();
        initTfod();


    }

    public void activate(){
        if (tfod != null) {
            tfod.activate();

            //tfod.setZoom(1, 16.0/9.0);
        }
    }
    public void deactivate(){
        if (tfod != null) {
            tfod.shutdown();

        }
    }

    public int detect() {
        //will return value based on which level the duck should go on
        //left is 1
        //middle is 2
        //right is 3
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions == null) {
            return 0;
        }
        else{
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel() == LABELS[2]) {
                    if (recognition.getLeft() > 150) {
                        return 3;
                    } else {
                        return 2;
                    }
                }
            }
            return 1;
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }


}
