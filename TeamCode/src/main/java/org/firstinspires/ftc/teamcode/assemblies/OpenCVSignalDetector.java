package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class OpenCVSignalDetector {
    HardwareMap hardwareMap;
    OpenCvWebcam webcam;
    SignalPipeline SignalPipe;
    Telemetry telemetry;

    public OpenCVSignalDetector() {
        SignalPipe = new SignalPipeline();
    }

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public void writeTelemetry() {
    }

    public void initialize() {
        teamUtil.log("Initializing OpenCVDetector");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(SignalPipe);
        teamUtil.log("Finished Initializing OpenCVDetector");
    }

    public void activate() {
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            public void onOpened(){
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                //webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError ( int errorCode){

            }
        });
        telemetry.addLine("Waiting for camera");
        telemetry.update();
        teamUtil.pause(2500);

    }

    public void deactivate() {
        webcam.stopStreaming();
    }

    public int warehouseDetect() {
        //will return value based on which level the duck should go on
        //left is 1
        //middle is 2
        //right is 3
        int leftThreshold;
        int rightThreshold;

        if(teamUtil.alliance == teamUtil.Alliance.BLUE) {

            leftThreshold = 0;
            rightThreshold = 340;
        }
        else{

            leftThreshold = 545;
            rightThreshold = 985;

        }
        int midpoint = SignalPipe.getMidpoint();
        //String midpointToPrint = String.format("%f", TSEPipe.getMidpoint());
        //log("midpoint:" + midpointToPrint);
        telemetry.addLine("Midpoint: "+midpoint);

        if(midpoint<=leftThreshold){
            return 1;
        }
        else if(midpoint > leftThreshold && midpoint < rightThreshold){
            return 2;
        }
        else{
            return 3;
        }
    }
    public int carouselDetect(){

        int leftThreshold;
        int rightThreshold;

        if(teamUtil.alliance == teamUtil.Alliance.BLUE) {
            leftThreshold= 80;
            rightThreshold = 550;

        }
        else{
            leftThreshold=110;
            rightThreshold = 610;

        }
        int midpoint = SignalPipe.getMidpoint();
        // String midpointToPrint = String.format("%f", TSEPipe.getMidpoint());
        //log("midpoint:" + midpointToPrint);
        telemetry.addLine("Midpoint: "+midpoint);

        if(midpoint<=leftThreshold){
            return 1;
        }
        else if(midpoint > leftThreshold && midpoint < rightThreshold){
            return 2;
        }
        else{
            return 3;
        }
    }

    static class SignalPipeline extends OpenCvPipeline {


        Mat HSVMat = new Mat();

        Scalar yellowLowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
        Scalar yellowHighHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
        Scalar greenLowHSV = new Scalar(40, 100, 100); //
        Scalar greenHighHSV = new Scalar(80, 255, 255); //
        Scalar pinkLowHSV = new Scalar(170, 100, 100); // 340/2
        Scalar pinkHighHSV = new Scalar(180, 255, 255); // 360/2
        Mat thresholdMat = new Mat();

        Mat edges = new Mat();

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Mat labeled = new Mat();
        Scalar labelColor = new Scalar(0, 0, 255);

        int midpoint = 0; // midpoint of detected TSE (0 if no TSE detected)

        enum Stage {
            RAW_IMAGE,
            HSV,
            THRESHOLD,
            EDGES,
            LABELED
        }

        private Stage stageToRenderToViewport = Stage.RAW_IMAGE;
        private Stage[] stages = Stage.values();

        public void nextView() {

            int currentStageNum = stageToRenderToViewport.ordinal();
            int nextStageNum = currentStageNum + 1;
            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }
            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {

            // Convert to HSV
            Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV);
            // if something is wrong, bail out
            if (HSVMat.empty()) {
                return input;
            }

            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            //Core.inRange(HSVMat, lowHSV, highHSV, thresholdMat);

            // Use Canny Edge Detection to find edges (might have to tune the thresholds for hysteresis)
            Imgproc.Canny(thresholdMat, edges, 100, 300);

            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            contours.clear(); // empty the list from last time
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            input.copyTo(labeled); // make a copy of orignal to add labels to

            // if no contours, we didn't find anything
            if (contours.isEmpty()) {
                midpoint = 0;
            } else {
                // find the bounding rectangles of those contours, compute midpoint, and prepare labeled mat
                int farLeft = 5000;
                int farRight = 0;
                MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
                Rect[] boundRect = new Rect[contours.size()];
                for (int i = 0; i < contours.size(); i++) {
                    contoursPoly[i] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                    boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                    if (boundRect[i].tl().x < farLeft) {
                        farLeft = (int) boundRect[i].tl().x;
                    }
                    if (boundRect[i].br().x > farRight) {
                        farRight = (int) boundRect[i].br().x;
                    }
                    Imgproc.rectangle(labeled, boundRect[i], labelColor, 5);

                }
                midpoint = farLeft + (farRight - farLeft) / 2;
            }


            switch (stageToRenderToViewport) {
                case RAW_IMAGE: {
                    return input;
                }
                case HSV: {
                    return HSVMat;
                }
                case THRESHOLD: {
                    return thresholdMat;
                }
                case EDGES: {
                    return edges;
                }
                case LABELED: {
                    return labeled;
                }
                default: {
                    return input;
                }
            }
        }

        public int getMidpoint() {
            return midpoint;
        }
    }
}

