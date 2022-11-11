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
    static int MIDHEIGHT;

    public OpenCVSignalDetector() {
        SignalPipe = new SignalPipeline();
    }

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public void initialize(boolean withMonitor) {
        teamUtil.log("Initializing OpenCVDetector");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
        if (withMonitor) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        } else {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        webcam.setPipeline(SignalPipe);
        teamUtil.log("Finished Initializing OpenCVDetector");
    }

    public void activate() {
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            public void onOpened(){
                //webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                //webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                MIDHEIGHT = 120;
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

    public void writeTelemetry() {
        telemetry.addData("Signal", this.signalDetect());
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();
    }
    public void nextView() {
        SignalPipe.nextView();
    }
    public int signalDetect() {
        //will return value based on which level the duck should go on
        //Yellow is 1
        //Pink is 2
        //Green is 3
        if (SignalPipe.foundColor == SignalPipeline.Color.YELLOW) {return 2;};
        if (SignalPipe.foundColor == SignalPipeline.Color.GREEN) {return 1;};
        return(3);
    }


    public static class SignalPipeline extends OpenCvPipeline {

        Mat HSVMat = new Mat();

        Scalar yellowLowHSV = new Scalar(14, 100, 100); // lower bound HSV for yellow
        Scalar yellowHighHSV = new Scalar(33, 255, 255); // higher bound HSV for yellow
        Scalar greenLowHSV = new Scalar(38, 100, 100); // lower bound HSV for green
        Scalar greenHighHSV = new Scalar(80, 255, 255); // higher bound HSV for green
        Scalar pinkLowHSV = new Scalar(140, 100, 100); //
        Scalar pinkHighHSV = new Scalar(155, 255, 255); //
        Mat thresholdMatYellow = new Mat();
        Mat thresholdMatGreen = new Mat();
        Mat thresholdMatPink = new Mat();

        Mat edgesYellow = new Mat();
        Mat edgesGreen = new Mat();
        Mat edgesPink = new Mat();

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        List<MatOfPoint> contoursGreen = new ArrayList<>();
        List<MatOfPoint> contoursPink = new ArrayList<>();

        Mat hierarchyYellow = new Mat();
        Mat hierarchyGreen = new Mat();
        Mat hierarchyPink = new Mat();

        Mat labeledYellow = new Mat();
        Mat labeledGreen = new Mat();
        Mat labeledPink = new Mat();
        Scalar labelColor = new Scalar(0, 0, 255);

        public boolean foundYellow = false; // Did we find a yellow shape in the last frame?
        public boolean foundGreen = false; // Did we find a green shape in the last frame?
        public boolean foundPink = false; // Did we find a pink shape in the last frame?

        enum Stage {
            RAW_IMAGE,
            HSV,
            THRESHOLD,
            EDGES,
            LABELED
        }
        enum Color {
            YELLOW,
            GREEN,
            OTHER
        }
        public Color foundColor = Color.OTHER;

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

            // We'll get  black and white images for each color range.
            // The white regions represent the color we are looking for.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            Core.inRange(HSVMat, yellowLowHSV, yellowHighHSV, thresholdMatYellow);
            Core.inRange(HSVMat, greenLowHSV, greenHighHSV, thresholdMatGreen);
           // Core.inRange(HSVMat, pinkLowHSV, pinkHighHSV, thresholdMatPink);

            // Use Canny Edge Detection to find edges (might have to tune the thresholds for hysteresis)
            Imgproc.Canny(thresholdMatYellow, edgesYellow, 100, 300);
            Imgproc.Canny(thresholdMatGreen, edgesGreen, 100, 300);
           // Imgproc.Canny(thresholdMatPink, edgesPink, 100, 300);

            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            contoursYellow.clear(); // empty the list from last time
            contoursGreen.clear(); // empty the list from last time
           // contoursPink.clear(); // empty the list from last time
            Imgproc.findContours(edgesYellow, contoursYellow, hierarchyYellow, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(edgesGreen, contoursGreen, hierarchyGreen, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
           // Imgproc.findContours(edgesPink, contoursPink, hierarchyPink, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            input.copyTo(labeledYellow); // make a copy of original to add labels to
            input.copyTo(labeledGreen); // make a copy of original to add labels to
           // input.copyTo(labeledPink); // make a copy of original to add labels to
            double maxGreenY = 0;
            double maxYellowY = 0;

            // if no contours, we didn't find anything
            if (!contoursYellow.isEmpty()) {
                // find the bounding rectangles of those contours, compute midpoint, and prepare labeled mat
                MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contoursYellow.size()];
                Rect[] boundRect = new Rect[contoursYellow.size()];
                for (int i = 0; i < contoursYellow.size(); i++) {
                    contoursPoly[i] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contoursYellow.get(i).toArray()), contoursPoly[i], 3, true);
                    boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                    Imgproc.rectangle(labeledYellow, boundRect[i], labelColor, 5);
                    if (boundRect[i].br().y > maxYellowY) {
                        maxYellowY = boundRect[i].br().y;
                    }
                }
            }
            if (!contoursGreen.isEmpty()) {
                // find the bounding rectangles of those contours, compute midpoint, and prepare labeled mat
                MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contoursGreen.size()];
                Rect[] boundRect = new Rect[contoursGreen.size()];
                for (int i = 0; i < contoursGreen.size(); i++) {
                    contoursPoly[i] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contoursGreen.get(i).toArray()), contoursPoly[i], 3, true);
                    boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                    Imgproc.rectangle(labeledGreen, boundRect[i], labelColor, 5);
                    if (boundRect[i].br().y > maxGreenY) {
                        maxGreenY = boundRect[i].br().y;
                    }
                }
            }
            if (maxYellowY > MIDHEIGHT) { // is the lowest yellow object at least partially in the lower half of the screen?
                foundColor = Color.YELLOW;
            } else if (maxGreenY > MIDHEIGHT) { // is the lowest green object at least partially in the lower half of the screen?
                foundColor = Color.GREEN;
            } else {
                foundColor = Color.OTHER;
            }
            teamUtil.log("Lowest Y:" + maxYellowY + " Lowest G:"+ maxGreenY);
            /*
            if (contoursPink.isEmpty()) {
                foundPink = false;
            } else {
                foundPink = true;
                // find the bounding rectangles of those contours, compute midpoint, and prepare labeled mat
                int farLeft = 5000;
                int farRight = 0;
                MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contoursPink.size()];
                Rect[] boundRect = new Rect[contoursPink.size()];
                for (int i = 0; i < contoursPink.size(); i++) {
                    contoursPoly[i] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contoursPink.get(i).toArray()), contoursPoly[i], 3, true);
                    boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                    Imgproc.rectangle(labeledPink, boundRect[i], labelColor, 5);
                }
            }

             */

            switch (stageToRenderToViewport) {
                case RAW_IMAGE: {
                    return input;
                }
                case HSV: {
                    return thresholdMatYellow;
                }
                case THRESHOLD: {
                    return thresholdMatGreen;
                }
                case EDGES: {
                    return labeledYellow;
                }
                case LABELED: {
                    return labeledGreen;
                }
                default: {
                    return input;
                }
            }
        }

    }
}

