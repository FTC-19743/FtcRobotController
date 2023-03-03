package org.firstinspires.ftc.teamcode.assemblies;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;


public class AprilTagDetector {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    OpenCvCamera camera;
    AprilTagDetectionPipeline  aprilTagDetectionPipeline;
    public AtomicBoolean ready = new AtomicBoolean(false);
    boolean isStreaming = false;
    String webcamName;
    OpenCvCameraRotation webcamRotation;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 1; // Tag ID 1 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    public AprilTagDetector(int theWebCamNum, OpenCvCameraRotation rotation, Telemetry theTelemetry, HardwareMap theHardwareMap) {
        teamUtil.log("Constructing Tag Detector");
        this.telemetry = theTelemetry;
        this.hardwareMap = theHardwareMap;
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcamName = "Webcam Left";
        webcamRotation = rotation;

        //election = new runningVoteCount(3001);  // track readings for the last 3 seconds
    }
    public int signalDetect() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() == 1) {
            return currentDetections.get(0).id;
        } else {
            return 0;
        }
    }

    // Pass in webcam number.  For PowerPlay, 1 is the right side of the robot, 2 is the left side
    public void initDetector(boolean withMonitor) {
        teamUtil.log("Initializing AprilTagDetector");

        if (withMonitor) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        }

        camera.setPipeline(aprilTagDetectionPipeline);
    }

    public void activate() {
        teamUtil.log("Tag Detector -- Activating");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                ready.set(true);
            }


            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void deactivate() {
        teamUtil.log("Tag Detector -- Shutting down");
        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose()
            {
                ready.set(false);
            }

        });
    }

    public void startProcessing() {
        teamUtil.log("Tag Detector: Starting Stream");
        camera.startStreaming(800, 600, webcamRotation);
        isStreaming = true;
    }
    public void stopProcessing() {
        teamUtil.log("Tag Detector: Stopping Stream");
        camera.stopStreaming();
        isStreaming = false;
    }
    public boolean isProcessing() {
        return isStreaming;
    }


    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("Detected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void writeTelemetry() {
        telemetry.addData("Signal:", this.signalDetect());

        telemetry.addData("Frame Count", camera.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
        telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
        telemetry.update();
    }



    class AprilTagDetectionPipeline extends OpenCvPipeline
    {
        private long nativeApriltagPtr;
        private Mat grey = new Mat();
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();

        private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
        private final Object detectionsUpdateSync = new Object();

        Rect cropRect = new  Rect(0,0,800, 256); // 800x448
        Scalar blackColor = new Scalar(0, 0, 0);

        Mat cameraMatrix;

        Scalar blue = new Scalar(7,197,235,255);
        Scalar red = new Scalar(255,0,0,255);
        Scalar green = new Scalar(0,255,0,255);
        Scalar white = new Scalar(255,255,255,255);

        double fx;
        double fy;
        double cx;
        double cy;

        // UNITS ARE METERS
        double tagsize;
        double tagsizeX;
        double tagsizeY;

        private float decimation;
        private boolean needToSetDecimation;
        private final Object decimationSync = new Object();

        public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy)
        {
            this.tagsize = tagsize;
            this.tagsizeX = tagsize;
            this.tagsizeY = tagsize;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            constructMatrix();

            // Allocate a native context object. See the corresponding deletion in the finalizer
            nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        @Override
        public void finalize()
        {
            // Might be null if createApriltagDetector() threw an exception
            if(nativeApriltagPtr != 0)
            {
                // Delete the native context we created in the constructor
                AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
                nativeApriltagPtr = 0;
            }
            else
            {
                System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
            }
        }

        @Override
        public Mat processFrame(Mat input)
        {
            // crop off top to avoid seeing other team's signal
            Imgproc.rectangle(input, cropRect, blackColor,-1);

            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            synchronized (decimationSync)
            {
                if(needToSetDecimation)
                {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                    needToSetDecimation = false;
                }
            }

            // Run AprilTag
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

            synchronized (detectionsUpdateSync)
            {
                detectionsUpdate = detections;
            }

            // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
            // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
            for(AprilTagDetection detection : detections)
            {
                Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
                drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
                draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
            }

            return input;
        }

        public void setDecimation(float decimation)
        {
            synchronized (decimationSync)
            {
                this.decimation = decimation;
                needToSetDecimation = true;
            }
        }

        public ArrayList<AprilTagDetection> getLatestDetections()
        {
            return detections;
        }

        public ArrayList<AprilTagDetection> getDetectionsUpdate()
        {
            synchronized (detectionsUpdateSync)
            {
                ArrayList<AprilTagDetection> ret = detectionsUpdate;
                detectionsUpdate = null;
                return ret;
            }
        }

        void constructMatrix()
        {
            //     Construct the camera matrix.
            //
            //      --         --
            //     | fx   0   cx |
            //     | 0    fy  cy |
            //     | 0    0   1  |
            //      --         --
            //

            cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

            cameraMatrix.put(0,0, fx);
            cameraMatrix.put(0,1,0);
            cameraMatrix.put(0,2, cx);

            cameraMatrix.put(1,0,0);
            cameraMatrix.put(1,1,fy);
            cameraMatrix.put(1,2,cy);

            cameraMatrix.put(2, 0, 0);
            cameraMatrix.put(2,1,0);
            cameraMatrix.put(2,2,1);
        }

        /**
         * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
         *
         * @param buf the RGB buffer on which to draw the marker
         * @param length the length of each of the marker 'poles'
         * @param rvec the rotation vector of the detection
         * @param tvec the translation vector of the detection
         * @param cameraMatrix the camera matrix used when finding the detection
         */
        void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(0,0,0),
                    new Point3(length,0,0),
                    new Point3(0,length,0),
                    new Point3(0,0,-length)
            );

            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Draw the marker!
            Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

            Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
        }

        void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
            //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(-tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2,-tagHeight/2,-length),
                    new Point3(-tagWidth/2,-tagHeight/2,-length));

            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Pillars
            for(int i = 0; i < 4; i++)
            {
                Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
            }

            // Base lines
            //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
            //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
            //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
            //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

            // Top lines
            Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
            Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
            Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
            Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
        }

        /**
         * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
         * original size of the tag.
         *
         * @param points the points which form the trapezoid
         * @param cameraMatrix the camera intrinsics matrix
         * @param tagsizeX the original width of the tag
         * @param tagsizeY the original height of the tag
         * @return the 6DOF pose of the camera relative to the tag
         */
        Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
        {
            // The actual 2d points of the tag detected in the image
            MatOfPoint2f points2d = new MatOfPoint2f(points);

            // The 3d points of the tag in an 'ideal projection'
            Point3[] arrayPoints3d = new Point3[4];
            arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
            arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
            MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

            // Using this information, actually solve for pose
            Pose pose = new Pose();
            Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

            return pose;
        }

        /*
         * A simple container to hold both rotation and translation
         * vectors, which together form a 6DOF pose.
         */
        class Pose
        {
            Mat rvec;
            Mat tvec;

            public Pose()
            {
                rvec = new Mat();
                tvec = new Mat();
            }

            public Pose(Mat rvec, Mat tvec)
            {
                this.rvec = rvec;
                this.tvec = tvec;
            }
        }
    }
}

