package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class CaporBack extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "EBWebcam"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left", pipeline.getType3());
            telemetry.addData("Center", pipeline.getType());
            telemetry.addData("Center Average", pipeline.getAverage());
            telemetry.addData("Right", pipeline.getType2());
            telemetry.addData("Right Average", pipeline.getAverage2());
            telemetry.update();
            sleep(50);
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);
        private static final Scalar GOLD = new Scalar(255, 215, 0);
        private static final int THRESHOLD = 100;

        Point topLeft1 = new Point(70, 80);
        Point bottomRight1 = new Point(100, 110);
        Point topLeft2 = new Point(250, 80);
        Point bottomRight2 = new Point(280, 110);

        Mat region1_Cb;
        Mat region2_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat YCrCb2 = new Mat();
        Mat Cb2 = new Mat();

        private volatile int average;
        private volatile int average2;
        private volatile TYPE type = TYPE.BACKGROUND;
        private volatile TYPE type2 = TYPE.BACKGROUND;
        private volatile TYPE type3 = TYPE.BACKGROUND;



        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
            Imgproc.cvtColor(input, YCrCb2, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb2, Cb2, 2);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            region1_Cb = Cb.submat(new Rect(topLeft1, bottomRight1));
            region2_Cb = Cb2.submat(new Rect(topLeft2, bottomRight2));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            average = (int) Core.mean(region1_Cb).val[0];
            average2 = (int) Core.mean(region2_Cb).val[0];


            Imgproc.rectangle(input, topLeft1, bottomRight1, BLUE, 2);
            Imgproc.rectangle(input, topLeft2, bottomRight2, GOLD, 2);


            if (average > THRESHOLD) {
                type = TYPE.BACKGROUND;
            } else {
                type = TYPE.CAPSTONE;
            }
            if (average2 > THRESHOLD) {
                type2 = TYPE.BACKGROUND;
            } else {
                type2 = TYPE.CAPSTONE;
            }
            if ( type == TYPE.BACKGROUND && type2 == TYPE.BACKGROUND) {
                type3 = TYPE.CAPSTONE;
            }
            if ( type == TYPE.CAPSTONE || type2 == TYPE.CAPSTONE ) {
                type3 = TYPE.BACKGROUND;
            }
            return input;
        }

        public TYPE getType() { return type; }
        public TYPE getType2() { return type2; }
        public TYPE getType3() { return type3; }
        public int getAverage() {
            return average;
        }
        public int getAverage2() {
            return average2;
        }

        public enum TYPE {
            BACKGROUND, CAPSTONE
        }
    }
}