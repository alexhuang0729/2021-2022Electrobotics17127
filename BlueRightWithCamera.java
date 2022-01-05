package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Blue Right With Camera", group="StarBots2021")
public class BlueRightWithCamera extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;
    private DcMotorEx DuckMotor = null;
    private Servo servo1;
    private Servo servo2;
    private DcMotorEx flipArmMotor = null;
    private DcMotor extendoArmMotor = null;
    /*private Servo servoTest1;
    private Servo servoTest2;
    private Servo servoTest3;
    private Servo servoTest4;*/

    static final double     COUNTS_PER_MOTOR_REV    = 485 ;    //NeverRest 20 Motor 537.6
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "EBWebcam"), cameraMonitorViewId);
        FrontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        DuckMotor = hardwareMap.get(DcMotorEx.class, "duckMotor");
        servo1 = hardwareMap.get(Servo.class, "Servo1");
        servo2 = hardwareMap.get(Servo.class, "Servo2");
        flipArmMotor  = hardwareMap.get(DcMotorEx.class, "FlipArmMotor");
        extendoArmMotor = hardwareMap.get(DcMotor.class, "ExtendoArmMotor");
        /*servoTest1 = hardwareMap.get(Servo.class, "Servo1");
        servoTest2 = hardwareMap.get(Servo.class, "Servo2");
        servoTest3 = hardwareMap.get(Servo.class, "Servo3");
        servoTest4 = hardwareMap.get(Servo.class, "Servo4");*/

        // Send telemetry message to signify robot waiting;


        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendoArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendoArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DuckMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //DuckMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        //DuckMotor.setVelocityPIDFCoefficients(5.1, 0.51, 0, 51);
        DuckMotor.setVelocityPIDFCoefficients(8, .2 , 0.1, 51);

        DuckMotor.setPositionPIDFCoefficients(5.0);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DuckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendoArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            servo1.setPosition(0);
            telemetry.addData("Left", pipeline.getType3());
            telemetry.addData("Center", pipeline.getType());
            telemetry.addData("Center Average", pipeline.getAverage());
            telemetry.addData("Right", pipeline.getType2());
            telemetry.addData("Right Average", pipeline.getAverage2());
            telemetry.update();
            sleep(3500);
            if (pipeline.getAverage() < 100) {
                encoderDrive(DRIVE_SPEED, 17.0, 17.0, 17.0, 17.0, 5.0);
                ArmEncoder(-1, 780, 5.0);
                encoderDrive(DRIVE_SPEED, -23, 23, 23, -23, 5.0);
                extendoArmEncoder(-1, -1200, 2.0);
                servo1.setPosition(.55);
                sleep(500);
                encoderDrive(DRIVE_SPEED, 5, -5, -5, 5, 5.0);
                encoderDrive(DRIVE_SPEED, -5.0, -5, -5, -5, 5.0);
                extendoArmEncoder(1, 1200, 2.0);
                ArmEncoder(1, -780, 5.0);
                encoderDrive(DRIVE_SPEED, -12, 12, 12, -12, 5.0);
                encoderDrive(DRIVE_SPEED, 4, 4, 4, 4, 5.0);
                restOfProgram();
                break;
            }
            else if ( pipeline.getAverage2() < 100)
            {
                encoderDrive(DRIVE_SPEED, 5.0, 5.0, 5.0, 5.0, 5.0);
                encoderDrive(DRIVE_SPEED, -33, 33, 33, -33, 5.0);
                encoderDrive(.3, 15, 15, 15, 15, 5.0);

                ArmEncoder(-1, 585, 5.0);
                extendoArmEncoder(-1, -3400, 2.0);
                servo1.setPosition(.55);
                sleep(100);
                ArmEncoder(.75, -200, 2);
                encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 5.0);
                sleep(100);
                extendoArmEncoder(1, 3400, 2.0);
                servo1.setPosition(0);
                ArmEncoder(.75, -200, 2);
                flipArmMotor.setPower(0);
                restOfProgram();
                break;
            }
            else {
                encoderDrive(DRIVE_SPEED, 17.0, 17.0, 17.0, 17.0, 5.0);
                ArmEncoder(-1, 900, 5.0);
                encoderDrive(DRIVE_SPEED, -23, 23, 23, -23, 5.0);
                extendoArmEncoder(-1, -800, 2.0);
                servo1.setPosition(.55);
                sleep(500);
                encoderDrive(DRIVE_SPEED, 5, -5, -5, 5, 5.0);
                encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 5.0);
                extendoArmEncoder(1, 800, 2.0);
                ArmEncoder(1, -900, 5.0);
                encoderDrive(DRIVE_SPEED, -12, 12, 12, -12, 5.0);
                encoderDrive(DRIVE_SPEED, 4, 4, 4, 4, 5.0);
                restOfProgram();
                break;
            }

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
            if (type == TYPE.BACKGROUND && type2 == TYPE.BACKGROUND) {
                type3 = TYPE.CAPSTONE;
            }
            if (type == TYPE.CAPSTONE || type2 == TYPE.CAPSTONE) {
                type3 = TYPE.BACKGROUND;
            }
            return input;
        }

        public TYPE getType() {
            return type;
        }

        public TYPE getType2() {
            return type2;
        }

        public TYPE getType3() {
            return type3;
        }

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
    public void encoderDrive(double speed,
                             double FrontLeftInches, double FrontRightInches, double BackLeftInches, double BackRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = FrontLeft.getCurrentPosition() - (int)(FrontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRight.getCurrentPosition() + (int)(FrontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = BackLeft.getCurrentPosition() - (int)(BackLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = BackRight.getCurrentPosition() + (int)(BackRightInches * COUNTS_PER_INCH);
            FrontLeft.setTargetPosition(newFrontLeftTarget);
            FrontRight.setTargetPosition(newFrontRightTarget);
            BackLeft.setTargetPosition(newBackLeftTarget);
            BackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy()&&BackLeft.isBusy()&&BackRight.isBusy())) {
                // (leftDrive.isBusy() || rightDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d: %7d: %7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d: %7d: %7d",
                        FrontLeft.getCurrentPosition(),
                        FrontRight.getCurrentPosition(),
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.addData("Left", pipeline.getType3());
                telemetry.addData("Center", pipeline.getType());
                telemetry.addData("Center Average", pipeline.getAverage());
                telemetry.addData("Right", pipeline.getType2());
                telemetry.addData("Right Average", pipeline.getAverage2());
                telemetry.update();
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void ArmEncoder(double ArmFlipPower, double ArmTravelLength, double timeoutSA) {

        int newArmTarget;

        if (opModeIsActive()) {
            newArmTarget = flipArmMotor.getCurrentPosition() + (int)(ArmTravelLength);
            flipArmMotor.setTargetPosition(newArmTarget);
            flipArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            runtime.reset();
            flipArmMotor.setVelocity(ArmFlipPower*600);


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSA) &&(flipArmMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("flpArmMotorPosition", "Running at %7d ", flipArmMotor.getCurrentPosition());
                telemetry.update();
            }



            // Turn off RUN_TO_POSITION

            sleep(250);
        }
    }
    public void extendoArmEncoder(double extendoArmPower, double extendoArmTravel, double timeoutSE) {

        int newExtendoArmTarget;

        if (opModeIsActive()) {
            newExtendoArmTarget = extendoArmMotor.getCurrentPosition() + (int)(extendoArmTravel);
            extendoArmMotor.setTargetPosition(newExtendoArmTarget);
            extendoArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            extendoArmMotor.setPower(extendoArmPower);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSE) &&(extendoArmMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("extendoArmMotorPosition", "Running at %7d ", extendoArmMotor.getCurrentPosition());
                telemetry.update();
            }

            extendoArmMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            extendoArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
    public void restOfProgram() {
        encoderDrive(DRIVE_SPEED, 70, -70, -70, 70, 5.0);
        encoderDrive(DRIVE_SPEED, -24, 24, -24, 24, 5.0);
        encoderDrive(.2, -2, -2, -2, -2, 5.0);
        /*while (opModeIsActive() && runtime.seconds() < 1) {
            DuckMotor.setPower(-.80);
         }
        encoderDriveWithDuck(.2, -11.5, 11.5, 11.5, -11.5, .40, 20);*/
        encoderDrive(.2, -12, 12, 12, -12, 5.0);

        while (opModeIsActive() && runtime.seconds() < 5) {
            DuckMotor.setVelocity(165);//Should be negative when on the red side; positive when on the blue side
        }
        DuckMotor.setVelocity(0);
        encoderDrive(DRIVE_SPEED, 25, -25, -25, 25, 5.0);

    }
}