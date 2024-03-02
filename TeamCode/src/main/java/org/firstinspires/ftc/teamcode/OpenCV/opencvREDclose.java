/*
 * MIT License
 *
 * Copyright (c) 2023 John Garcia
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.OpenCV;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static java.lang.Math.PI;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Red closeside")

public class opencvREDclose extends LinearOpMode {

    //not camera stuff
    DcMotorEx BLmotor, BRmotor, FLmotor, FRmotor, Xrail,Xrail2,Intake2,Intake1;
    Servo Servo1, Servo2,Servo3,Servo4,Servo5;
    ColorSensor colorSensor;
    double Kp = 0.05; // adjust as needed
    double cX = 0;
    double cY = 0;
    double width = 0;

    //double TicksPerRevolution = 537.7;
    //mine belo
    double TicksPerRevolution = 537.6;
    // diameter Millimeters
    double DiameterMM = 96;
    // diameter Inches
    double DiameterIn = DiameterMM / 25.4; //3.7795275590551185 Inches
    // Circumference
    double Circumference = DiameterIn * PI; //11.873736013567724 Inches
    // Finding ticks per inch
    double TicksPerInch = TicksPerRevolution / Circumference; //45.28482015985433 Inches

    //**guessed values**
    //Ticks per degree
    //double TicksPerDegree = 7.52;
    //MINE BELOW
    double TicksPerDegree = 28.72;
    //strafe num
    double StrafeNum = 56;
    //end of not camera stuff

    //camera stuff

    static final double FEET_PER_METER = 3.28084;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.5;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    static final double COUNTS_PER_MOTOR_REV = 1120; // Number of encoder ticks per revolution
    static final double WHEEL_DIAMETER_INCHES = 4.0; // Diameter of the wheel in inches
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);


    public void runOpMode() throws InterruptedException {
        //getting motors
        BLmotor = hardwareMap.get(DcMotorEx.class, "BackLeftmotor");
        FLmotor = hardwareMap.get(DcMotorEx.class, "FrontLeftmotor");
        BRmotor = hardwareMap.get(DcMotorEx.class, "BackRightmotor");
        FRmotor = hardwareMap.get(DcMotorEx.class, "FrontRightmotor");
        Xrail = hardwareMap.get(DcMotorEx.class, "Lift1");
        Xrail2 = hardwareMap.get(DcMotorEx.class, "Lift2");
        Intake1 = hardwareMap.get(DcMotorEx.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotorEx.class,"Intake2");
        //stops motors when not moving
        BLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Xrail.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Xrail2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor directions
        BLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BRmotor.setDirection(DcMotorEx.Direction.FORWARD);
        FRmotor.setDirection(DcMotorEx.Direction.FORWARD);
        Xrail.setDirection(DcMotorEx.Direction.FORWARD);
        Xrail2.setDirection(DcMotorEx.Direction.FORWARD);
        Intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        //getting servos
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        Servo4 = hardwareMap.get(Servo.class, "Servo4");
        Servo5 = hardwareMap.get(Servo.class, "Servo5");
        //colorSensor = hardwareMap.colorSensor.get("color_sensor");

        //servo directions
        Servo1.setDirection(Servo.Direction.FORWARD);
        Servo2.setDirection(Servo.Direction.REVERSE);
        Servo3.setDirection(Servo.Direction.REVERSE);
        Servo4.setDirection(Servo.Direction.REVERSE);


        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

                waitForStart();

                if (cX < CAMERA_WIDTH / 2 - 50) {
                    // If the centroid is significantly to the left of the screen center, turn the robot left
                    forward(0.3,2);
                    sleep(100);
                    Sleft(0.3,4);
                    forward(0.3,15);
                    backwards(0.3,15);
                    sleep(1000);
                    Sleft(0.5,20);
                    sleep(100);

                    requestOpModeStop();
                } else if (cX > CAMERA_WIDTH / 2 + 50) {
                    // If the centroid is significantly to the right of the screen center, turn the robot right
                    forward(0.3,16);
                    TurnLeft(0.3,23);
                    forward(0.2,4);
                    sleep(100);
                    backwards(0.2,4);
                    TurnRight(0.3,23);
                    backwards(0.3,16);
                    sleep(1000);
                    Sleft(0.5,20);
                    sleep(100);
                    requestOpModeStop();

                } else {
                    // If the centroid is close to the center, move the robot forward
                    forward(0.3,20);
                    sleep(100);
                    backwards(0.3,20);
                   Sleft(0.5,20);
                   sleep(100);



                    requestOpModeStop();
                }


            }

            return input;
        }
        public void forward(double power, double Inches) {
            int distance = (int) (Inches * TicksPerInch);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            FRmotor.setTargetPosition(distance);
            FLmotor.setTargetPosition(distance);
            BRmotor.setTargetPosition(distance);
            BLmotor.setTargetPosition(distance);

            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            FLmotor.setPower(power);
            FRmotor.setPower(power);
            BLmotor.setPower(power);
            BRmotor.setPower(power);

            while (FRmotor.isBusy() &&
                    FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy()) {
            }

            FRmotor.setPower(0);
            FLmotor.setPower(0);
            BRmotor.setPower(0);
            BLmotor.setPower(0);
            FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        //backwards
        public void backwards(double power, double Inches) {
            int distance = (int) (Inches * TicksPerInch);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            FRmotor.setTargetPosition(-distance);
            FLmotor.setTargetPosition(-distance);
            BRmotor.setTargetPosition(-distance);
            BLmotor.setTargetPosition(-distance);

            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            FRmotor.setPower(-power);
            FLmotor.setPower(-power);
            BRmotor.setPower(-power);
            BLmotor.setPower(-power);

            while (FRmotor.isBusy() &&
                    FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy()) {
            }

            FRmotor.setPower(0);
            FLmotor.setPower(0);
            BRmotor.setPower(0);
            BLmotor.setPower(0);
            FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        //strafing stuff
        //strafe left
        public void Sright(double power, double Inches) {
            int distance = (int) (Inches * StrafeNum);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            FRmotor.setTargetPosition(-distance);
            FLmotor.setTargetPosition(distance);
            BRmotor.setTargetPosition(distance);
            BLmotor.setTargetPosition(-distance);

            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            FRmotor.setPower(-power);
            FLmotor.setPower(power);
            BRmotor.setPower(power);
            BLmotor.setPower(-power);

            while (FRmotor.isBusy() &&
                    FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy()) {
            }

            FRmotor.setPower(0);
            FLmotor.setPower(0);
            BRmotor.setPower(0);
            BLmotor.setPower(0);
            FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        //strafe right
        public void Sleft(double power, double Inches) {
            int distance = (int) (Inches * StrafeNum);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            FRmotor.setTargetPosition(distance);
            FLmotor.setTargetPosition(-distance);
            BRmotor.setTargetPosition(-distance);
            BLmotor.setTargetPosition(distance);

            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            FRmotor.setPower(power);
            FLmotor.setPower(-power);
            BRmotor.setPower(-power);
            BLmotor.setPower(power);

            while (FRmotor.isBusy() &&
                    FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy()) {
            }

            FRmotor.setPower(0);
            FLmotor.setPower(0);
            BRmotor.setPower(0);
            BLmotor.setPower(0);
            FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }



        //turning stuff
        //diagonal front right
        public void DFright(double power, double Inches) {
            int distance = (int) (Inches * TicksPerInch);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


            FRmotor.setTargetPosition(distance);
            BLmotor.setTargetPosition(distance);


            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


            FRmotor.setPower(power);
            BLmotor.setPower(power);

            while (FRmotor.isBusy() && BLmotor.isBusy()) {
            }

            FRmotor.setPower(0);
            BLmotor.setPower(0);
            FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        //diagonal front left
        public void DFleft(double power, double Inches) {
            int distance = (int) (Inches * TicksPerInch);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


            FRmotor.setTargetPosition(distance);
            BLmotor.setTargetPosition(distance);


            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


            FLmotor.setPower(power);
            BRmotor.setPower(power);

            while (FRmotor.isBusy() && BLmotor.isBusy()) {
            }

            FLmotor.setPower(0);
            BRmotor.setPower(0);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        //diagonal back right
        public void DBright(double power, double Inches) {
            int distance = (int) (Inches * TicksPerInch);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


            FRmotor.setTargetPosition(-distance);
            BLmotor.setTargetPosition(-distance);


            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


            FRmotor.setPower(-power);
            BLmotor.setPower(-power);

            while (FRmotor.isBusy() && BLmotor.isBusy()) {
            }

            FRmotor.setPower(0);
            BLmotor.setPower(0);
            FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        //diagonal back left
        public void DBleft(double power, double Inches) {
            int distance = (int) (Inches * TicksPerInch);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


            FRmotor.setTargetPosition(-distance);
            BLmotor.setTargetPosition(-distance);


            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


            FLmotor.setPower(-power);
            BRmotor.setPower(-power);

            while (FRmotor.isBusy() && BLmotor.isBusy()) {
            }

            FLmotor.setPower(0);
            BRmotor.setPower(0);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        //turning
        public void TurnRight(double power, double degrees) {

            int distance = (int) (degrees * TicksPerDegree);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            FRmotor.setTargetPosition(-distance);
            FLmotor.setTargetPosition(distance);
            BRmotor.setTargetPosition(-distance);
            BLmotor.setTargetPosition(distance);

            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            FRmotor.setPower(-power);
            FLmotor.setPower(power);
            BRmotor.setPower(-power);
            BLmotor.setPower(power);

            while (opModeIsActive() && (FRmotor.isBusy() &&
                    FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy())) {
            }

            FRmotor.setPower(0);
            FLmotor.setPower(0);
            BRmotor.setPower(0);
            BLmotor.setPower(0);
            FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        public void TurnLeft(double power, double degrees) {
            int distance = (int) (degrees * TicksPerDegree);

            FRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            FRmotor.setTargetPosition(distance);
            FLmotor.setTargetPosition(-distance);
            BRmotor.setTargetPosition(distance);
            BLmotor.setTargetPosition(-distance);

            FRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            FRmotor.setPower(power);
            FLmotor.setPower(-power);
            BRmotor.setPower(power);
            BLmotor.setPower(-power);

            while (opModeIsActive() && (FRmotor.isBusy() &&
                    FLmotor.isBusy() && BRmotor.isBusy() && BLmotor.isBusy())) {
            }

            FRmotor.setPower(0);
            FLmotor.setPower(0);
            BRmotor.setPower(0);
            BLmotor.setPower(0);
            FRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            FLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        private void moveServo(double position) {
            // Ensure the position is within the valid range (0.0 to 1.0)
            position = Math.min(Math.max(position, 0.0), 1.0);

            // Set the servo position
            Servo1.setPosition(position);

            // Wait for the servo to reach the target position
            // You may adjust the sleep duration based on the servo's speed and the desired position change
            sleep(1000); // Sleep for 0.5 seconds
        }
    }

    // Method to lift up
    private void Lift(double power) {
        Xrail2.setPower(power);
        Xrail.setPower(power);
    }

    // Method to stop the lift
    private void stopLift() {
        Xrail2.setPower(0);
        Xrail.setPower(0);
    }
        /*
        private void Lift(double inches, double power) {
            int targetPosition = (int) (inches * COUNTS_PER_INCH);

            Xrail.setTargetPosition(targetPosition);
            Xrail2.setTargetPosition(targetPosition);

            Xrail.setPower(power);
            Xrail2.setPower(power);

            // Wait for the motors to reach the target position
            while (opModeIsActive() && (Xrail.isBusy() || Xrail2.isBusy())) {
                telemetry.addData("Status", "Running to target position...");
                telemetry.update();
            }

            // Stop the motors after reaching the target position
            Xrail.setPower(0);
            Xrail2.setPower(0);

            // Reset motor encoders
            Xrail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Xrail2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Xrail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Xrail2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
///////////////////
        /*
            double wDist = Xrail.getCurrentPosition() + distance;
            double wDist2 = Xrail2.getCurrentPosition() + distance;

            Xrail2.setTargetPosition(Xrail2.getCurrentPosition() + distance);
            Xrail.setTargetPosition(Xrail.getCurrentPosition() + distance);


            Xrail.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            Xrail2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            Xrail2.setPower(power);
            Xrail.setPower(power);


            while (opModeIsActive() && (Math.abs(Xrail.getCurrentPosition() - wDist) > 20)) {
                Xrail.setPower((Xrail.getCurrentPosition() - wDist) * 0.1);
                Xrail2.setPower((Xrail2.getCurrentPosition() - wDist2) * 0.1);
                telemetry.addData("Arm Position", Xrail.getCurrentPosition());
                telemetry.addData("Arm Positoin 2 ", Xrail2.getCurrentPosition());
                telemetry.update();
            }



 */


    //RED
    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowerYellow = new Scalar(0, 100, 100);
        Scalar upperYellow = new Scalar(100, 255, 255);


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }
    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }


    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }


}
