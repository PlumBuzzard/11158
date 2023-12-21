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

import static java.lang.Math.PI;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



import java.util.ArrayList;
import java.util.SplittableRandom;
@Disabled
@Autonomous(name="Auto_park",group="auto")
public class Auto_park extends LinearOpMode {


    //not camera stuff
    DcMotorEx BLmotor, BRmotor, FLmotor, FRmotor, Xrail, Xrail2;
    Servo Servo1, Servo2,Servo3,Servo4;
    ColorSensor colorSensor;
    double Kp = 0.05; // adjust as needed

/*
 //double TicksPerRevolution = 537.7;
    //mine below
    double TicksPerRevolution = 10321.44;
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
    double TicksPerDegree = 28.64;
    //strafe num
    double StrafeNum = 56;
    //end of not camera stuff

    //camera stuff

    static final double FEET_PER_METER = 3.28084;
 */

    double TicksPerRevolution = 537.7;
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
    double TicksPerDegree = 7.52;
    //strafe num
    double StrafeNum = 56;
    //end of not camera stuff

    //camera stuff

    //OpenCvCamera camera;
    // AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
   // double fx = 578.272;
 //   double fy = 578.272;
 //   double cx = 402.145;
  //  double cy = 221.506;

    // UNITS ARE METERS
  //  double tagsize = 0.166;

    // Tag ID 17, 18, and 19 from the 36h11 family
   // int LEFT = 17;
    //int MIDDLE = 18;
   // int RIGHT = 19;


    // Initialize color sensor




   // AprilTagDetection tagOfInterest = null;
    //end of camera stuff


    @Override
    public void runOpMode() throws InterruptedException {
        //getting motors
        BLmotor = hardwareMap.get(DcMotorEx.class, "BackLeftmotor");
        FLmotor = hardwareMap.get(DcMotorEx.class, "FrontLeftmotor");
        BRmotor = hardwareMap.get(DcMotorEx.class, "BackRightmotor");
        FRmotor = hardwareMap.get(DcMotorEx.class, "FrontRightmotor");
        Xrail = hardwareMap.get(DcMotorEx.class, "Lift1");
        Xrail2 = hardwareMap.get(DcMotorEx.class, "Lift2");
        //stops motors when not moving
        BLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Xrail.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Xrail2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //motor directions
        BLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BRmotor.setDirection(DcMotorEx.Direction.FORWARD);
        FRmotor.setDirection(DcMotorEx.Direction.FORWARD);
        Xrail.setDirection(DcMotorEx.Direction.FORWARD);
        Xrail2.setDirection(DcMotorEx.Direction.FORWARD);
        //getting servos
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        Servo4 = hardwareMap.get(Servo.class, "Servo4");

        //colorSensor = hardwareMap.colorSensor.get("color_sensor");

        //servo directions
        Servo1.setDirection(Servo.Direction.FORWARD);
        Servo2.setDirection(Servo.Direction.REVERSE);
        Servo3.setDirection(Servo.Direction.REVERSE);
        Servo4.setDirection(Servo.Direction.REVERSE);


        //init
        Servo2.setPosition(-1);
        Servo3.setPosition(1);



        //end of not camera stuff

        //back to camera stuff
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {





            /*
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        /*
        /* Update the telemetry */
            /*
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        //if (tagOfInterest == null) {
            //trajectory



       // } else if (tagOfInterest.id == LEFT) {
            //Left code

          //  int red = colorSensor.red();
           // int blue = colorSensor.blue();
            // int green = colorSensor.green();

            // error is the difference between the sensor reading and the target value (blue = 255)
           // double error = blue - 255;
            // correction is the proportional control constant times the error
            //double correction = Kp * error;


        }

    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
       // telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));


    }

    //end of camera stuff
    //not camera stuff
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

    //strafe right
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

    public void grippers(double position) {
        Servo2.setPosition(position);
        Servo3.setPosition(position);
    }
    public void Lift (double power, double Inches) {
        int distance = (int) (Inches * TicksPerInch);

//        Xrail.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        Xrail2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        double wDist = Xrail.getCurrentPosition() + distance;
        double wDist2 = Xrail2.getCurrentPosition() + distance;

//        Xrail2.setTargetPosition(Xrail2.getCurrentPosition() + distance);
//        Xrail.setTargetPosition(Xrail.getCurrentPosition() + distance);



        Xrail.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Xrail2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

//        Xrail2.setPower(power);
//        Xrail.setPower(power);



        while (opModeIsActive() && (Math.abs(Xrail.getCurrentPosition() - wDist) > 20)) {
            Xrail.setPower((Xrail.getCurrentPosition() - wDist) * 0.1);
            Xrail2.setPower((Xrail2.getCurrentPosition() - wDist2) * 0.1);
            telemetry.addData("Arm Position", Xrail.getCurrentPosition());
            telemetry.addData("Arm Positoin 2 ", Xrail2.getCurrentPosition());
            telemetry.update();
        }

        Xrail.setPower(0);
        Xrail2.setPower(0);

        Xrail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Xrail2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }

}
