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


package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DriveMain1", group="Controlled")
public class drive1 extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    //pre init
    //motors
    DcMotorEx FrontLeftmotor;
    DcMotorEx FrontRightmotor;
    DcMotorEx BackLeftmotor;
    DcMotorEx BackRightmotor;
    DcMotorEx Lift1;
    DcMotorEx Intake;
     DcMotorEx Intake3;
    //DcMotorEx Hang;
    DcMotorEx Intake2;

    //Servos

    Servo Servo1;
    double servoPosition = 0.0;
    Servo Servo2;
    double ServoPosition = 0.0;

    Servo Servo3;
    double Servo_Position = 0.0;
    Servo Servo4;
    double ServoPosition_;


    //speed variables
    double TurnSpeed = .4;
    double drivespeed = .5;


    //servo positions

    public void runOpMode() {

        //getting motors
        BackLeftmotor = hardwareMap.get(DcMotorEx.class, "BackLeftmotor");
        FrontLeftmotor = hardwareMap.get(DcMotorEx.class, "FrontLeftmotor");
        BackRightmotor = hardwareMap.get(DcMotorEx.class, "BackRightmotor");
        FrontRightmotor = hardwareMap.get(DcMotorEx.class, "FrontRightmotor");
        Lift1 = hardwareMap.get(DcMotorEx.class, "Lift1");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake2 = hardwareMap.get(DcMotorEx.class,"Intake2");
        //Hang = hardwareMap.get(DcMotorEx.class, "Hang");
        Intake3 = hardwareMap.get(DcMotorEx.class, "Intake3");


        BackRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FrontLeftmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BackRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FrontRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //motor directions
        BackLeftmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FrontLeftmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BackRightmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FrontRightmotor.setDirection(DcMotorEx.Direction.REVERSE);
        Lift1.setDirection(DcMotorEx.Direction.FORWARD);
        Intake.setDirection(DcMotorEx.Direction.FORWARD);
        Intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        //Hang.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake3.setDirection(DcMotorSimple.Direction.FORWARD);

        //getting servos
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        Servo4 = hardwareMap.get(Servo.class, "Servo4");

        //servo directions
        // Servo1.setDirection(Servo.Direction.REVERSE);
        Servo1.setDirection(Servo.Direction.FORWARD);
        Servo2.setDirection(Servo.Direction.REVERSE);
        Servo3.setDirection(Servo.Direction.REVERSE);
        Servo4.setDirection(Servo.Direction.REVERSE);

        //init
       // Servo1.setPosition(-1);
       // Servo2.setPosition(1);
        // Servo3.setPosition(.9);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //Box
            if (gamepad2.y) {
                Servo1.setPosition(-1);
                //Servo2.setPosition(-1);
            }
            else {

                Servo1.setPosition(1);

            }

           /* if (gamepad2.a) {
            Servo2.setPosition(-1);
            Servo3.setPosition(1);
           }
            else{
                Servo2.setPosition(1);
                Servo3.setPosition(-1);
            }
            */
            //Plane
            if (gamepad2.b){
                Servo4.setPosition(1);
            }
            else {
                Servo4.setPosition(-1);
            }
                //Hang.setPower(gamepad2.left_trigger * -1);
               // Hang.setPower(gamepad2.right_trigger * 1);
                Lift1.setPower(gamepad2.left_stick_y * -1);
                Intake.setPower(gamepad2.right_stick_y * 0.35);
                Intake2.setPower(gamepad2.right_stick_y * -1);
                Intake3.setPower(gamepad2.right_stick_y * -1);


                //drive
                BackLeftmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) - (gamepad1.left_stick_y + gamepad1.right_stick_x)*(drivespeed));
                FrontLeftmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) - (gamepad1.left_stick_y - gamepad1.right_stick_x)*(drivespeed));
                BackRightmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) + (gamepad1.left_stick_y - gamepad1.right_stick_x)*(drivespeed));
                FrontRightmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) + (gamepad1.left_stick_y + gamepad1.right_stick_x)*(drivespeed));
                //end of drive


            }

        }
    }
