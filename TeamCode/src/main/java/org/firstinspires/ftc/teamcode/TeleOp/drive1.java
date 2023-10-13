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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DriveMain1", group="Controlled")
public class drive1 extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    //pre init
    //motors
    DcMotorEx FrontLeftmotor;
    DcMotorEx FrontRightmotor;
    DcMotorEx BackLeftmotor;
    DcMotorEx BackRightmotor;
    DcMotorEx Lift1;
   // DcMotorEx Lift2;


    //Servos

    Servo Servo1;
    double servoPosition = 0.0;
    Servo Servo2;
    double ServoPosition = 0.0;



    //speed variables
    double TurnSpeed = .5;
    double drivespeed = .6;

    //servo positions

    public void runOpMode() {

        //getting motors
        BackLeftmotor = hardwareMap.get(DcMotorEx.class, "BackLeftmotor");
        FrontLeftmotor = hardwareMap.get(DcMotorEx.class, "FrontLeftmotor");
        BackRightmotor = hardwareMap.get(DcMotorEx.class, "BackRightmotor");
        FrontRightmotor = hardwareMap.get(DcMotorEx.class, "FrontRightmotor");
        Lift1 = hardwareMap.get(DcMotorEx.class, "Lift1");
       // Lift2 = hardwareMap.get(DcMotorEx.class, "Lift2");



        BackRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FrontLeftmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BackRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FrontRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //motor directions
        BackLeftmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FrontLeftmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BackRightmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FrontRightmotor.setDirection(DcMotorEx.Direction.REVERSE);
        Lift1.setDirection(DcMotorEx.Direction.FORWARD);
        //Lift2.setDirection(DcMotorSimple.Direction.FORWARD);

        //getting servos
        Servo1 = hardwareMap.get(Servo.class,"Servo1");
        Servo2 = hardwareMap.get(Servo.class,"Servo2");

        //servo directions
        Servo1.setDirection(Servo.Direction.FORWARD);
        Servo2.setDirection(Servo.Direction.REVERSE);

        //init
        Servo1.setPosition(.9);
        Servo2.setPosition(.9);

        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {

            if(gamepad2.a){
                Servo1.setPosition(.1);
                Servo2.setPosition(.1);
            }
            else{
                Servo1.setPosition(.9);
                Servo2.setPosition(.9);
            }




            Lift1.setPower(gamepad2.left_stick_y * -1);
            //Lift2.setPower(gamepad2.left_stick_y *-1);

            //drive
            BackLeftmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) - (gamepad1.left_stick_y + gamepad1.right_stick_x)*(drivespeed));
            FrontLeftmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) - (gamepad1.left_stick_y - gamepad1.right_stick_x)*(drivespeed));
            BackRightmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) + (gamepad1.left_stick_y - gamepad1.right_stick_x)*(drivespeed));
            FrontRightmotor.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) * TurnSpeed) + (gamepad1.left_stick_y + gamepad1.right_stick_x)*(drivespeed));
            //end of drive
        }
    }
}

