package org.firstinspires.ftc.teamcode.OpenCV;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="PARK", group="auto")
public class PARK extends LinearOpMode {

    //pre init
    //motors
    DcMotorEx FrontLeftmotor;
    DcMotorEx FrontRightmotor;
    DcMotorEx BackLeftmotor;
    DcMotorEx BackRightmotor;
    DcMotorEx Lift1;
    DcMotorEx Intake;
     DcMotorEx Lift2;
    //DcMotorEx Hang;

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
        Intake = hardwareMap.get(DcMotorEx.class, "Intake1");
        //Hang = hardwareMap.get(DcMotorEx.class, "Hang");
        Lift2 = hardwareMap.get(DcMotorEx.class, "Lift2");


        BackRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FrontLeftmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BackRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FrontRightmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //motor directions
        BackLeftmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FrontLeftmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BackRightmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FrontRightmotor.setDirection(DcMotorEx.Direction.REVERSE);
        Lift1.setDirection(DcMotorEx.Direction.FORWARD);
        Intake.setDirection(DcMotorEx.Direction.FORWARD);
        //Hang.setDirection(DcMotorSimple.Direction.FORWARD);
        Lift2.setDirection(DcMotorSimple.Direction.FORWARD);

        //getting servos
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        Servo4 = hardwareMap.get(Servo.class, "Servo4");

        //servo directions
        Servo1.setDirection(Servo.Direction.FORWARD);
        Servo2.setDirection(Servo.Direction.REVERSE);
        Servo3.setDirection(Servo.Direction.FORWARD);
        Servo4.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        {
            BackLeftmotor.setPower(.5);
            FrontLeftmotor.setPower(.5);
            BackRightmotor.setPower(-.5);
            FrontRightmotor.setPower(-.5);
            sleep(1500);
        }
    }
}