package org.firstinspires.ftc.teamcode.TeleOp;






import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="test1", group="Controlled")
    public class java_test extends OpMode {

        private DcMotor frontLeft, frontRight, backLeft, backRight,Lift1,Lift2,Intake2,Intake1;
    Servo Servo1;
    double servoPosition = 0.0;
    Servo Servo2;
    double ServoPosition = 0.0;

    Servo Servo3;
    double Servo_Position = 0.0;
    Servo Servo4;
    double ServoPosition_ = 0.0;

    Servo Servo5;
    double Servo_Position_ = 0.0;

        @Override
        public void init() {
            frontLeft = hardwareMap.dcMotor.get("FrontLeftmotor");
            frontRight = hardwareMap.dcMotor.get("FrontRightmotor");
            backLeft = hardwareMap.dcMotor.get("BackLeftmotor");
            backRight = hardwareMap.dcMotor.get("BackRightmotor");
            Lift1 = hardwareMap.get(DcMotor.class, "Lift1");
            Lift2 = hardwareMap.get(DcMotor.class, "Lift2");
            Intake1 = hardwareMap.get(DcMotor.class,"Intake1");
            Intake2 = hardwareMap.get(DcMotor.class,"Intake2");

            // Set motor directions
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);
            Lift1.setDirection(DcMotorEx.Direction.FORWARD);
            Lift2.setDirection(DcMotorEx.Direction.FORWARD);
            Intake1.setDirection(DcMotor.Direction.FORWARD);
            Intake2.setDirection(DcMotor.Direction.FORWARD);

            //getting servos
            Servo1 = hardwareMap.get(Servo.class, "Servo1");
            Servo2 = hardwareMap.get(Servo.class, "Servo2");
            Servo3 = hardwareMap.get(Servo.class, "Servo3");
            Servo4 = hardwareMap.get(Servo.class, "Servo4");

            //servo directions
            // Servo1.setDirection(Servo.Direction.REVERSE);
            Servo1.setDirection(Servo.Direction.FORWARD);
            Servo2.setDirection(Servo.Direction.REVERSE);
            Servo3.setDirection(Servo.Direction.FORWARD);
            Servo4.setDirection(Servo.Direction.REVERSE);

            // Set motor modes
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public void loop() {
            // Gamepad inputs
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Calculate motor powers
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize motor powers
            double maxPower = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );
            if (maxPower == 7.0) {
                frontLeftPower /= 0.7;
                frontRightPower /= 0.7;
                backLeftPower /= 0.7;
                backRightPower /= 0.7;
            }
                //Box
                if (gamepad2.y) {
                    Servo1.setPosition(-0.5);
                    Servo5.setPosition(0.5);
                }
                else {

                    Servo1.setPosition(1);
                }
                //Plane
                if (gamepad2.x){
                    Servo2.setPosition(1);
                }
                else {
                    Servo2.setPosition(-1);
                }
                // stopper
                if (gamepad1.a){
                    Servo3.setPosition(1);
                } else if (gamepad1.y) {
                    Servo3.setPosition(-1);
                }


                // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            Lift1.setPower(gamepad2.left_stick_y * -1);
            Lift2.setPower(gamepad2.left_stick_y * -1);
            Intake1.setPower(gamepad2.right_stick_y * 1);
            Intake2.setPower(gamepad2.right_stick_y* 1);

        }
    }


