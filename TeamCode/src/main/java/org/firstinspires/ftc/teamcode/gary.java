package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.teamcode.BuildConfig.DEBUG;

@Config
@TeleOp(name = "gary")
public class gary extends LinearOpMode {
    MecanumDrivetrain drive;
    FtcDashboard dashboard;
    DcMotor in_front;
    DcMotor in_back;
    DcMotor wobbleGoal;
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;
    DcMotor shooter;
    Servo wobbleGrab;
    protected static final double DEFAULT_POWER_REDUCTION_FACTOR = 0.4;
    protected double powerReductionFactor = DEFAULT_POWER_REDUCTION_FACTOR;
    double turnPowerFactor = 0.6;
    double power_multiplier;
    double basePower = 0.2;


    public void teleopInitFn() {
        drive = new MecanumDrivetrain(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        in_front = hardwareMap.dcMotor.get("in_front");
        in_back = hardwareMap.dcMotor.get("in_back");
        wobbleGoal = hardwareMap.dcMotor.get("WobbleGoal");
        shooter = hardwareMap.dcMotor.get("Launch");


        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightRear");
        leftBack = hardwareMap.dcMotor.get("leftRear");

        wobbleGrab = hardwareMap.servo.get("wobbleGrab");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void vectorCombineRoadrunner(double x, double y) {
        double maxPower = 0.8;

        double a = y + x;
        double b = y - x;

        double scalar = maxPower / Math.max(a, b);

        a *= scalar;
        b *= scalar;

        drive.setMotorPowers(a, b, a, b);
    }

    public void vectorCombine(double x, double y, double turn) {


        double a = powerReductionFactor * (x + y) + basePower * ((x + y) / Math.abs(x + y));
        double b = powerReductionFactor * (y - x) + basePower * ((y - x) / Math.abs(y - x));
        double c = -powerReductionFactor * (y - x) - basePower * ((y - x) / Math.abs(y - x));
        double d = -powerReductionFactor * (x + y) - basePower * ((x + y) / Math.abs(x + y));

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setPower(a + turn);
        rightFront.setPower(b - turn);
        leftBack.setPower(c - turn);
        rightBack.setPower(d + turn);

        if (DEBUG)
            System.out.println("14564dbg vectorCombine a " + a + " b " + b + " c " + c + " d " + d);
//        telemetry.addData("x:", x);
//        telemetry.addData("y:", y);
//
//        telemetry.addData("a:", a);
//        telemetry.addData("b:", b);
//        telemetry.addData("c:", c);
//        telemetry.addData("d:", d);
//
//        telemetry.update();

    }


    @Override
    public void runOpMode() {

        teleopInitFn();

        waitForStart();

        int counter = 0;

        while (opModeIsActive()) {

//                        float forward = -1 * gamepad1.right_stick_y;
//                        double turn_component = gamepad1.left_stick_x * turnPowerFactor;
//
//                        double x_component = gamepad1.right_stick_x;
//                        double y_component = -1 * gamepad1.right_stick_y;
//
//                        if (y_component == 0)
//                                y_component = 0.001;
//                        if (x_component == 0)
//                                x_component = 0.001;
//
//
//                        if (gamepad1.right_trigger > 0) {
//                                powerReductionFactor = DEFAULT_POWER_REDUCTION_FACTOR + (gamepad1.right_trigger * 2 / 3);
//                                //Subtract from 1 to make the trigger give a reduction in power
//                                //Multiply by 2/3 to not completely reduce the power
//                        }


//                        if (gamepad1.left_bumper && Math.abs(forward) > 0.1) {
//                                //right turn
//                                leftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //changed
//                                leftBack.setDirection(DcMotorSimple.Direction.FORWARD);   //changed
//                                rightFront.setDirection(DcMotorSimple.Direction.FORWARD); //default
//                                rightBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
//                                rightFront.setPower(forward * power_multiplier);
//                                rightBack.setPower(forward * power_multiplier);
//                                leftFront.setPower(forward * power_multiplier);
//                                leftBack.setPower(forward * power_multiplier);
//                        } else if (gamepad1.right_bumper && Math.abs(forward) > 0.1) {
//                                //left turn
//                                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);  //default
//                                leftBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
//                                rightFront.setDirection(DcMotorSimple.Direction.REVERSE); //changed
//                                rightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //changed
//                                rightFront.setPower(forward * power_multiplier);
//                                rightBack.setPower(forward * power_multiplier);
//                                leftFront.setPower(forward * power_multiplier);
//                                leftBack.setPower(forward * power_multiplier);
//                        } else if (gamepad1.left_trigger > 0.1 && Math.abs(x_component) > 0.1) {
//                                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);  //default
//                                leftBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
//                                rightFront.setDirection(DcMotorSimple.Direction.FORWARD); //default
//                                rightBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
//
//                                rightFront.setPower(-0.5 * (x_component / Math.abs(x_component)) * power_multiplier);
//                                rightBack.setPower(0.5 * (x_component / Math.abs(x_component)) * power_multiplier);
//                                leftFront.setPower(0.5 * (x_component / Math.abs(x_component)) * power_multiplier);
//                                leftBack.setPower(-0.5 * (x_component / Math.abs(x_component)) * power_multiplier);
//
//                        } else if (gamepad1.left_trigger > 0.1 && Math.abs(y_component) > 0.1) {
//                                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);  //default
//                                leftBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
//                                rightFront.setDirection(DcMotorSimple.Direction.FORWARD); //default
//                                rightBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
//
//                                rightFront.setPower(0.3 * (y_component / Math.abs(y_component)) * power_multiplier);
//                                rightBack.setPower(0.3 * (y_component / Math.abs(y_component)) * power_multiplier);
//                                leftFront.setPower(0.3 * (y_component / Math.abs(y_component)) * power_multiplier);
//                                leftBack.setPower(0.3 * (y_component / Math.abs(y_component)) * power_multiplier);
//
//                        }
//                else if (gamepad1.left_trigger > 0.1 && Math.abs(y_component) > 0.1){
//                    vectorCombineSimple(0, 0.5*(y_component/Math.abs(y_component)), 0);
//                }


//                    if (((Math.abs(x_component) > 0.1) || (Math.abs(y_component) > 0.1) && gamepad1.left_bumper)) {
//                                vectorCombine(x_component, y_component, turn_component * (y_component / Math.abs(y_component)));
//                                idle();
//
//                        } else {
/*
{
                                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                rightFront.setPower(0);
                                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                rightBack.setPower(0);
                                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                leftFront.setPower(0);
                                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                leftBack.setPower(0);


                        }*/
            if (gamepad1.left_bumper) {
                vectorCombineRoadrunner(gamepad1.left_stick_x, gamepad1.left_stick_y);
            }
//            } else if(Math.abs(gamepad1.right_stick_y) > 0.1){
//                rightFront.setPower(gamepad1.right_stick_y);
//                rightBack.setPower(gamepad1.right_stick_y);
//                leftFront.setPower(gamepad1.right_stick_y);
//                leftBack.setPower(gamepad1.right_stick_y);
//            } else if(Math.abs(gamepad1.left_stick_x) > 0.1){
//                rightFront.setPower(-gamepad1.left_stick_x);
//                rightBack.setPower(gamepad1.left_stick_x);
//                leftFront.setPower(gamepad1.left_stick_x);
//                leftBack.setPower(-gamepad1.left_stick_x);
//            } else if(Math.abs(gamepad1.right_stick_y) > 0.1 && gamepad1.right_bumper){
//                rightFront.setPower(gamepad1.right_stick_y);
//                rightBack.setPower(gamepad1.right_stick_y);
//                leftFront.setPower(-gamepad1.right_stick_y);
//                leftBack.setPower(-gamepad1.right_stick_y);
//            } else {
//                rightBack.setPower(0);
//                rightFront.setPower(0);
//                leftBack.setPower(0);
//                leftFront.setPower(0);
//            }
            if (gamepad2.left_trigger > 0.1) {
                in_front.setPower(-1);
                in_back.setPower(-1);
            }
            if (gamepad2.right_trigger > 0.1) {
                in_front.setPower(1);
                in_back.setPower(1);
            }
//
            if (gamepad2.y) {
                wobbleGoal.setPower(1);
            }
            if (gamepad2.x) {
                wobbleGoal.setPower(-1);
            }
            if (gamepad2.right_bumper) {
                shooter.setPower(-0.8);

            }
            if (gamepad2.left_bumper) {
                shooter.setPower(0);

            }
            //TelemetryPacket hi = new TelemetryPacket();
            if (Math.abs(gamepad1.right_stick_y) > 0.01) {
                rightFront.setPower(gamepad1.right_stick_y);
                rightBack.setPower(gamepad1.right_stick_y);

            }

            if (Math.abs(gamepad1.left_stick_y) > 0.01) {

                leftFront.setPower(gamepad1.left_stick_y);
                leftBack.setPower(gamepad1.left_stick_y);


            }

            if (gamepad1.right_bumper&&Math.abs(gamepad1.right_stick_x) > 0.01) {

                rightFront.setPower(gamepad1.right_stick_x);
                rightBack.setPower(-gamepad1.right_stick_x);
                leftFront.setPower(-gamepad1.right_stick_x);
                leftBack.setPower(gamepad1.right_stick_x);
//                        hi_parth.put("Left Stick y", gamepad1.left_stick_y);
//                        hi_parth.put("Left Stick x", gamepad1.left_stick_x);
            }
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) < 0.05)
                drive.setMotorPowers(0, 0, 0, 0);

            if (gamepad2.dpad_left) {
                wobbleGrab.setPosition(0);

            }
            if (gamepad2.dpad_right) {
                wobbleGrab.setPosition(1);

            }
//            else{
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//            leftBack.setPower(0);
//            leftFront.setPower(0);
//            }

            wobbleGoal.setPower(0);
            in_front.setPower(0);
            in_back.setPower(0);
            telemetry.addData("shooter power: ", shooter.getPower());
            telemetry.update();
        }

        //dashboard.sendTelemetryPacket(hi);
//                    }

    }



    }














