package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.bosch.BNO055IMU;
import android.os.Bundle;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@TeleOp (name = "Teleop2021")
public class Teleop2021 extends LinearOpMode {

    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;
    //leftFront, leftRear, rightRear, rightFront
    DcMotor launch;
    DcMotor intake;
    DcMotor ramp;
    DcMotor wobbleGoal;
    Servo goalGrab;


    BNO055IMU imu, imu1;
    Orientation angles, angles1;

    double basePower = 0.2;
    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone = false;
    double power_multiplier;
    double armPosition;
    double angleToTurn;
    double liftPosition;
    private static final double REV_CORE_HEX_TICKS_PER_INCH = 47.127;
    private static final double LIFT_JUMP_RESOLUTION = 1;
    private static final double LIFT_MAX_INCH = 16;

    private static final double LIFT_NON_SLIP_POWER = 0.2;
    //    private static final double ARM_INCH_PER_MS = 1471.724;
    private static final double ARM_INCH_PER_MS = 735;


    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor


    public void teleopInitFn() {

        telemetry.addData("Init: start ", "");

        //leftFront, leftRear, rightRear, rightFront
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        launch = hardwareMap.dcMotor.get("Launch");
        intake = hardwareMap.dcMotor.get("Intake");
        ramp = hardwareMap.dcMotor.get("Ramp");
        wobbleGoal = hardwareMap.dcMotor.get("WobbleGoal");
        goalGrab = hardwareMap.servo.get("Grabber");

        armPosition = 0;
        //sideArm = hardwareMap.servo.get("sideArm");

        //extend = hardwareMap.dcMotor.get("extend");

        strafing = false;

        rightFront.setMode(RUN_WITHOUT_ENCODER);
        rightBack.setMode(RUN_WITHOUT_ENCODER);
        leftFront.setMode(RUN_WITHOUT_ENCODER);
        leftBack.setMode(RUN_WITHOUT_ENCODER);

        power_multiplier = 1;


        angleToTurn = 30;

        double driveSpeed = 0;
        leftBack.setPower(driveSpeed);
        leftFront.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
    }


    public void stopWheels() {
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setPower(0);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setPower(0);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setPower(0);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0);

    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void vectorCombine(double x, double y, double turn) {
        telemetry.addData("x:", x);
        telemetry.addData("y:", y);
        telemetry.update();

        double a = (x + y) + basePower * ((x + y) / Math.abs(x + y));
        double b = (y - x) + basePower * ((y - x) / Math.abs(y - x));
        double c = -(y - x) - basePower * ((y - x) / Math.abs(y - x));
        double d = -(x + y) - basePower * ((x + y) / Math.abs(x + y));

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE); //changed for 2020 config
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setPower(a + turn);
        rightFront.setPower(b - turn);
        leftBack.setPower(c - turn);
        rightBack.setPower(d + turn);

    }

    public void launchRing(){
        launch.setPower(1);
    }

    public void closeGrabber(){
        goalGrab.setPosition(1);
    }

    public void openGrabber(){
        goalGrab.setPosition(0);
    }

//    public void setLiftPosition(double position){
//        lift.setMode(RUN_WITHOUT_ENCODER);
//        double margin = 0.25* REV_CORE_HEX_TICKS_PER_INCH;
//        telemetry.addData("TargetLift Value", position);
//
//        while ((lift.getCurrentPosition() < (position - margin)) && !isStopRequested()){
//            idle();
//            lift.setPower(1);
//            lift_assist.setPower(1);
//            position-=1; //to avoid getting stuck at top position
//        }
//        while ((lift.getCurrentPosition() > (position + margin)) && !isStopRequested()) {
//            idle();
//            lift.setPower(-0.25);
//            lift_assist.setPower(-0.25);
//
//        }
//        double noSlipPower = LIFT_NON_SLIP_POWER + (position / (REV_CORE_HEX_TICKS_PER_INCH * 32));
//        lift.setPower(noSlipPower);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift_assist.setPower(noSlipPower);
//        lift_assist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//    }
//    public void liftInch(double inches) {
//
//        lift.setMode(RUN_WITHOUT_ENCODER);
//        lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        double ticks = inches * REV_CORE_HEX_TICKS_PER_INCH;
//
//        if ((Math.abs(lift.getCurrentPosition()) < Math.abs(ticks))) {
//            while ((Math.abs(lift.getCurrentPosition()) < Math.abs(ticks)) && !isStopRequested()) {
//                idle();
//                lift.setPower(1.0);
//                lift_assist.setPower(1.0);
//            }
//        } else {
//            while ((Math.abs(lift.getCurrentPosition()) > Math.abs(ticks)) && !isStopRequested()) {
//                idle();
//                lift.setPower(-1.0);
//                lift_assist.setPower(-1.0);
//            }
//        }
//
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift.setPower(LIFT_NON_SLIP_POWER);
//        lift_assist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift_assist.setPower(LIFT_NON_SLIP_POWER);
//
//    }

//    public void armExtended(double inches){
//        long currentTime = System.currentTimeMillis();
//        double targetTime = Math.abs(inches) * ARM_INCH_PER_MS + currentTime;
//        while((currentTime < targetTime ) && !isStopRequested()){
//            idle();
//            currentTime = System.currentTimeMillis();
//            extend.setPower(1);
//        }
//        extend.setPower(0);
//    }
//
//    public void grabCollection() {
//        grab_front.setPosition(1);
//        grab_back.setPosition(0.5);
//    }
//
//    public void closeGrabber() {
//        grab_front.setPosition(0.2);
//        grab_back.setPosition(0.5);
//    }

    @Override
    public void runOpMode() {

        double liftTarget = 0;
        teleopInitFn();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            idle();

            float forward = -1 * gamepad1.right_stick_y;
            double turn_component = gamepad1.left_stick_x*0.7;
            double x_component = gamepad1.right_stick_x;
            double y_component = -1 * gamepad1.right_stick_y;

            if (gamepad1.left_bumper && Math.abs(forward) > 0.1) {
                //right turn
                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);  //changed
                leftBack.setDirection(DcMotorSimple.Direction.REVERSE);   //changed
                rightFront.setDirection(DcMotorSimple.Direction.REVERSE); //default
                rightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
                rightFront.setPower(forward * power_multiplier);
                rightBack.setPower(forward * power_multiplier);
                leftFront.setPower(forward * power_multiplier);
                leftBack.setPower(forward * power_multiplier);

            } else if (gamepad1.right_bumper && Math.abs(forward) > 0.1) {
                //left turn
                leftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                leftBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                rightFront.setDirection(DcMotorSimple.Direction.FORWARD); //changed
                rightBack.setDirection(DcMotorSimple.Direction.FORWARD);  //changed
                rightFront.setPower(forward * power_multiplier);
                rightBack.setPower(forward * power_multiplier);
                leftFront.setPower(forward * power_multiplier);
                leftBack.setPower(forward * power_multiplier);

            } else if (Math.abs(x_component) > 0.1 || (Math.abs(y_component)>0.1)) {
                vectorCombine(x_component, y_component, turn_component);
            } else {
                stopWheels();
            }

            if (gamepad1.dpad_up){
                openGrabber();
            }

            if (gamepad1.dpad_down){
                closeGrabber();
            }

            while (gamepad1.y){
                launchRing();
            }

//            if (gamepad2.right_bumper) {
//                grab_back.setPosition(0);
//                grab_front.setPosition(0.5);
//            }
//
//            if (gamepad2.left_bumper) {
//                grab_back.setPosition(0);
//                grab_front.setPosition(0);
//            }
//
//            if (gamepad1.x) {  //position up
//                foundation.setPosition(0);
//            }

//            if (gamepad1.y) {   //position down
//                foundation.setPosition(1);
//            }
//
//            if (lift.getCurrentPosition() < LIFT_MAX_INCH * REV_CORE_HEX_TICKS_PER_INCH && gamepad2.y) {
//                liftTarget = liftTarget + (LIFT_JUMP_RESOLUTION * REV_CORE_HEX_TICKS_PER_INCH);
//            }
//
//            if (gamepad2.x) {
//                liftTarget = liftTarget - (LIFT_JUMP_RESOLUTION * REV_CORE_HEX_TICKS_PER_INCH);
//            }
//
//            if(Math.abs(liftTarget) > 5) {
//                setLiftPosition(Math.abs(liftTarget));
//            }
        }

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        launch.setPower(0);
        wobbleGoal.setPower(0);
        intake.setPower(0);
        //telemetry.addData("lift encoder value", lift.getCurrentPosition());


    }
}