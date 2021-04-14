package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

//import static org.firstinspires.ftc.teamcode.BuildConfig.DEBUG;

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
    double basePower = 0.6;
    double turnPower = 0.2;
    double indivTurnPower = 0.5;


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

    public void vectorCombineRoadrunner(double x, double y, double turn, double indivTurn) {
        double a = y - x;
        double b = y + x;

        double scalar = basePower /  Math.max(Math.abs(a), Math.abs(b));

        if(a!=0 || b!=0) {
            a *= scalar; b *= scalar;
            drive.setMotorPowers(a + turn, b + turn, a - turn, b - turn);
        }else{
            drive.setMotorPowers(indivTurn, indivTurn, -indivTurn, -indivTurn);
        }
    }


    @Override
    public void runOpMode() {

        teleopInitFn();

        waitForStart();

        while (opModeIsActive()) {
            double turn = 0.0;
            double indivTurn = 0.0;
            if (gamepad1.left_bumper) {
               turn += turnPower;
               indivTurn += indivTurnPower;
            }
            if (gamepad1.right_bumper){
               turn -= turnPower;
               indivTurn -= indivTurnPower;
            }

            vectorCombineRoadrunner(gamepad1.left_stick_x, gamepad1.left_stick_y, turn, indivTurn);

            if (gamepad2.left_trigger > 0.1) {
                in_front.setPower(-1);
                in_back.setPower(-1);
            }
            if (gamepad2.right_trigger > 0.1) {
                in_front.setPower(1);
                in_back.setPower(1);
            }

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
            if (gamepad2.dpad_left) {
                wobbleGrab.setPosition(0);

            }
            if (gamepad2.dpad_right) {
                wobbleGrab.setPosition(1);

            }

            wobbleGoal.setPower(0);
            in_front.setPower(0);
            in_back.setPower(0);
            telemetry.addData("shooter power: ", shooter.getPower());
            telemetry.update();
            drive.update(); //comment this out during real runs
        }
    }
    }














