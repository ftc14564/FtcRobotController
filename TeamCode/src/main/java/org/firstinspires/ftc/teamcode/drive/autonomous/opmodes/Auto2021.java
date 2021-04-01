package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;


public class Auto2021 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivetrain drive = new MecanumDrivetrain(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "Intake");
        DcMotorEx grabber = hardwareMap.get(DcMotorEx.class, "WobbleGoal");
        DcMotorEx ramp = hardwareMap.get(DcMotorEx.class, "Ramp");


        grabber.setPower(-0.75);

        intake.setPower(1);

        ramp.setPower(1);

        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj);


        sleep(1000);

        drive.turn(Math.toRadians(90));

        sleep(500);


        intake.setVelocity(0);

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(-60, 50), Math.toRadians(180))
                .build());
        grabber.setPower(0.5);

        sleep(200);

        grabber.setPower(0);

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(40)
                .build());
    }
}
