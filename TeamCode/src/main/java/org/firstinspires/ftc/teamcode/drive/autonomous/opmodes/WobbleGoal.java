package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;

@Autonomous (name = "WobbleGoal")
public class WobbleGoal extends Autonomous2021{
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrivetrain(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(new Pose2d(-qFieldLength * (2.0/3.0), -qFieldLength/3, 0.0));

        waitForStart();

        if (isStopRequested()) return;

        // intake = hardwareMap.get(DcMotorEx.class, "Intake");
        //grabber = hardwareMap.get(DcMotorEx.class, "wobbleGrab");
        //launch = hardwareMap.get(DcMotorEx.class , "Launch");

        DepositWobbleGoal(drive.getPoseEstimate());
        Park(drive.getPoseEstimate());

        drive.setMotorPowers(0,0,0,0);
    }
}
