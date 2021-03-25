package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;

@Autonomous (name = "Park")
public class Park extends Autonomous2021{
    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        drive.followTrajectory(drive.trajectoryBuilder(startPose)
        .lineTo(new Vector2d(sq/2, startPose.getY()))
        .build());

        drive.setMotorPowers(0,0,0,0);
    }
}
