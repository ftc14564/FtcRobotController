package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;
import org.opencv.core.Mat;

import java.util.ArrayList;

import static java.lang.Math.atan;

@Autonomous (name = "WobbleGoal")
public class WobbleGoal extends Autonomous2021{

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

        trajectories.addAll(detectStack());

        drive.followTrajectory(trajectories.get(0));

        drive.turn(FP*atan(1.0/2.0)+ Math.toRadians(90.0));
        state = getRingState();
        drive.turn(-Math.toRadians(90.0));

        trajectories.addAll(doRingStackAndWobbleGoal());

        trajectories.addAll(oneWobbleGoal(trajectories.get(trajectories.size()-1).end().vec()));

        drive.followTrajectory(trajectories.get(1));
        if (RedAlliance == (state == 1)){
            drive.turn(Math.toRadians(180.0));
        }
        dropOffWobbleGoal();
        drive.followTrajectory(trajectories.get(2));

        drive.setMotorPowers(0,0,0,0);
    }
}
