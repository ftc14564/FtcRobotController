package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

import static java.lang.Math.atan;

@Autonomous
public class PowershotPreload extends Autonomous2021{

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        startPose = new Pose2d(startPose.getX(), startPose.getY(), Math.toRadians(-180.0));
        drive.setPoseEstimate(startPose);

        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

        trajectories.addAll(shootPowershot());

        drive.followTrajectory(trajectories.get(0));

        shootRings();

        drive.followTrajectory(trajectories.get(1));

        state = getRingState();

        trajectories.addAll(doRingStackAndWobbleGoalPreload());

        trajectories.addAll(oneWobbleGoal(trajectories.get(trajectories.size()-1).end().vec()));

        drive.turn(Math.toRadians(180));

        drive.followTrajectory(trajectories.get(2));
        if (RedAlliance == (state == 1)){
            drive.turn(Math.toRadians(180.0));
        }
        dropOffWobbleGoal();
        drive.followTrajectory(trajectories.get(3));

        drive.setMotorPowers(0,0,0,0);
    }
}
