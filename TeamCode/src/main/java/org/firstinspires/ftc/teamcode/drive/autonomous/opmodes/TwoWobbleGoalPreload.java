package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static java.lang.Math.atan;

@Autonomous
public class TwoWobbleGoalPreload extends Autonomous2021{
    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

        trajectories.addAll(detectStack());

        drive.followTrajectory(trajectories.get(0));

        drive.turn(FP*atan(1.0/2.0)+ Math.toRadians(90.0));
        state = getRingState();
        drive.turn(-drive.getPoseEstimate().getHeading()+Math.toRadians(-15.0));

        trajectories.addAll(doRingStackAndWobbleGoalPreload());

        trajectories.addAll(pickupWobbleGoal(trajectories.get(trajectories.size()-1).end()));

        drive.followTrajectory(trajectories.get(1));
        if (RedAlliance == (state == 1)){
            drive.turn(Math.toRadians(180.0));
        }
        dropOffWobbleGoal();
        drive.followTrajectory(trajectories.get(2));
        drive.turn(PI/2-drive.getPoseEstimate().getHeading());
        drive.followTrajectory(trajectories.get(3));
        pickUpWobbleGoal();
        drive.turn(PI);
        trajectories.addAll(doWobbleGoalAfterPickup(drive.getPoseEstimate()));
        trajectories.addAll(oneWobbleGoal(trajectories.get(trajectories.size()-1).end().vec()));
        drive.followTrajectory(trajectories.get(4));
        dropOffWobbleGoal();
        drive.followTrajectory(trajectories.get(5));
        drive.setMotorPowers(0,0,0,0);
    }
}
