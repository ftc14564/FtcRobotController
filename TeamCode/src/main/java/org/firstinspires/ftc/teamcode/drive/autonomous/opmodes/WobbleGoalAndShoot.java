package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

import static java.lang.Math.atan;

    @Autonomous(name = "WobbleGoalAndShoot")
    public class WobbleGoalAndShoot extends Autonomous2021{

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

            trajectories.addAll(oneWobbleGoalAndShoot(trajectories.get(trajectories.size()-1).end().vec()));

            drive.followTrajectory(trajectories.get(1)) ;

            if (RedAlliance == (state == 1)){
                drive.turn(Math.toRadians(180.0));
            }
            dropOffWobbleGoal();

            drive.followTrajectory(trajectories.get(2));

            drive.turn(Math.toRadians(-110.0)); //theoretically 90 deg
            shootRings();

            drive.followTrajectory(trajectories.get(3));

            drive.setMotorPowers(0,0,0,0);
        }
    }

