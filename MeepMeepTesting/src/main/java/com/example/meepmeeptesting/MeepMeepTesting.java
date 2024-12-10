package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Pose2d pos1 = new Pose2d(66,-35,0);
        Pose2d pos2 = new Pose2d(66,35,0);
        Pose2d pos3 = new Pose2d(-70,-35,0);
        Pose2d pos4 = new Pose2d(-70,35,0);

        // Initialize MeepMeep with the simulation window size
        MeepMeep meepMeep = new MeepMeep(700);

        // Create a robot entity and set its constraints
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        // Define the trajectory sequence using spline paths
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(40,45, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-40,20, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(20,-55, Math.toRadians(90)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(0)) // Linear spline to heading
                                .build()
                );

        // Set up the background theme and simulation settings
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true) // Enable dark mode
                .setBackgroundAlpha(0.95f) // Set background transparency
                .addEntity(myBot) // Add the robot to the simulation
                .start(); // Start the simulation
    }
}
