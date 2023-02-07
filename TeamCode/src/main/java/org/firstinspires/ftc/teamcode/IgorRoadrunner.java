package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "IgorRoadrunner", group = "Concept")
public class IgorRoadrunner extends LinearOpMode{

    private DefaultFunctions defaultFunctions;

    /* input intergers beschrijving. Output: real
     * beschrijf kort en krachtig de werking van de functoe
     * @param servoPosition
     */

    @Override
    public  void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

//        initAutonomous();
        /*
        defaultFunctions = new DefaultFunctions();
        defaultFunctions.initAutonomous(hardwareMap, this);
        defaultFunctions.driveRight(7);
        waitForStart();
        servoPositioning(0.3);
*/
        /*
        TrajectorySequence PreloadedConePlusFirstCone = drivetrain.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(0)))
                .lineTo(new Vector2d(36, 0))
                .lineTo(new Vector2d(34, 0))
                .waitSeconds(1)
                .lineTo(new Vector2d(36, 0))
                .lineTo(new Vector2d(36, -12))
                .lineTo(new Vector2d(56, -12))
                .waitSeconds(1)
                .build();

        TrajectorySequence ConeStack = drivetrain.trajectorySequenceBuilder(new Pose2d(56, -12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47.5, -12, Math.toRadians(270)))
                .lineTo(new Vector2d(47.5, -14))
                //.waitSeconds(1)
                .lineTo(new Vector2d(47.5, -10))
                .lineToLinearHeading(new Pose2d(56, -12, Math.toRadians(0)))
                //.waitSeconds(1)
                .build();
        */
        TrajectorySequence Park = drivetrain.trajectorySequenceBuilder(new Pose2d(0, -0, Math.toRadians(0)))
                .lineTo(new Vector2d(36, -36))
                .lineTo(new Vector2d(24, -36))
                .build();

        TrajectorySequence test = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(1, 0, Math.toRadians(90)))
                .build();

        TrajectorySequence TurnTest = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(48,0))
                .lineTo(new Vector2d(24, 0))
                .waitSeconds(5)
                .lineTo(new Vector2d(0,0))
                .turn(Math.toRadians(90))
                .build();

        drivetrain.followTrajectorySequence(TurnTest);
        drivetrain.followTrajectorySequence((Park));
    }
}