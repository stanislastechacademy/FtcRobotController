/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Locale;

@Autonomous(name = "34pointsRight", group = "Final")
@Disabled
public class OneplusxtestingTHREE extends LinearOpMode
{
    private DcMotor armMotor1;
    private DcMotor armMotor2;
    private Servo intakeServo;

    public void armMovement(int armTicks) throws InterruptedException {
        armMotor1.setTargetPosition(armTicks);
        armMotor2.setTargetPosition(armTicks);
        armMotor1.setPower(0.5);
        armMotor2.setPower(0.5);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor1.isBusy()) {
            idle();
        }
    }
    public void servoPositioning(double servoPosition) {intakeServo.setPosition(servoPosition);}
    public void trystop(){
        if(!opModeIsActive() && isStopRequested()){
            return;
        }
    }
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double TagSize = 0.166;

    // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException{
        armMotor1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armMotor2 = hardwareMap.get(DcMotor.class, "armmotor2");

        intakeServo = hardwareMap.get(Servo.class, "Intake");

        armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(TagSize, fx, fy, cx, cy);

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Trajectory RightPark1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(0, 6))
                .build();

        Trajectory RightPark2 = drivetrain.trajectoryBuilder(RightPark1.end())
                .lineTo(new Vector2d(26, 6))
                .build();

        Trajectory MiddlePark = drivetrain.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .lineTo(new Vector2d(4,0))
                .build();

        Trajectory LeftPark1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(0, 6))
                .build();

        Trajectory LeftPark2 = drivetrain.trajectoryBuilder(LeftPark1.end())
                .lineTo(new Vector2d(-16, 6))
                .build();

        Trajectory FirstBloodPartOne1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(0, 2.5))
                .build();

        Trajectory FirstBloodPartOne2 = drivetrain.trajectoryBuilder(FirstBloodPartOne1.end())
                .lineTo(new Vector2d(26,2.5))
                .build();

        Trajectory FirstBloodPartOne3 = drivetrain.trajectoryBuilder(FirstBloodPartOne2.end())
                .lineTo(new Vector2d(26, 13))
                .build();

        Trajectory FirstBloodPartTwo = drivetrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(-4,0))
                .build();

        Trajectory FirstBloodPartThree = drivetrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(5,0))
                .build();

        Trajectory AlmostMoreBlood = drivetrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(-3, 0))
                .build();

        Trajectory MoreBloodPartOne1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(0,-8))
                .build();

        Trajectory MoreBloodPartOne2 = drivetrain.trajectoryBuilder(MoreBloodPartOne1.end())
                .lineTo(new Vector2d(26,-7))
                .build();

        TrajectorySequence MoreBloodPartTwo = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(23,-1.5))
                .addDisplacementMarker(() -> {servoPositioning(0.3);})
                .lineTo(new Vector2d(22,-1.5))
                .addDisplacementMarker(()  ->{
                    try {
                        armMovement(1400);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .lineTo(new Vector2d(20,-1.5))
                .turn(Math.toRadians(10))
                .build();

        Trajectory MoreBloodPartThree1 = drivetrain.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .lineTo(new Vector2d(-17,0))
                .build();

        Trajectory MoreBloodPartThree2 = drivetrain.trajectoryBuilder(MoreBloodPartThree1.end())
                .lineTo(new Vector2d(-17,-7.5))
                .build();

        Trajectory MoreBloodPartThree3 = drivetrain.trajectoryBuilder(MoreBloodPartThree2.end())
                .lineTo(new Vector2d(-21,-7.5))
                .build();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        servoPositioning(0.3);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        drivetrain.followTrajectory(FirstBloodPartOne1);
        trystop();
        drivetrain.followTrajectory(FirstBloodPartOne2);
        trystop();
        drivetrain.followTrajectory(FirstBloodPartOne3);
        trystop();
        drivetrain.followTrajectory(FirstBloodPartTwo);
        trystop();
        armMovement(1600);
        trystop();
        drivetrain.followTrajectory(FirstBloodPartThree);
        trystop();
        armMovement(1000);
        trystop();
        servoPositioning(0.5);
        trystop();
        drivetrain.followTrajectory(AlmostMoreBlood);
        trystop();
        armMovement(1300);
        trystop();
        drivetrain.followTrajectory(MoreBloodPartOne1);
        trystop();
        drivetrain.followTrajectory(MoreBloodPartOne2);
        trystop();
        drivetrain.turn(Math.toRadians(77));
        trystop();
        armMovement(500);
        trystop();
        drivetrain.followTrajectorySequence(MoreBloodPartTwo);
        trystop();
        drivetrain.followTrajectory(MoreBloodPartThree1);
        trystop();
        drivetrain.followTrajectory(MoreBloodPartThree2);
        trystop();
        drivetrain.followTrajectory(MoreBloodPartThree3);
        trystop();
        armMovement(700);
        trystop();
        servoPositioning(0.5);
        trystop();
        armMovement(100);
        trystop();

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drivetrain.followTrajectory(LeftPark1);
            trystop();
            drivetrain.followTrajectory(LeftPark2);
            trystop();
            telemetry.addLine(String.format(Locale.ENGLISH,"Going for the left!"));
        }else if(tagOfInterest.id == MIDDLE){
            drivetrain.followTrajectory(MiddlePark);
            trystop();
            telemetry.addLine(String.format(Locale.ENGLISH,"Going for the middle!"));
        }else if(tagOfInterest.id == RIGHT){
            drivetrain.followTrajectory(RightPark1);
            trystop();
            drivetrain.followTrajectory(RightPark2);
            trystop();
            telemetry.addLine(String.format(Locale.ENGLISH,"Going for the right!"));
        }
    }

    void tagToTelemetry(@NonNull AprilTagDetection detection)
    {
        telemetry.addLine(String.format(Locale.ENGLISH,"Detected tag ID=%d", detection.id));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}