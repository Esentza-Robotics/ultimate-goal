package Esentza;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.TuningController;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name="Auto", group="yes")

public class Autonomy extends LinearOpMode {

    public static OpenCvCamera camera;

    public void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new CameraTest.RingDeterminationPipeline();
        camera.setPipeline(pipeline);

        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(() ->
        {
            camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            FtcDashboard.getInstance().startCameraStream(camera, 0);
        });
    }

    public static CameraTest.RingDeterminationPipeline pipeline;

    public static double INITIAL_LENGTH = 25;
    public static double SERVO_SPEED = 0.2;
    public static double SERVO_SPEED_TOWER = 0.225;
    public static double FLYWHEEL_POWER = 3686;
    public static double FLYWHEEL_POWER_ERROR = 3500;
    @Override
    public void runOpMode() {
        Output out = new Output(hardwareMap);
        Intake in = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initCam();

        Trajectory trajectory0forAllPaths = drive.trajectoryBuilder(new Pose2d())
                //.back(INITIAL_LENGTH)
                .lineTo(new Vector2d(-62, 13))
                .addDisplacementMarker(INITIAL_LENGTH, () ->
                {
                    in.setPosition(1);
                    out.setVelocity(FLYWHEEL_POWER);
                })
                .addDisplacementMarker(() -> out.turnOnServo(SERVO_SPEED))
                .build();


        Trajectory trajectory1forAllPaths = drive.trajectoryBuilder(trajectory0forAllPaths.end())
                .lineTo(new Vector2d(-62, 22))
                .addDisplacementMarker(() ->
                        out.turnOnServo(SERVO_SPEED))
                .build();

        Trajectory trajectory2forAllPaths = drive.trajectoryBuilder(trajectory1forAllPaths.end())
                .lineTo(new Vector2d(-62, 29.5))
                .addDisplacementMarker(() ->
                        out.turnOnServo(SERVO_SPEED))
                .build();


        Trajectory trajectory3forNoRing = drive.trajectoryBuilder(trajectory2forAllPaths.end())
                .addDisplacementMarker(1, ()->
                {
                    out.setUpPosition(0);
                    out.setUpPosition(1);
                })
                .lineToLinearHeading(new Pose2d(-80, 39, Math.toRadians(0)))
                .addDisplacementMarker(()-> out.setPositionClaw(1))
                .build();


        Trajectory trajectory4forNoRing = drive.trajectoryBuilder(trajectory3forNoRing.end())
                .lineTo(new Vector2d(-80, 15))
                .build();

        Trajectory trajectory5forNoRing = drive.trajectoryBuilder(trajectory4forNoRing.end())
                .lineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(-17)))
                .addDisplacementMarker(()->out.setPositionClaw(-1))
                .build();

        Trajectory trajectory6forNoRing = drive.trajectoryBuilder(trajectory5forNoRing.end())
                .lineToLinearHeading(new Pose2d(-62, 45, Math.toRadians(45)))
                .addDisplacementMarker(()->out.setPositionClaw(1))
                .build();

        Trajectory trajectory7forNoRing = drive.trajectoryBuilder(trajectory6forNoRing.end())
                .lineTo(new Vector2d(-75, 0))
                .build();

        Trajectory trajectory3forOneRing = drive.trajectoryBuilder(trajectory2forAllPaths.end())
                .addDisplacementMarker(1, ()->
                {
                    out.setUpPosition(0);
                    out.setUpPosition(1);
                })
                .lineTo(new Vector2d(-100, 7))
                .addDisplacementMarker(()-> out.setPositionClaw(1))
                .build();

        Trajectory trajectory4forOneRing = drive.trajectoryBuilder(trajectory3forOneRing.end())
                .lineTo(new Vector2d(-100, -10))
                .build();

        Trajectory trajectory5forOneRing = drive.trajectoryBuilder(trajectory4forOneRing.end())
                .lineToConstantHeading(new Vector2d(-55, 25))
                .addDisplacementMarker(()->in.setVelocity(10000, DcMotorSimple.Direction.FORWARD))
                .splineToConstantHeading(new Vector2d(-27, 10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-19, 21), Math.toRadians(0))
                .addDisplacementMarker(()-> out.setPositionClaw(-1))
                .build();

        Trajectory trajectory6forOneRing = drive.trajectoryBuilder(trajectory5forOneRing.end())
                .addDisplacementMarker(()->
                {
                    out.setVelocity(4800);
                    out.setUpPosition(0);
                })
                .lineTo(new Vector2d(-53, 48))
                .addDisplacementMarker(() ->
                        out.turnOnServo(0.3))
                .build();


        Trajectory trajectory7forOneRing = drive.trajectoryBuilder(trajectory6forOneRing.end())
                .addDisplacementMarker(()-> {
                    out.setVelocity(0);
                    out.turnOnServo(0);
                })
                .lineToLinearHeading(new Pose2d(-88, 20, Math.toRadians(90)))
                .addDisplacementMarker(10, ()->out.setUpPosition(1))
                .build();

        Trajectory trajectory8forOneRing = drive.trajectoryBuilder(trajectory7forOneRing.end())
                .addDisplacementMarker(()-> out.setUpPosition(0))
                .lineTo(new Vector2d(-72, 15))
                .addDisplacementMarker(()->drive.turn(Math.toRadians(90)))
                .build();


        Trajectory trajectory3forFourRings = drive.trajectoryBuilder(trajectory2forAllPaths.end())
                .lineTo(new Vector2d(-121, 40))
                .addDisplacementMarker(10,() ->
                {
                    out.setUpPosition(0);
                    out.setUpPosition(1);
                })
                .addDisplacementMarker(() -> out.setPositionClaw(1))
                .build();

        Trajectory trajectory4forFourRings = drive.trajectoryBuilder(trajectory3forFourRings.end())
                .addDisplacementMarker(() ->
                {
                    out.setUpPosition(0);
                    in.setVelocity(10000, DcMotorSimple.Direction.FORWARD);
                    out.setVelocity(4800);
                })
                .lineToLinearHeading(new Pose2d(-40, 21, Math.toRadians(-17)))
                .addDisplacementMarker(() -> out.setUpPosition(1))
                .build();

        Trajectory trajectory5forFourRings = drive.trajectoryBuilder(trajectory4forFourRings.end())
                .lineToLinearHeading(new Pose2d(-43, 20, Math.toRadians(-17)))
                .addDisplacementMarker(() -> out.turnOnServo(SERVO_SPEED_TOWER))
                .build();

        Trajectory trajectory6forFourRings = drive.trajectoryBuilder(trajectory5forFourRings.end())
                .addDisplacementMarker(() -> out.turnOnServo(0))
                .lineTo(new Vector2d(-20, 20))
                .addDisplacementMarker(() -> out.setPositionClaw(0))
                .build();

        Trajectory trajectory7forFourRings = drive.trajectoryBuilder(trajectory6forFourRings.end())
                .lineToLinearHeading(new Pose2d(-43, 20, Math.toRadians(-17)))
                .addDisplacementMarker(() -> out.turnOnServo(SERVO_SPEED_TOWER))
                .build();

        Trajectory trajectory8forFourRings = drive.trajectoryBuilder(trajectory7forFourRings.end())
                .lineTo(new Vector2d(-121, 34))
                .addDisplacementMarker(45, () -> {
                    out.setUpPosition(1);
                    in.setVelocity(0, DcMotor.Direction.FORWARD);
                    out.turnOnServo(0);
                })
                .addDisplacementMarker(() -> out.setPositionClaw(1))
                .build();

        Trajectory trajectory9forFourRings = drive.trajectoryBuilder(trajectory8forFourRings.end())
                .addDisplacementMarker(()-> out.setUpPosition(0))
                .lineTo(new Vector2d(-70, 20))
                .build();

        drive.setPoseEstimate(new Pose2d());

        out.setPositionClaw(0);

        waitForStart();

        CameraTest.RingDeterminationPipeline.RingPosition pos = pipeline.getNumberOfRings();

        if (pos != CameraTest.RingDeterminationPipeline.RingPosition.NONE)
            FLYWHEEL_POWER = 3720;

        telemetry.addData("Number of Rings", pos);
        telemetry.update();
        
        drive.followTrajectory(trajectory0forAllPaths);

        while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(FLYWHEEL_POWER_ERROR))
        {
            telemetry.addData("First Ring", "Processing");
            telemetry.update();
        }
        out.turnOnServo(0);

        drive.followTrajectory(trajectory1forAllPaths);
        while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(FLYWHEEL_POWER_ERROR))
        {
            telemetry.addData("Second Ring", "Processing");
            telemetry.update();
        }
        out.turnOnServo(0);

        drive.followTrajectory(trajectory2forAllPaths);
        while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(FLYWHEEL_POWER_ERROR))
        {
            telemetry.addData("Third Ring", "Processing");
            telemetry.update();
        }
        out.turnOnServo(0);

        if (pos == CameraTest.RingDeterminationPipeline.RingPosition.NONE) {

            drive.followTrajectory(trajectory3forNoRing);

            sleep(500);

            drive.followTrajectory(trajectory4forNoRing);

            drive.followTrajectory(trajectory5forNoRing);

            sleep(1000);

            drive.followTrajectory(trajectory6forNoRing);

            sleep(800);

            drive.followTrajectory(trajectory7forNoRing);
        }

        else if (pos == CameraTest.RingDeterminationPipeline.RingPosition.ONE) {

            drive.followTrajectory(trajectory3forOneRing);

            sleep(500);

            drive.followTrajectory(trajectory4forOneRing);

            drive.followTrajectory(trajectory5forOneRing);

            sleep(1000);

            drive.followTrajectory(trajectory6forOneRing);

            sleep(1000);

            drive.followTrajectory(trajectory7forOneRing);

            out.setPositionClaw(1);

            sleep(750);

            drive.followTrajectory(trajectory8forOneRing);
        }

        else
        {
            drive.followTrajectory(trajectory3forFourRings);

            drive.followTrajectory(trajectory4forFourRings);

            in.setVelocity(10000, DcMotorSimple.Direction.FORWARD);

            sleep(800);

            out.setVelocity(4800);

            out.turnOnServo(0);

            drive.followTrajectory(trajectory5forFourRings);

            sleep(750);

            drive.followTrajectory(trajectory6forFourRings);

            sleep(750);

            drive.followTrajectory(trajectory7forFourRings);

            sleep(1000);

            drive.followTrajectory(trajectory8forFourRings);

            sleep(600);

            drive.followTrajectory(trajectory9forFourRings);

            out.setUpPosition(1);
        }
    }
}

