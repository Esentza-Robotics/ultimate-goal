package Esentza;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.TuningController;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(name="Red1stPosition", group="yes")

public class Red1stPosition extends LinearOpMode {

    public static OpenCvCamera camera;

    private ElapsedTime runtime = new ElapsedTime();

    public void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new CameraTest.RingDeterminationPipeline();
        camera.setPipeline(pipeline);

        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(() ->
        {
            camera.startStreaming(800, 600, OpenCvCameraRotation.UPSIDE_DOWN);
            FtcDashboard.getInstance().startCameraStream(camera, 0);
        });
    }

    public static CameraTest.RingDeterminationPipeline pipeline;

    public static double POSITION_FOR_ANGLE = 0.41;

    public static double POSITION_FOR_POWERSHOTS = 0.43;

    public static double POSITION_UP_WOBBLE_ARM = 0.5;
    public static double POSITION_DOWN_WOBBLE_ARM = 1;

    public static double POSITION_CLOSED_FIRST_CLAW = 0;
    public static double POSITION_OPEN_FIRST_CLAW = 0.5;

    public static double POSITION_CLOSED_SECOND_CLAW = 0;
    public static double POSITION_OPEN_SECOND_CLAW = 0.75;

    public static double INITIAL_LENGTH = 10;
    public static double SERVO_SPEED = -0.2;

    public static double FLYWHEEL_POWER = 4200;
    public static double FLYWHEEL_POWER_ERROR = 3800;

    public static double INTAKE_POWER = 10000;

    @Override
    public void runOpMode() {
        CameraStorage.x = 410;
        CameraStorage.y = 260;
        Output out = new Output(hardwareMap);
        Intake in = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        initCam();

        out.setPositionClaw(POSITION_CLOSED_FIRST_CLAW, POSITION_CLOSED_SECOND_CLAW);

        out.setUpPosition(POSITION_UP_WOBBLE_ARM);

        drive.setPoseEstimate(new Pose2d());

        Trajectory trajectory0forAllPaths = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(61.6, 24))
                .addDisplacementMarker(INITIAL_LENGTH, () -> {
                    turret.setPositionForAngleServo(POSITION_FOR_POWERSHOTS);
                    out.setVelocity(FLYWHEEL_POWER);
                })
                .addDisplacementMarker(() -> {
                    out.setPusherSpeed(SERVO_SPEED);
                    in.setVelocity(INTAKE_POWER, DcMotorSimple.Direction.FORWARD);
                })
                .build();

        Trajectory trajectory1forAllPaths = drive.trajectoryBuilder(trajectory0forAllPaths.end())
                .lineToLinearHeading(new Pose2d(61.6, 14, Math.toRadians(0)))
                .addDisplacementMarker(() -> out.setPusherSpeed(SERVO_SPEED))
                .build();


        Trajectory trajectory2forAllPaths = drive.trajectoryBuilder(trajectory1forAllPaths.end())
                .lineToLinearHeading(new Pose2d(61.6, 4, Math.toRadians(0)))
                .addDisplacementMarker(() -> out.setPusherSpeed(SERVO_SPEED))
                .build();

        Trajectory trajectory3forNoRings = drive.trajectoryBuilder(trajectory2forAllPaths.end())
                .lineTo(new Vector2d(103, -32))
                .addDisplacementMarker(() -> out.setUpPosition(POSITION_DOWN_WOBBLE_ARM))
                .build();

        Trajectory trajectory4forNoRings = drive.trajectoryBuilder(trajectory3forNoRings.end())
                .lineTo(new Vector2d(110, -32))
                .addDisplacementMarker(4, () -> out.setPositionClaw(POSITION_CLOSED_FIRST_CLAW, POSITION_CLOSED_SECOND_CLAW))
                .addDisplacementMarker(() -> {
                    out.setUpPosition(POSITION_UP_WOBBLE_ARM);
                    drive.turn(Math.toRadians(45));
                })
                .build();


        Trajectory trajectory5forNoRings = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(45)))
                .lineTo(new Vector2d(0, 55))
                .addDisplacementMarker(() -> {
                    drive.turn(Math.toRadians(-45));
                    in.setVelocity(0, DcMotorSimple.Direction.FORWARD);
                })
                .build();

        Trajectory trajectory6forNoRings = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-55, -9))
                .addDisplacementMarker(() -> {
                    out.setVelocity(FLYWHEEL_POWER);
                    drive.turn(Math.toRadians(-25));
                })
                .build();

        Trajectory trajectory7forNoRings = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .build();


        Trajectory trajectory3forOneRing = drive.trajectoryBuilder(trajectory2forAllPaths.end())
                .lineTo(new Vector2d(120, -10))
                .addDisplacementMarker(() -> out.setUpPosition(POSITION_DOWN_WOBBLE_ARM))
                .build();


        Trajectory trajectory4forOneRing = drive.trajectoryBuilder(trajectory3forOneRing.end())
                .lineTo(new Vector2d(120, -32))
                .addDisplacementMarker(4, () -> out.setPositionClaw(POSITION_CLOSED_FIRST_CLAW, POSITION_CLOSED_SECOND_CLAW))
                .addDisplacementMarker(() -> {
                    out.setUpPosition(POSITION_UP_WOBBLE_ARM);
                    drive.turn(Math.toRadians(60));
                })
                .build();


        Trajectory trajectory5forOneRing = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(45)))
                .lineTo(new Vector2d(0, 55))
                .addDisplacementMarker(() -> {
                    drive.turn(Math.toRadians(-60));
                    in.setVelocity(0, DcMotorSimple.Direction.FORWARD);
                })
                .build();

        Trajectory trajectory6forOneRing = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-60, -9))
                .addDisplacementMarker(() -> {
                    out.setVelocity(FLYWHEEL_POWER);
                    drive.turn(Math.toRadians(-25));
                })
                .build();

        Trajectory trajectory7forOneRing = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .build();


        Trajectory trajectory3forFourRings = drive.trajectoryBuilder(trajectory2forAllPaths.end())
                .lineTo(new Vector2d(120, -20))
                .addDisplacementMarker(() -> {
                            drive.turn(Math.toRadians(90));
                            out.setUpPosition(POSITION_DOWN_WOBBLE_ARM);
                        }
                )
                .build();


        Trajectory trajectory4forFourRings = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(55, 0))
                .addDisplacementMarker(() ->
                {
                    drive.turn(Math.toRadians(-90));
                    in.setVelocity(0, DcMotorSimple.Direction.FORWARD);
                })
                .build();

        Trajectory trajectory5forFourRings = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-60, -9))
                .addDisplacementMarker(() ->
                {
                    out.setVelocity(FLYWHEEL_POWER);
                    drive.turn(Math.toRadians(-25));
                })
                .build();

        Trajectory trajectory6forFourRings = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .build();


        waitForStart();

        CameraTest.RingDeterminationPipeline.RingPosition pos = pipeline.getNumberOfRings();

        telemetry.addData("Number of Rings", pos);
        telemetry.update();

        drive.followTrajectory(trajectory0forAllPaths);

        runtime.reset();

        while (out.getMotorVelocity() < TuningController.rpmToTicksPerSecond(FLYWHEEL_POWER_ERROR) && runtime.seconds() < 1.0) {
            telemetry.addData("Loading Shooter", "Processing");
            telemetry.update();
        }

        out.setPusherSpeed(SERVO_SPEED);

        while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(FLYWHEEL_POWER_ERROR)) {
            telemetry.addData("First Ring", "Processing");
            telemetry.update();
        }

        out.setPusherSpeed(0);

        drive.followTrajectory(trajectory1forAllPaths);

        sleep(150);

        while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(FLYWHEEL_POWER_ERROR)) {
            telemetry.addData("Second Ring", "Processing");
            telemetry.update();
        }
        out.setPusherSpeed(0);

        drive.followTrajectory(trajectory2forAllPaths);

        out.setPusherSpeed(SERVO_SPEED);

        while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(FLYWHEEL_POWER_ERROR)) {
            telemetry.addData("Third Ring", "Processing");
            telemetry.update();
        }
        out.setPusherSpeed(0);
        out.setVelocity(0);

        if (pos == CameraTest.RingDeterminationPipeline.RingPosition.NONE){
            drive.followTrajectory(trajectory3forNoRings);
            sleep(750);

            out.setPositionClaw(POSITION_OPEN_FIRST_CLAW, POSITION_OPEN_SECOND_CLAW);

            sleep(750);

            drive.followTrajectory(trajectory4forNoRings);

            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(45)));

            drive.followTrajectory(trajectory5forNoRings);

            drive.setPoseEstimate(new Pose2d());

            drive.followTrajectory(trajectory6forNoRings);

            turret.setPositionForAngleServo(POSITION_FOR_ANGLE);

            out.setPusherSpeed(SERVO_SPEED);

            sleep(3000);

            out.setVelocity(0);
            out.setPusherSpeed(0);

            drive.setPoseEstimate(new Pose2d());

            drive.followTrajectory(trajectory7forNoRings);
        }
        else if (pos == CameraTest.RingDeterminationPipeline.RingPosition.ONE) {
            drive.followTrajectory(trajectory3forOneRing);

            sleep(750);

            out.setPositionClaw(POSITION_OPEN_FIRST_CLAW, POSITION_OPEN_SECOND_CLAW);

            sleep(750);

            drive.followTrajectory(trajectory4forOneRing);

            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(45)));

            drive.followTrajectory(trajectory5forOneRing);

            drive.setPoseEstimate(new Pose2d());

            drive.followTrajectory(trajectory6forOneRing);

            turret.setPositionForAngleServo(POSITION_FOR_ANGLE);

            out.setPusherSpeed(SERVO_SPEED);

            sleep(3000);

            out.setVelocity(0);
            out.setPusherSpeed(0);

            drive.setPoseEstimate(new Pose2d());

            drive.followTrajectory(trajectory7forOneRing);
        }
        else {
            drive.followTrajectory(trajectory3forFourRings);

            sleep(750);

            out.setPositionClaw(POSITION_OPEN_FIRST_CLAW, POSITION_OPEN_SECOND_CLAW);

            sleep(750);

            drive.setPoseEstimate(new Pose2d(0, 0));

            drive.followTrajectory(trajectory4forFourRings);

            drive.setPoseEstimate(new Pose2d());

            drive.followTrajectory(trajectory5forFourRings);

            turret.setPositionForAngleServo(POSITION_FOR_ANGLE);

            out.setPusherSpeed(SERVO_SPEED);

            sleep(3000);

            out.setVelocity(0);
            out.setPusherSpeed(0);

            drive.setPoseEstimate(new Pose2d());

            drive.followTrajectory(trajectory6forFourRings);
        }
    }
}

