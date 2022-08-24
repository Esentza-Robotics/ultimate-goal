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
@Autonomous(name="Blue2ndPosition", group="yes")

public class Blue2ndPosition extends LinearOpMode {

    public static OpenCvCamera camera;

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

    public static double POSITION_FOR_POWERSHOTS = 0.417;

    public static double POSITION_UP_WOBBLE_ARM = 0.5;
    public static double POSITION_DOWN_WOBBLE_ARM = 0.95;

    public static double POSITION_CLOSED_FIRST_CLAW = 0;
    public static double POSITION_OPEN_FIRST_CLAW = 0.5;

    public static double POSITION_CLOSED_SECOND_CLAW = 0;
    public static double POSITION_OPEN_SECOND_CLAW = 0.75;


    public static double INITIAL_LENGTH = 10;
    public static double SERVO_SPEED = -0.2;
    public static double SERVO_SPEED_TOWER = 0.225;
    public static double FLYWHEEL_POWER = 5400;
    public static double FLYWHEEL_POWER_ERROR = 4500;
    @Override
    public void runOpMode() {
        CameraStorage.x = 425;
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
                .splineToConstantHeading(new Vector2d(40, 23), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -5), Math.toRadians(0))
                .addDisplacementMarker(INITIAL_LENGTH, ()->
                {
                    turret.setPositionForAngleServo(POSITION_FOR_ANGLE);
                    out.setVelocity(FLYWHEEL_POWER);
                })
                .addDisplacementMarker(()->out.setPusherSpeed(SERVO_SPEED))
                .build();

        Trajectory trajectory1forNoRings = drive.trajectoryBuilder(trajectory0forAllPaths.end())
                .lineTo(new Vector2d(85, 17))
                .addDisplacementMarker(()->out.setUpPosition(POSITION_DOWN_WOBBLE_ARM))
                .build();

        Trajectory trajectory2forNoRings = drive.trajectoryBuilder(trajectory1forNoRings.end())
                .splineToConstantHeading(new Vector2d(90, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(80, -30), Math.toRadians(0))
                .addDisplacementMarker(()->out.setPositionClaw(POSITION_CLOSED_FIRST_CLAW, POSITION_CLOSED_SECOND_CLAW))
                .build();


        Trajectory trajectory1forOneRing = drive.trajectoryBuilder(trajectory0forAllPaths.end())
                .lineTo(new Vector2d(112, -8))
                .addDisplacementMarker(()->out.setUpPosition(POSITION_DOWN_WOBBLE_ARM))
                .build();

        Trajectory trajectory2forOneRing = drive.trajectoryBuilder(trajectory1forOneRing.end())
                .lineTo(new Vector2d(120, 17))
                .addDisplacementMarker(5, ()->out.setPositionClaw(POSITION_CLOSED_FIRST_CLAW, POSITION_CLOSED_SECOND_CLAW))
                .build();

        Trajectory trajectory3forOneRing = drive.trajectoryBuilder(trajectory2forOneRing.end())
                .lineTo(new Vector2d(70, 17))
                .addDisplacementMarker(()->out.setUpPosition(POSITION_UP_WOBBLE_ARM))
                .build();

        Trajectory trajectory1forFourRings = drive.trajectoryBuilder(trajectory0forAllPaths.end())
                .lineTo(new Vector2d(112, -8))
                .addDisplacementMarker(()->
                {
                    drive.turn(Math.toRadians(-90));
                    out.setUpPosition(POSITION_DOWN_WOBBLE_ARM);
                })
                .build();

        Trajectory trajectory2forFourRings = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-8, 0))
                .addDisplacementMarker(()->out.setPositionClaw(POSITION_OPEN_FIRST_CLAW, POSITION_OPEN_SECOND_CLAW))
                .build();

        Trajectory trajectory3forFourRings = drive.trajectoryBuilder(trajectory2forFourRings.end())
                .lineTo(new Vector2d(-15, -45))
                .addDisplacementMarker(5, ()->out.setPositionClaw(POSITION_CLOSED_FIRST_CLAW, POSITION_CLOSED_SECOND_CLAW))
                .addDisplacementMarker(10, ()->out.setUpPosition(POSITION_UP_WOBBLE_ARM))
                .build();

        waitForStart();

        CameraTest.RingDeterminationPipeline.RingPosition pos = pipeline.getNumberOfRings();

        telemetry.addData("Number of Rings", pos);
        telemetry.update();

        drive.followTrajectory(trajectory0forAllPaths);

        sleep(5000);

        out.setVelocity(0);
        out.setPusherSpeed(0);


        if (pos == CameraTest.RingDeterminationPipeline.RingPosition.NONE)
        {
            drive.followTrajectory(trajectory1forNoRings);

            sleep(500);

            out.setPositionClaw(POSITION_OPEN_FIRST_CLAW, POSITION_OPEN_SECOND_CLAW);

            sleep(1500);

            drive.followTrajectory(trajectory2forNoRings);

            sleep(500);

            out.setUpPosition(POSITION_UP_WOBBLE_ARM);

            sleep(500);
        }
        else if (pos == CameraTest.RingDeterminationPipeline.RingPosition.ONE)
        {

            drive.followTrajectory(trajectory1forOneRing);

            sleep(1500);

            out.setPositionClaw(POSITION_OPEN_FIRST_CLAW, POSITION_OPEN_SECOND_CLAW);

            sleep(500);

            drive.followTrajectory(trajectory2forOneRing);
            drive.followTrajectory(trajectory3forOneRing);

            sleep(1000);
        }
        else
        {
            drive.followTrajectory(trajectory1forFourRings);

            drive.setPoseEstimate(new Pose2d());

            drive.followTrajectory(trajectory2forFourRings);

            sleep(1000);

            drive.followTrajectory(trajectory3forFourRings);
        }
    }
}

