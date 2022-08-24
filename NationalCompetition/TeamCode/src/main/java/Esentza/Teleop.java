package Esentza;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.TuningController;

@Config
@TeleOp(name="Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {

    public static double INTAKE_POWER = 10000;
    public static double TURRET_POWER = 0.6;

    public static double OUTPUT_POWER = 5400;
    public static double PUSHER_SPEED = 0.2;

    public static double POSITION_FOR_ANGLE = 0.41;
    public static double POSITION_FOR_POWERSHOTS = 0.4275;

    public static double POSITION_UP_WOBBLE_ARM = 0.5;
    public static double POSITION_DOWN_WOBBLE_ARM = 1;

    public static double POSITION_CLOSED_FIRST_CLAW = 0;
    public static double POSITION_OPEN_FIRST_CLAW = 0.5;

    public static double POSITION_CLOSED_SECOND_CLAW = 0;
    public static double POSITION_OPEN_SECOND_CLAW = 0.75;


    public static double POSITION_UP_BLOCKER = 0;
    public static double POSITION_DOWN_BLOCKER = 0.46;


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Intake in = new Intake(hardwareMap);
        Output out = new Output(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        waitForStart();

        while (!opModeIsActive())
        {
            telemetry.addData("Didn't start the match", "F");
            telemetry.update();
        }

        out.setBlockerPosition(POSITION_UP_BLOCKER);

        turret.setPositionForAngleServo(POSITION_FOR_ANGLE);

        drive.setPoseEstimate(new Pose2d(0, 0,0));

        Localizer myLocalizer = drive.getLocalizer();


        Trajectory firstPowerShotRedAlliance = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(0, -7.5))
                .addDisplacementMarker(() -> out.setPusherSpeed(PUSHER_SPEED))
                .build();


        Trajectory secondPowerShotRedAlliance = drive.trajectoryBuilder(firstPowerShotRedAlliance.end())
                .lineTo(new Vector2d(0, -15))
                .addDisplacementMarker(() -> out.setPusherSpeed(PUSHER_SPEED))
                .build();

        Trajectory thirdPowerShotRedAlliance = drive.trajectoryBuilder(secondPowerShotRedAlliance.end())
                .lineTo(new Vector2d(0, -22.5))
                .addDisplacementMarker(() -> out.setPusherSpeed(PUSHER_SPEED))
                .build();


        Trajectory firstPowerShotBlueAlliance = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(0, 7.5))
                .addDisplacementMarker(() -> out.setPusherSpeed(PUSHER_SPEED))
                .build();


        Trajectory secondPowerShotBlueAlliance = drive.trajectoryBuilder(firstPowerShotBlueAlliance.end())
                .lineTo(new Vector2d(0, 15))
                .addDisplacementMarker(() -> out.setPusherSpeed(PUSHER_SPEED))
                .build();

        Trajectory thirdPowerShotBlueAlliance = drive.trajectoryBuilder(secondPowerShotBlueAlliance.end())
                .lineTo(new Vector2d(0, 22.5))
                .addDisplacementMarker(() -> out.setPusherSpeed(PUSHER_SPEED))
                .build();


        while (opModeIsActive())
        {
            myLocalizer.update();

            Pose2d currentPose = myLocalizer.getPoseEstimate();

            telemetry.addData("X Coordinate", currentPose.getX());
            telemetry.addData("Y Coordinate", currentPose.getY());
            telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Output Speed", out.getMotorVelocity() * 60.0 / 28.0);
            telemetry.update();

            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            drive.setMotorPowers(leftPower, leftPower, rightPower, rightPower);

            if (gamepad1.right_trigger != 0) {
                double strafePower = gamepad1.right_trigger;
                drive.setMotorPowers(strafePower, -strafePower, strafePower, -strafePower);
            }

            if (gamepad1.left_trigger != 0) {
                double strafePower = gamepad1.left_trigger;
                drive.setMotorPowers(-strafePower, strafePower, -strafePower, strafePower);
            }

            if (gamepad2.right_bumper)
                in.setVelocity(INTAKE_POWER, DcMotor.Direction.FORWARD);
            else if (gamepad2.right_trigger != 0)
                in.setVelocity(INTAKE_POWER, DcMotor.Direction.REVERSE);
            else in.setVelocity(0, DcMotor.Direction.FORWARD);

            if (gamepad2.left_bumper)
            {
                out.setBlockerPosition(POSITION_UP_BLOCKER);
                out.setVelocity(OUTPUT_POWER);
                if (gamepad2.left_stick_y > 0)
                    out.setPusherSpeed(PUSHER_SPEED);

                else if (gamepad2.left_stick_y < 0)
                    out.setPusherSpeed(-PUSHER_SPEED);

                else out.setPusherSpeed(0);
            }

            else
            {
                out.setBlockerPosition(POSITION_DOWN_BLOCKER);
                out.setPusherSpeed(0);
                out.setVelocity(0);
            }

            if (gamepad2.dpad_up)
                out.setUpPosition(POSITION_UP_WOBBLE_ARM);

            if (gamepad2.dpad_down)
                out.setUpPosition(POSITION_DOWN_WOBBLE_ARM);

            if (gamepad2.dpad_left)
                out.setPositionClaw(POSITION_OPEN_FIRST_CLAW, POSITION_OPEN_SECOND_CLAW);

            if (gamepad2.dpad_right)
                out.setPositionClaw(POSITION_CLOSED_FIRST_CLAW, POSITION_CLOSED_SECOND_CLAW);

            if (gamepad1.dpad_left)
                turret.spinTurrent(TURRET_POWER);

            else if (gamepad1.dpad_right)
                turret.spinTurrent(-TURRET_POWER);

            if (gamepad1.dpad_up)
                turret.setPositionForAngleServo(POSITION_FOR_ANGLE);

            else if (gamepad1.dpad_right)
                turret.setPositionForAngleServo(POSITION_FOR_POWERSHOTS);

            else turret.spinTurrent(0);


            if (gamepad1.a)
            {
                turret.setPositionForAngleServo(POSITION_FOR_POWERSHOTS);
                out.setVelocity(4200);

                while (out.getMotorVelocity() <= TuningController.rpmToTicksPerSecond(3800))
                {
                    telemetry.addData("Flywheel Power", "Calibrating");
                    telemetry.update();
                }

                drive.setPoseEstimate(new Pose2d());
                drive.followTrajectory(firstPowerShotRedAlliance);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3800))
                {
                    telemetry.addData("First Power Shot", "Launching");
                    telemetry.update();
                }

                out.setPusherSpeed(0);

                drive.followTrajectory(secondPowerShotRedAlliance);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600)) {
                    telemetry.addData("Second Power Shot", "Launching");
                    telemetry.update();
                }
                out.setPusherSpeed(0);

                if (gamepad1.x)
                    continue;

                drive.followTrajectory(thirdPowerShotRedAlliance);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600))
                {
                    telemetry.addData("First Power Shot", "Launching");
                    telemetry.update();
                }
                out.setPusherSpeed(0);
                out.setVelocity(0);

                telemetry.addData("Power Shots","Launched");
                telemetry.update();

                turret.setPositionForAngleServo(POSITION_FOR_ANGLE);
            }
            if (gamepad1.b)
            {
                turret.setPositionForAngleServo(POSITION_FOR_POWERSHOTS);
                out.setVelocity(4200);

                while (out.getMotorVelocity() <= TuningController.rpmToTicksPerSecond(3800))
                {
                    telemetry.addData("Flywheel Power", "Calibrating");
                    telemetry.update();
                }

                drive.setPoseEstimate(new Pose2d());
                drive.followTrajectory(firstPowerShotBlueAlliance);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3800))
                {
                    telemetry.addData("First Power Shot", "Launching");
                    telemetry.update();
                }

                out.setPusherSpeed(0);

                drive.followTrajectory(secondPowerShotBlueAlliance);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600)) {
                    telemetry.addData("Second Power Shot", "Launching");
                    telemetry.update();
                }
                out.setPusherSpeed(0);

                if (gamepad1.x)
                    continue;

                drive.followTrajectory(thirdPowerShotBlueAlliance);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600))
                {
                    telemetry.addData("First Power Shot", "Launching");
                    telemetry.update();
                }
                out.setPusherSpeed(0);
                out.setVelocity(0);

                telemetry.addData("Power Shots","Launched");
                telemetry.update();

                turret.setPositionForAngleServo(POSITION_FOR_ANGLE);
            }
        }
    }
}
