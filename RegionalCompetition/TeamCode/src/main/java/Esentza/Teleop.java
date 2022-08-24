package Esentza;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TuningController;

@Config
@TeleOp(name="Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {

    SampleMecanumDrive drive;
    Intake in;
    Output out;

    public static double OUTPUT_POWER = 4800;
    public static double SERVO_POWER = 0.275;

    @Override
    public void runOpMode() {
        boolean positioned = false;

        int cnt = 0;

        drive = new SampleMecanumDrive(hardwareMap);
        in = new Intake(hardwareMap);
        out = new Output(hardwareMap);

        Trajectory positioningForNoRing = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-48, -10))
                .build();

        Trajectory positioningForOneRing = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-38, 2))
                .build();

        Trajectory positioningForFourRing = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(40, -8, Math.toRadians(180)))
                .build();

        Trajectory secondPowerShot = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(0, -7.5))
                .addDisplacementMarker(() -> out.turnOnServo(0.17))
                .build();


        Trajectory thirdPowerShot = drive.trajectoryBuilder(secondPowerShot.end())
                .lineTo(new Vector2d(0, -15))
                .addDisplacementMarker(() -> out.turnOnServo(0.17))
                .build();

        waitForStart();

        while (!opModeIsActive())
        {
            telemetry.addData("Didn't start the match", "F");
            telemetry.update();
        }

        out.setUpPosition(0);

        in.setPosition(1);

        drive.setPoseEstimate(new Pose2d());

        boolean ok = true;

        while (opModeIsActive())
        {

                telemetry.addData("The number of rings colected", cnt);
                telemetry.update();

                if (gamepad1.x && !positioned) {
                    drive.followTrajectory(positioningForNoRing);
                    positioned = true;
                }
                if (gamepad1.y && !positioned) {
                    drive.followTrajectory(positioningForOneRing);
                    positioned = true;
                }
                if (gamepad1.b && !positioned) {
                    drive.followTrajectory(positioningForFourRing);
                    positioned = true;
                }

            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            if (leftPower != 0 || rightPower != 0)
                positioned = true;

            drive.setMotorPowers(leftPower, leftPower, rightPower, rightPower);

            if (gamepad1.right_trigger != 0) {
                double strafePower = gamepad1.right_trigger;
                drive.setMotorPowers(strafePower, -strafePower, strafePower, -strafePower);
            }

            if (gamepad1.left_trigger != 0)
            {
                double strafePower = gamepad1.left_trigger;
                drive.setMotorPowers(-strafePower, strafePower, -strafePower, strafePower);
            }

            if (gamepad2.right_bumper)
                in.setVelocity(10000, DcMotor.Direction.FORWARD);
            else if (gamepad2.right_trigger != 0)
                in.setVelocity(10000, DcMotor.Direction.REVERSE);
            else in.setVelocity(0, DcMotor.Direction.FORWARD);

            if (gamepad1.dpad_left)
                drive.turn(Math.toRadians(-90));

            if (gamepad1.dpad_up)
                drive.turn(Math.toRadians(-17));

            if (gamepad1.dpad_left)
                drive.turn(Math.toRadians(90));

            if (gamepad2.a)
                out.setUpPosition(0);

            if (gamepad2.b)
                out.setUpPosition(1);

            if (gamepad2.x)
                out.setPositionClaw(1);

            if (gamepad2.y)
                out.setPositionClaw(0);

            if (gamepad2.left_stick_y > 0)
                out.turnOnServo(-SERVO_POWER);

            else if (gamepad2.left_stick_y < 0)
                out.turnOnServo(SERVO_POWER);

            else out.turnOnServo(0);

            if (gamepad2.right_stick_y != 0)
                    in.setPosition(0.65);

            if (gamepad2.dpad_up) {
                out.setPositionClaw(0);
                sleep(500);
                out.setUpPosition(0);
            }

            if (gamepad2.dpad_down) {
                out.setUpPosition(1);
                sleep(500);
                out.setPositionClaw(1);
            }

            if (gamepad2.left_bumper)
                out.setVelocity(OUTPUT_POWER);

            else
            {
                out.turnOnServo(0);
                out.setVelocity(0);
            }

            if (gamepad1.a)
            {
                out.setVelocity(3700);

                while (out.getMotorVelocity() <= TuningController.rpmToTicksPerSecond(3600))
                {
                    telemetry.addData("Flywheel Power", "Calibrating");
                    telemetry.update();
                }

                drive.setPoseEstimate(new Pose2d());
                out.turnOnServo(0.25);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600))
                {
                    telemetry.addData("First Power Shot", "Launching");
                    telemetry.update();
                }
                out.turnOnServo(0);

                drive.followTrajectory(secondPowerShot);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600)) {
                    telemetry.addData("Second Power Shot", "Launching");
                    telemetry.update();
                }
                out.turnOnServo(0);

                if (gamepad1.x)
                    continue;

                drive.followTrajectory(thirdPowerShot);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600))
                {
                    telemetry.addData("First Power Shot", "Launching");
                    telemetry.update();
                }
                out.turnOnServo(0);
                out.setVelocity(0);

                telemetry.addData("Power Shots","Launched");
                telemetry.update();
            }
        }
    }
}
