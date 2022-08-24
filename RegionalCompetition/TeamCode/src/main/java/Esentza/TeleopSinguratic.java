
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
@TeleOp(name="Teleop Singuratic", group="Linear Opmode")

public class TeleopSinguratic extends LinearOpMode {

    SampleMecanumDrive drive;
    Intake in;
    Output out;

    public static double OUTPUT_POWER = 4800;
    public static double SERVO_POWER = 0.275;

    @Override
    public void runOpMode() {

        boolean positioned = false;

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
                .lineTo(new Vector2d(0, -8))
                .addDisplacementMarker(() -> out.turnOnServo(0.17))
                .build();


        Trajectory thirdPowerShot = drive.trajectoryBuilder(secondPowerShot.end())
                .lineTo(new Vector2d(0, -16))
                .addDisplacementMarker(() -> out.turnOnServo(0.17))
                .build();

        waitForStart();

        out.setUpPosition(0);

        drive.setPoseEstimate(new Pose2d());

        while (!opModeIsActive())
        {
            telemetry.addData("Didn't start the match", "F");
            telemetry.update();
        }

        out.setUpPosition(0);
        drive.setPoseEstimate(new Pose2d());

        while (opModeIsActive())
        {
            if (!positioned)
            {
                positioned = true;
                if (gamepad1.x)
                    drive.followTrajectory(positioningForNoRing);

                if (gamepad1.y)
                    drive.followTrajectory(positioningForOneRing);

                if (gamepad1.b)
                    drive.followTrajectory(positioningForFourRing);
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

            if (gamepad1.right_bumper)
                in.setVelocity(10000, DcMotor.Direction.FORWARD);
            else in.setVelocity(0, DcMotor.Direction.FORWARD);

            if (gamepad1.dpad_down)
                out.turnOnServo(SERVO_POWER);

            else out.turnOnServo(0);

            if (gamepad1.dpad_up)
                in.setPosition(0.65);

            if (gamepad1.left_bumper)
                out.setVelocity(OUTPUT_POWER);

            else
            {
                out.turnOnServo(0);
                out.setVelocity(0);
            }

            if (gamepad1.a)
            {
                out.setVelocity(3700);

                while (out.getMotorVelocity() <= TuningController.rpmToTicksPerSecond(3600)) ;

                drive.setPoseEstimate(new Pose2d());
                out.turnOnServo(0.25);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600)) ;
                out.turnOnServo(0);

                drive.followTrajectory(secondPowerShot);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600)) ;
                out.turnOnServo(0);

                if (gamepad1.x)
                    continue;

                drive.followTrajectory(thirdPowerShot);

                while (out.getMotorVelocity() > TuningController.rpmToTicksPerSecond(3600)) ;
                out.turnOnServo(0);
            }
        }
    }
}
