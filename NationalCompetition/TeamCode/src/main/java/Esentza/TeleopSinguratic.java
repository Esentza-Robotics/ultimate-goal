package Esentza;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@Config
@TeleOp(name="TeleopSinguratic", group="Linear Opmode")

public class TeleopSinguratic extends LinearOpMode {

    public static double INTAKE_POWER = 5400;

    public static double TURRET_POWER = 0.7;

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

        turret.setPositionForAngleServo(POSITION_FOR_ANGLE);

        drive.setPoseEstimate(new Pose2d(0, 0,0));

        Localizer myLocalizer = drive.getLocalizer();

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

            if (gamepad1.left_trigger != 0)
            {
                double strafePower = gamepad1.left_trigger;
                drive.setMotorPowers(-strafePower, strafePower, -strafePower, strafePower);
            }

            if (gamepad1.right_bumper)
                in.setVelocity(INTAKE_POWER, DcMotor.Direction.FORWARD);
            else in.setVelocity(0, DcMotor.Direction.FORWARD);

            if (gamepad1.left_bumper)
            {
                out.setVelocity(OUTPUT_POWER);
                if (gamepad1.a)
                    out.setPusherSpeed(PUSHER_SPEED);

                else if (gamepad1.b)
                    out.setPusherSpeed(-PUSHER_SPEED);

                else out.setPusherSpeed(0);
            }

            else
            {
                out.setPusherSpeed(0);
                out.setVelocity(0);
            }

            if (gamepad1.dpad_up)
                out.setUpPosition(POSITION_UP_WOBBLE_ARM);

            if (gamepad1.dpad_down)
                out.setUpPosition(POSITION_DOWN_WOBBLE_ARM);

            if (gamepad1.dpad_left)
                out.setPositionClaw(POSITION_OPEN_FIRST_CLAW, POSITION_OPEN_SECOND_CLAW);

            if (gamepad1.dpad_right)
                out.setPositionClaw(POSITION_CLOSED_FIRST_CLAW, POSITION_CLOSED_SECOND_CLAW);

        }
    }
}
