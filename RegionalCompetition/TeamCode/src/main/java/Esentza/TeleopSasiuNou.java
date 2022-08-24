package Esentza;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TuningController;

@Config
@TeleOp(name="TeleopSasiuNou", group="Linear Opmode")

public class TeleopSasiuNou extends LinearOpMode {

    DcMotor leftFront, leftRear, rightFront, rightRear;

    Intake in;

    void setPowerToMotors(DcMotor motor1, DcMotor motor2, double power)
    {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    @Override
    public void runOpMode() {
        in = new Intake(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive())
        {
            double leftPower = gamepad1.left_stick_y;
            double rightPower = gamepad1.right_stick_y;

            double strafeLeftPower = gamepad1.left_trigger;
            double strafeRightPower = gamepad1.right_trigger;

            setPowerToMotors(leftFront, leftRear, leftPower);
            setPowerToMotors(rightFront, rightRear, rightPower);

            if (strafeRightPower != 0)
            {
                setPowerToMotors(leftFront, rightRear, strafeRightPower);
                setPowerToMotors(rightFront, leftRear, -strafeRightPower);
            }
            if (strafeLeftPower != 0)
            {
                setPowerToMotors(leftFront, rightRear, -strafeLeftPower);
                setPowerToMotors(rightFront, leftRear, strafeLeftPower);
            }

            if (gamepad1.a)
                in.setVelocity(10000, DcMotor.Direction.FORWARD);

            else in.setVelocity(0, DcMotorSimple.Direction.FORWARD);
        }
        }
    }
