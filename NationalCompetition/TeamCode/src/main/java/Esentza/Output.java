package Esentza;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.opmode.TuningController;

public class Output{
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30, 0, 2, 15);

    private final VoltageSensor batteryVoltageSensor;

    static DcMotorEx avion;
    static Servo firstClaw, secondClaw, up, blocker;
    static CRServo s1;

    public Output(HardwareMap hardwareMap)
    {
        avion = hardwareMap.get(DcMotorEx.class, "avion");
        avion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType motorConfigurationType = avion.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        avion.setMotorType(motorConfigurationType);

        avion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(avion, MOTOR_VELO_PID);

        s1 = hardwareMap.get(CRServo.class, "servo");
        firstClaw = hardwareMap.get(Servo.class, "firstClaw");
        secondClaw = hardwareMap.get(Servo.class, "secondClaw");
        up = hardwareMap.get(Servo.class, "up");
        blocker = hardwareMap.get(Servo.class, "blocker");
    }

    public void setPositionClaw(double positionForFirstClaw, double positionForSecondClaw)
    {
        firstClaw.setPosition(positionForFirstClaw);
        secondClaw.setPosition(positionForSecondClaw);
    }

    public void setUpPosition(double position)
    {
        up.setPosition(position);
    }

    public void setBlockerPosition(double position)
    {
        blocker.setPosition(position);
    }

    public void setPusherSpeed(double speed)
    {
        s1.setPower(speed);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()));
    }

    public void setVelocity(double power) {
            avion.setVelocity(TuningController.rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
    }

    public double getMotorVelocity()
    {
        return avion.getVelocity();
    }


}
