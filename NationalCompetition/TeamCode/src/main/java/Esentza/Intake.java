package Esentza;


import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.TuningController;


class Intake
{
    static DcMotorEx intakeMotor;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0, 10, 100);

    private final VoltageSensor batteryVoltageSensor;

    public Intake(HardwareMap hm)
    {
        intakeMotor = hm.get(DcMotorEx.class, "intake");

        for (LynxModule module : hm.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
            MotorConfigurationType motorConfigurationType = intakeMotor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            intakeMotor.setMotorType(motorConfigurationType);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hm.voltageSensor.iterator().next();
        setPIDFCoefficients(intakeMotor, MOTOR_VELO_PID);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        Log.i("config", "setting custom gains");
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()));
    }

    public double getMeasuredVelocity()
    {
        return intakeMotor.getVelocity();
    }


    public void setVelocity(double power, DcMotor.Direction dir) {
        intakeMotor.setDirection(dir);
        intakeMotor.setVelocity(TuningController.rpmToTicksPerSecond(power));
        Log.i("mode", "setting velocity");
    }
}

