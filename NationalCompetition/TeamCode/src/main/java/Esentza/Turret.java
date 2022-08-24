package Esentza;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

class Turret {
    DcMotorEx firstMotor, secondMotor;

    Servo angleServo, stopper;

    static final double COUNTS_PER_MOTOR_REV = 751.8;
    static final double DRIVE_GEAR_REDUCTION = 2.7;
    static final double COUNT_PER_COMPLETE_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double MOTOR_SPEED = 0.4;


    public Turret(HardwareMap hmap)
    {
        firstMotor = hmap.get(DcMotorEx.class, "firstMotor");
        secondMotor = hmap.get(DcMotorEx.class, "secondMotor");
        firstMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleServo = hmap.get(Servo.class, "angleServo");
        stopper = hmap.get(Servo.class, "stopper");
    }


    public void spinTurrent(double power)
    {
        firstMotor.setPower(power);
        secondMotor.setPower(power);
    }

    public void setPositionForAngleServo(double position)
    {
        angleServo.setPosition(position);
    }

    public void setPositionForStopper(double position)
    {
        stopper.setPosition(position);
    }

}
