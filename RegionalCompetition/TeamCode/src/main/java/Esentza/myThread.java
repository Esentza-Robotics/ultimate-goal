package Esentza;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class myThread extends Thread
{
    Telemetry telem;

    Intake in;

    public myThread(Intake intake, Telemetry tel) {
        in = intake;
        telem = tel;
    }

    boolean ok, isRunning = true;

    public void setCurrentStateOfRunning(boolean currentStateOfRunning)
    {
        isRunning = currentStateOfRunning;
    }


    public void run()
    {

    }

}
