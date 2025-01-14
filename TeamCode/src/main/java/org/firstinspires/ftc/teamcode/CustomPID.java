package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomPID {
    double kP;
    double kI;
    double kD;
    double iT;
    double pErr;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public CustomPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iT = 0;
        this.pErr = 0;
    }

    // This function cycles the PID controller.
    public double cycleController(double setPoint, double realPoint) {
        double time = timer.milliseconds();
        double err = setPoint - realPoint;
        double p = kP * err;
        double i = kI * err * time;
        iT += i;
        double d = kD * (err - pErr) / time;
        pErr = err;
        timer.reset();
        return (p + iT + d);
    }

    public void updateController() {
        pErr = 0;
        iT = 0;
        timer.reset();
    }
}
