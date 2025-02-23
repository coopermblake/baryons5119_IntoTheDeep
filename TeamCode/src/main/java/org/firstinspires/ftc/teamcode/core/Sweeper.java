package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Sweeper {
    public Servo sweeper;
    public Sweeper(Servo sweeper){
        this.sweeper = sweeper;
    }

    public void teleInput(Gamepad gamepad1){
        if(gamepad1.left_bumper){
            deploy();
        }
        if(gamepad1.right_bumper){
            retract();
        }
    }

    public void retract(){
        sweeper.setPosition(0.70);
    }

    public void deploy(){
        sweeper.setPosition(0);
    }
}
