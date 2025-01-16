package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ViperSlide {
    public final DcMotor slideExt;
    public final DcMotor slideRot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Servo gripper;
    private boolean debounce = false;
    private boolean gripperPosition = false;
    public boolean driverControl = false;
    public int rotMin;
    public int rotMax;
    public int extMin;
    public int extMaxLow;
    public int extMaxHigh;
    private final double rotSensitivity = 1; // equivalent to 1000 ticks/s
    private final double extSensitivity = 1;
    private long lastCycle;

    public ViperSlide(DcMotor slideExt, DcMotor slideRot, Gamepad gamepad1, Gamepad gamepad2, Servo gripper) {
        this.slideExt = slideExt;
        this.slideRot = slideRot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.gripper = gripper;
        slideExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoder();
    }

    public void moveSlide(double inputRot, double inputExt) {
       if(slideExt.getCurrentPosition()> extMaxLow && inputExt > 0 && slideRot.getCurrentPosition() - rotMin > -2500){
           inputExt = 0;
       }

       else if (slideExt.getCurrentPosition()>extMaxHigh && inputExt > 0){
           inputExt = 0;
        }

        slideRot.setPower(-inputRot);
        slideExt.setPower(inputExt);

    }

    public void resetEncoder(){
        rotMin = slideRot.getCurrentPosition();
        rotMax = rotMin + 4000;
        extMin = slideExt.getCurrentPosition();
        extMaxLow = extMin + 3300;
        extMaxHigh = extMin + 4000;

    }

    private void handleGripper() {
        // we use debounce to make pressing the trigger only toggle the gripper once per press
        if (gamepad2.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
            if (!debounce) {
                if (gripperPosition) {
                    gripper.setPosition(0.81);
                } else {
                    gripper.setPosition(0.47);
                }
                gripperPosition = !gripperPosition;
                debounce = true;
            }
        } else {
            debounce = false;
        }
    }

    public void teleopSlideMovement(Gamepad gamepad1, Gamepad gamepad2) {
        double extPower;
        double rotPower;
        if (driverControl) {
            handleGripper();

            if(gamepad1.y){
                resetEncoder();
            }

            //moveSlide(-gamepad2.right_stick_y, -gamepad2.left_stick_y);
            rotPower=-gamepad2.right_stick_y;
            extPower = -gamepad2.left_stick_y;
            if (Math.abs(-gamepad2.right_stick_y) < 0.05) {
                //slideRot.setPower(0);
                rotPower=0;
            } else {
                //slideRot.setPower(-gamepad2.right_stick_y);
            }
            if (Math.abs(-gamepad2.left_stick_y) < 0.05) {
                //slideExt.setPower(0);
                extPower = 0;
            } else {
                //slideExt.setPower(-gamepad2.left_stick_y);
            }
            moveSlide(rotPower, extPower);
            handleMacros(gamepad2);
        }
    }

    private void handleMacros(Gamepad gamepad2) {

    }
}
