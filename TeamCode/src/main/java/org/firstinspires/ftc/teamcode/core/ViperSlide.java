package org.firstinspires.ftc.teamcode.core;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

    private int lastRotPosition;
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


       //hold position if not trying to move slideRot
       if(inputRot == 0){
            slideRot.setTargetPosition(lastRotPosition);
            slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(slideRot.getCurrentPosition() > lastRotPosition){
                inputRot = 1.0; //only move if rot has dropped too low
            }
       }
       else{
           slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           lastRotPosition = slideRot.getCurrentPosition();
       }

        slideRot.setPower(-inputRot);
        slideExt.setPower(inputExt);

    }

    public void resetEncoder(){
        rotMin = slideRot.getCurrentPosition();
        rotMax = rotMin - 4000;
        extMin = slideExt.getCurrentPosition();
        extMaxLow = extMin + 3300;
        extMaxHigh = extMin + 4120;

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

            if(gamepad2.start){
                resetEncoder();
            }

            //moveSlide(-gamepad2.right_stick_y, -gamepad2.left_stick_y);
            rotPower=-gamepad2.right_stick_y;
            extPower = -gamepad2.left_stick_y;
            if (Math.abs(-gamepad2.right_stick_y) < 0.05) {
                //slideRot.setPower(0);
                rotPower=0;
            }
            if (Math.abs(-gamepad2.left_stick_y) < 0.05) {
                //slideExt.setPower(0);
                extPower = 0;
            }
            moveSlide(rotPower, extPower);
            handleMacros(gamepad2);
        }
    }

    private void handleMacros(Gamepad gamepad2) {

    }

    //lift arm up to hang
    public class RotateUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            int error = slideRot.getCurrentPosition() - (rotMin-Arm.rot_hang);
            double power = -0.1*(error);
            power = Range.clip(power, -1, 1);
            slideRot.setPower(power);
            return Math.abs(error)>10;
        }
    }

    //extend arm to hang
    public class ExtendToHang implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            int error = slideExt.getCurrentPosition() - (extMin+Arm.ext_hang);
            double power = -0.1*(error);
            power = Range.clip(power, -1, 1);
            slideExt.setPower(power);
            return Math.abs(error)>10;
        }
    }

    //lower arm to grabbing
    public class RotateHorizontal implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            int error = slideRot.getCurrentPosition() - (rotMin-Arm.rot_hor);
            double power = -0.1*(error);
            power = Range.clip(power, -1, 1);
            slideRot.setPower(power);
            return Math.abs(error)>10;
        }
    }

    //retract arm for Hang
    public class RetractToHang implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            int error = slideExt.getCurrentPosition() - (extMin+Arm.ret_hang);
            double power = -0.1*(error);
            power = Range.clip(power, -1, 1);
            slideExt.setPower(power);
            return Math.abs(error)>10;
        }
    }

    //extend arm for grabbing
    public class ExtendToGrab implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            int error = slideExt.getCurrentPosition() - (extMin+Arm.ext_grab);
            double power = -0.1*(error);
            power = Range.clip(power, -1, 1);
            slideExt.setPower(power);
            return Math.abs(error)>10;
        }
    }

    public class CloseGripper implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(0.81);
            return false;
        }
    }

    public class OpenGripper implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            gripper.setPosition(0.47);
            return false;
        }
    }

    public Action rotateUp(){
        return new RotateUp();
    }
    public Action rotateHorizontal(){
        return new RotateHorizontal();
    }
    public Action extendToHang(){
        return new ExtendToHang();
    }
    public Action retractToHang(){
        return new RetractToHang();
    }
    public Action extendToGrab(){
        return new ExtendToGrab();
    }
    public Action openGripper(){
        return new OpenGripper();
    }
    public Action closeGripper(){
        return new CloseGripper();
    }
    public static class Arm{
        public static int ext_hang = 1200;
        public static int ret_hang = 600;
        public static int ext_grab = 400;
        public static int rot_hang = 2900;
        public static int rot_hor = 1000;
    }

}