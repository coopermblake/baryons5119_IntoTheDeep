package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.CustomPID;

public class ViperSlide {
    public final DcMotor slideExt;
    public final DcMotor slideRot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Servo gripper;
    private boolean debounceGripper = false;
    private boolean debounceY = false;
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

    public enum RotateMacro{
        GOING_UP,
        GOING_DOWN,
        NONE,
        GOING_HORIZONTAL
    }

    public RotateMacro rotateMacro = RotateMacro.NONE;

    private boolean hanging = false;

    public enum ExtendMacro{
        NONE,
        MIN,
        HANG,
        GRAB
    }

    public ExtendMacro extendMacro = ExtendMacro.NONE;

//    @Config
//    public static class rotPID {
//        public static double kP = 0.001;
//        public static double kI = 0.0;
//        public static double kD = 0.0;
//    }

    //private CustomPID rotationHoldPID = new CustomPID(rotPID.kP, rotPID.kI, rotPID.kD);

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
       if(inputRot == 0 && rotateMacro == RotateMacro.NONE){
            slideRot.setTargetPosition(lastRotPosition);
            slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(slideRot.getCurrentPosition() > lastRotPosition){
                //double power = rotationHoldPID.cycleController(lastRotPosition, slideExt.getCurrentPosition());
                //inputRot = Range.clip(power, -1.0, 1.0); //only move if rot has dropped too low

                slideRot.setTargetPosition(lastRotPosition);
                slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                inputRot = 1.0;
            }
       }
       else{
           slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           lastRotPosition = slideRot.getCurrentPosition();
       }

       if(hanging && slideExt.getCurrentPosition() > (extMin + 50)){
           inputExt = -1.0;
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
        if (gamepad2.right_trigger > 0.05) {
            if (!debounceGripper) {
                if (gripperPosition) {
                    gripper.setPosition(0.81);
                } else {
                    gripper.setPosition(0.47);
                }
                gripperPosition = !gripperPosition;
                debounceGripper = true;
            }
        } else {
            debounceGripper = false;
        }
    }

    public void teleopSlideMovement(Gamepad gamepad1, Gamepad gamepad2) {
        double extPower;
        double rotPower;
        //TODO: make separate funcions to allow macroing one while manual the other

        handleRotateMacros(gamepad2);
        handleExtendMacros(gamepad2);

        if (driverControl && rotateMacro == RotateMacro.NONE && extendMacro == ExtendMacro.NONE) {
            handleGripper();

            if(gamepad2.start){
                resetEncoder();
            }

            if(gamepad2.left_trigger>0.05){
                hanging = true;
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
        }
    }

    private void handleRotateMacros(Gamepad gamepad2){


        //up for up, down for down, right for horizontal, left for stop
        //TODO: make these values based on the Arm config class


        if (gamepad2.dpad_up) {
            rotateMacro = RotateMacro.GOING_UP;
            slideRot.setTargetPosition(rotMin - 2900);
            slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRot.setPower(1);
        }
        if (gamepad2.dpad_down) {
            rotateMacro = RotateMacro.GOING_DOWN;
            slideRot.setTargetPosition(rotMin-100);
            slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRot.setPower(1);
        }
        if(gamepad2.dpad_right){
            rotateMacro = RotateMacro.GOING_HORIZONTAL;
            slideRot.setTargetPosition(rotMin-1100);
            slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRot.setPower(1);
        }

        if(gamepad2.dpad_left || (Math.abs(slideRot.getCurrentPosition()  - slideRot.getTargetPosition()) < 10)
                || gamepad2.back) {
            rotateMacro = RotateMacro.NONE;
            slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRot.setPower(0);
        }

        lastRotPosition = slideRot.getCurrentPosition();

    }

    private void handleExtendMacros(Gamepad gamepad2){
        //up for up, down for down, right for horizontal, left for stop
        //TODO: make these values based on the Arm config class

        if (gamepad2.y) {
            if(!debounceY) {
                extendMacro = ExtendMacro.HANG;

                if (slideExt.getCurrentPosition() < extMin + 1100) {
                    slideExt.setTargetPosition(extMin + 1200);
                } else {
                    slideExt.setTargetPosition(extMin + 600);
                }
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideExt.setPower(1);
                debounceY = true;
            }
            else{
                debounceY = false;
            }
        }
        if (gamepad2.a) {
            extendMacro = ExtendMacro.MIN;
            slideExt.setTargetPosition(extMin + 400);
            slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideExt.setPower(1);
        }
        if(gamepad2.x || extendMacro == ExtendMacro.GRAB){
            extendMacro = ExtendMacro.GRAB;

            //start extending when rotation reaches horizontal
            if(Math.abs(slideRot.getCurrentPosition() - slideRot.getTargetPosition()) < 50){
                slideExt.setTargetPosition(extMin + 1000);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideExt.setPower(0.8);
            }

        }

        if(gamepad2.b || (Math.abs(slideExt.getCurrentPosition()  - slideExt.getTargetPosition()) < 10)
                || gamepad2.back) {
            extendMacro = ExtendMacro.NONE;
            slideExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideExt.setPower(0);
        }

    }
}
