package org.firstinspires.ftc.teamcode.core;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.CustomPID;

import java.util.Timer;

public class ViperSlide {
    public final DcMotor slideExt;
    public final DcMotor slideRot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Servo gripper;
    private boolean debounceGripper = false;
    private boolean last_d_pad_up = false;
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

    long holdDelay;

    private int lastRotPosition;

    public enum RotateMacro{
        GOING_UP,
        GOING_DOWN,
        NONE,
        GOING_HORIZONTAL
    }

    public RotateMacro rotateMacro = RotateMacro.NONE;

    private boolean hanging = false;

    private boolean justStopped = false;

    public enum ExtendMacro{
        NONE,
        MIN,
        HANG,
        GRAB
    }

    public ExtendMacro extendMacro = ExtendMacro.NONE;

    @Config
    public static class rotPID {
        public static double kP = 0.000001;
        public static double kI = 0.0;
        public static double kD = 0.00001;
    }


    //TODO: make final
    public CustomPID rotationHoldPID = new CustomPID(rotPID.kP, rotPID.kI, rotPID.kD);

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

    public void reverseExt(){
        slideExt.setDirection(DcMotor.Direction.REVERSE);
    }

    public void moveSlide(double inputRot, double inputExt) {

       if(!justStopped && inputRot == 0 && rotateMacro == RotateMacro.NONE) {
           justStopped = true;
           holdDelay = System.nanoTime();
           lastRotPosition = slideRot.getCurrentPosition();//only set last rot position right after stopping
       }

        if(inputRot == 0 && rotateMacro == RotateMacro.NONE && (System.nanoTime()-holdDelay)>0.5){
            double power = rotationHoldPID.cycleController(lastRotPosition, slideExt.getCurrentPosition());
            inputRot = Range.clip(power, -1.0, 1.0);
        }
        else{
            slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastRotPosition = slideRot.getCurrentPosition();
            justStopped = false;
        }

        if(hanging && slideExt.getCurrentPosition() > (extMin + 50)){
           inputExt = -1.0;
       }

        if(slideExt.getCurrentPosition()> extMaxLow && inputExt > 0 && slideRot.getCurrentPosition() - rotMin > -2500){
            inputExt = 0;
        }

        else if (slideExt.getCurrentPosition()>extMaxHigh && inputExt > 0){
            inputExt = 0;
        }

        if(slideRot.getCurrentPosition() < rotMax){
//            inputRot = 0;
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

    public void teleReset(){
        rotMin = slideRot.getCurrentPosition() + Arm.rot_hor;
        rotMax = rotMin - 4000;
        extMin = slideExt.getCurrentPosition() - Arm.ext_home;
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
                teleReset();
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


        if (gamepad2.y) {
            rotateMacro = RotateMacro.GOING_UP;
            slideRot.setTargetPosition(rotMin - Arm.tele_rot_hang);
            slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRot.setPower(1);
        }
//        if (gamepad2.dpad_down) {
//            rotateMacro = RotateMacro.GOING_DOWN;
//            slideRot.setTargetPosition(rotMin-Arm.tele_rot_sub);
//            slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideRot.setPower(1);
//        }
        if(gamepad2.a){
            rotateMacro = RotateMacro.GOING_HORIZONTAL;
            slideRot.setTargetPosition(rotMin-Arm.rot_hor);
            slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRot.setPower(1);
        }

        if(gamepad2.x || gamepad2.b || (Math.abs(slideRot.getCurrentPosition()  - slideRot.getTargetPosition()) < 10)
                || gamepad2.back) {
            rotateMacro = RotateMacro.NONE;
            slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRot.setPower(0);
        }


    }

    private void handleExtendMacros(Gamepad gamepad2){
        //up for up, down for down, right for horizontal, left for stop
        //TODO: make these values based on the Arm config class

        if (gamepad2.dpad_up) {
            if(!last_d_pad_up) {
                extendMacro = ExtendMacro.HANG;

                if (slideExt.getCurrentPosition() < extMin + Arm.ext_hang - 100) {
                    slideExt.setTargetPosition(extMin + Arm.ext_hang);
                }
                else {
                    slideExt.setTargetPosition(extMin + Arm.ret_hang);
                }
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideExt.setPower(1);
            }
        }
        last_d_pad_up = gamepad2.dpad_up;

//        if (gamepad2.a) {
//            extendMacro = ExtendMacro.MIN;
//            slideExt.setTargetPosition(extMin + Arm.ext_home);
//            slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideExt.setPower(1);
//        }
        if(gamepad2.dpad_down || extendMacro == ExtendMacro.GRAB){
            extendMacro = ExtendMacro.GRAB;

            //start extending when rotation reaches horizontal
            if(rotateMacro == RotateMacro.NONE){
                slideExt.setTargetPosition(extMin + Arm.ext_grab);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideExt.setPower(0.8);
            }

        }

        if(gamepad2.dpad_left || gamepad2.dpad_right || (Math.abs(slideExt.getCurrentPosition()  - slideExt.getTargetPosition()) < 10)
                || gamepad2.back) {
            extendMacro = ExtendMacro.NONE;
            slideExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideExt.setPower(0);
        }

    }

    //lift arm up to hang
    public class RotateUp implements Action {
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideRot.setTargetPosition(rotMin + Arm.rot_hang);
                slideRot.setPower(1);
                slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return Math.abs(slideRot.getTargetPosition() - slideRot.getCurrentPosition()) > 10;
        }
    }

    public class RotateLock implements Action {
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            if(!initialized){
                slideRot.setTargetPosition(rotMin + Arm.rot_lock);
                slideRot.setPower(1);
                slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return Math.abs(slideRot.getTargetPosition() - slideRot.getCurrentPosition()) > 10;
        }
    }

    //extend arm to hang
    public class ExtendToHang implements Action{
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideExt.setTargetPosition(extMin + Arm.ext_hang);
                slideExt.setPower(1);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;

            }
            return Math.abs(slideExt.getTargetPosition() - slideExt.getCurrentPosition()) > 10;
        }
    }

    //lower arm to grabbing
    public class RotateHorizontal implements Action{
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideRot.setTargetPosition(rotMin + Arm.rot_hor);
                slideRot.setPower(1);
                slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return Math.abs(slideRot.getTargetPosition() - slideRot.getCurrentPosition()) > 10;
        }
    }

    public class RotateToDrag implements Action{
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideRot.setTargetPosition(rotMin + Arm.rot_drag);
                slideRot.setPower(1);
                slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return Math.abs(slideRot.getTargetPosition() - slideRot.getCurrentPosition()) > 10;
        }
    }

    //retract arm for Hang
    public class RetractToHang implements Action{
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideExt.setTargetPosition(extMin + Arm.ret_hang);
                slideExt.setPower(1);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;

            }
            return Math.abs(slideExt.getTargetPosition() - slideExt.getCurrentPosition()) > 10;
        }
    }

    //extend arm for grabbing
    public class ExtendToGrab implements Action{
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideExt.setTargetPosition(extMin + Arm.ext_grab);
                slideExt.setPower(1);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return Math.abs(slideExt.getTargetPosition() - slideExt.getCurrentPosition()) > 10;
        }
    }

    public class SlowExtendToGrab implements Action{
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideExt.setTargetPosition(extMin + Arm.ext_grab);
                slideExt.setPower(Arm.slow_ext_speed);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return Math.abs(slideExt.getTargetPosition() - slideExt.getCurrentPosition()) > 10;
        }
    }

    public class ExtendToHome implements Action{
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideExt.setTargetPosition(extMin + Arm.ext_home);
                slideExt.setPower(1);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return Math.abs(slideExt.getTargetPosition() - slideExt.getCurrentPosition()) > 10;
        }
    }

    public class ExtendToDrag implements Action{
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideExt.setTargetPosition(extMin + Arm.ext_drag);
                slideExt.setPower(1);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return Math.abs(slideExt.getTargetPosition() - slideExt.getCurrentPosition()) > 10;
        }
    }

    public class RetractToDrag implements Action {
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized){
                slideExt.setTargetPosition(extMin + Arm.drag_retract);
                slideExt.setPower(1);
                slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            return Math.abs(slideExt.getTargetPosition() - slideExt.getCurrentPosition()) > 10;
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
    public Action rotateToDrag(){
        return new RotateToDrag();
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
    public Action extendToDrag(){
        return new ExtendToDrag();
    }
    public Action openGripper(){
        return new OpenGripper();
    }
    public Action closeGripper(){
        return new CloseGripper();
    }
    public Action extendToHome(){
        return new ExtendToHome();
    }
    public Action rotateLock(){ return new RotateLock();}
    public Action slowExtendToGrab(){return new SlowExtendToGrab();}
    public Action retractToDrag(){return new RetractToDrag();}
    @Config
    public static class Arm{
        public static int ext_hang = 1500;
        public static int ret_hang = 600;
        public static int ext_home = 400;
        public static int ext_grab = 1500;
        public static int rot_hang = 2700;
        public static int rot_lock = 2500;
        public static int rot_hor = 1000;
        public static int rot_home = 400;
        public static int tele_rot_sub = 200;
        public static int tele_rot_hor = 1300;
        public static int tele_rot_hang = 3350;
        public static int tele_ext_hang = 1400;
        public static int rot_drag = 480;
        public static int ext_drag = 3779;
        public static double slow_ext_speed = 0.5;
        public static int drag_retract = 3000;
    }

}