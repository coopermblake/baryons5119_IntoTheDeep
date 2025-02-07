package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.CustomPID;

public class Drivetrain {
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private boolean fieldCentric = true;
    private boolean debounce = false;

    private IMU imu;

    @Config
    public static class turnMacroPIDVals {
        public static double kP = 0.000001;
        public static double kI = 0.0;
        public static double kD = 0.0;
    }


    //TODO: make final
    private CustomPID turnMacroPID = new CustomPID(turnMacroPIDVals.kP, turnMacroPIDVals.kI, turnMacroPIDVals.kD);

    private enum TurnMacro {
        NONE,
        TURN_TO_BAR,
        TURN_TO_BREN
    }

    public TurnMacro turnMacro = TurnMacro.NONE;

    public double targetHeading;
    public Drivetrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, IMU imu) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.imu = imu;
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveRobot(Gamepad gamepad1, Gamepad gamepad2, double heading) {
        double[] inputs = getTeleopDriveInputs(gamepad1, gamepad2, heading);
        handleMacros(gamepad1, heading);
        if(turnMacro == TurnMacro.NONE) {
            driveMovement(inputs[0], inputs[1], inputs[2], inputs[3]);
        }
    }

    public double[] getTeleopDriveInputs(Gamepad gamepad1, Gamepad gamepad2, double heading) {
        double inputY = -gamepad1.left_stick_y;
        double inputX = gamepad1.left_stick_x;
        double inputRot = gamepad1.right_stick_x;

        //check buttons
        /*DEBOUNCE LOGIC
        debounce starts false
        after button is pressed, toggle fieldCentric and turn debounce true
        debounce true means nothing will happen until button is released
        after button is released, debounce will go back to false
        ready for next button press
         */
        if(gamepad1.x && !debounce){
            fieldCentric = !fieldCentric;
            debounce = true;
        } else if (!gamepad1.x && debounce) {
            debounce = false;
        }

        if(gamepad1.y){
            imu.resetYaw();
        }

        if(fieldCentric){
            //rotate inputs to align with robot
            double rotX = inputX*Math.cos(-heading) - inputY*Math.sin(-heading);
            double rotY = inputX*Math.sin(-heading) + inputY*Math.cos(-heading);

            inputX = rotX * 1.1;
            inputY = rotY;
        }

        // Divisor is used to scale the other inputs if an input is too large or slow down the robot
        double divisor = Math.max(Math.abs(inputY) + Math.abs(inputX) + Math.abs(inputRot), 1) + gamepad1.right_trigger*3;
//        if (gamepad2.dpad_up) {
//            inputY += 0.25;
//        } else if (gamepad2.dpad_down) {
//            inputY -= 0.25;
//        }
//        if (gamepad2.right_bumper) {
//            inputRot += 0.125;
//        } else if (gamepad2.left_bumper) {
//            inputRot -= 0.125;
//        }




        return new double[] {inputY, inputX, inputRot, divisor};
    }

    public void driveMovement(double inputY, double inputX, double inputRot, double divisor) {
        backLeft.setPower((inputY - inputX + inputRot)/divisor);
        backRight.setPower((inputY + inputX - inputRot)/divisor);
        frontLeft.setPower((inputY + inputX + inputRot)/divisor);
        frontRight.setPower((inputY - inputX - inputRot)/divisor);
    }

    public boolean getFieldCentric(){
        return fieldCentric;
    }

    private void handleMacros(Gamepad gamepad1, double heading){
        if(gamepad1.dpad_up || turnMacro == TurnMacro.TURN_TO_BAR){
            turnMacro = TurnMacro.TURN_TO_BAR;
            heading = Math.toDegrees(heading);
            targetHeading = 0.0;

            double power = turnMacroPID.cycleController(targetHeading, heading);
            power = Range.clip(power, -1.0, 1);

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

        }
        if(gamepad1.dpad_down || turnMacro == TurnMacro.TURN_TO_BREN){
            turnMacro = TurnMacro.TURN_TO_BREN;
            targetHeading = 180 * heading / Math.abs(heading); //copy sign of current heading to target heading

            double power = turnMacroPID.cycleController(targetHeading, heading);
            power = Range.clip(power, -1.0, 1);

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);
        }

        if(Math.abs(heading-targetHeading) < 1.0 || gamepad1.dpad_left || gamepad1.dpad_right){
            turnMacro = TurnMacro.NONE;
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }
}