package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class DoublePivotArm {

    private final DcMotor pivot1;
    private final DcMotor pivot2;
    private final Servo gripper;
    private final Gamepad gamepad;

    private static final int TICKS_PER_DEGREE = 10; // Adjust for your gear ratio + encoder setup
    private boolean manualControl = true;

    public DoublePivotArm(DcMotor pivot1, DcMotor pivot2, Servo gripper, Gamepad gamepad) {
        this.pivot1 = pivot1;
        this.pivot2 = pivot2;
        this.gripper = gripper;
        this.gamepad = gamepad;

        pivot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void manualControl() {
        double shoulderInput = -gamepad.left_stick_y;
        double elbowInput = -gamepad.right_stick_y;

        shoulderInput = Math.abs(shoulderInput) > 0.05 ? shoulderInput : 0;
        elbowInput = Math.abs(elbowInput) > 0.05 ? elbowInput : 0;

        pivot1.setPower(shoulderInput);
        pivot2.setPower(elbowInput);

        // Example gripper toggle (simple):
        if (gamepad.a) {
            gripper.setPosition(0.3); // Open
        } else if (gamepad.b) {
            gripper.setPosition(0.7); // Closed
        }
    }

    public void moveToAngles(double shoulderAngle, double elbowAngle) {
        int shoulderTarget = (int) (shoulderAngle * TICKS_PER_DEGREE);
        int elbowTarget = (int) (elbowAngle * TICKS_PER_DEGREE);

        pivot1.setTargetPosition(shoulderTarget);
        pivot2.setTargetPosition(elbowTarget);

        pivot1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivot1.setPower(1);
        pivot2.setPower(1);
    }

    public void stop() {
        pivot1.setPower(0);
        pivot2.setPower(0);
        pivot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        if (manualControl) {
            manualControl();
        }
    }

    public void toggleManualControl(boolean enabled) {
        manualControl = enabled;
    }
}