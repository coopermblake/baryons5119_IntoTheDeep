package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.AutoDrive;

public class Robot {
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;

    public DcMotor slideExt;
    public DcMotor slideRot;

    public Servo gripper;
    public IMU imu;

    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public Drivetrain drivetrain;
    public ViperSlide viperSlide;
    public AutoDrive autoDrive;

    public Robot(HardwareMap HardwareMap, Gamepad Gamepad1, Gamepad Gamepad2) {
        gamepad1 = Gamepad1;
        gamepad2 = Gamepad2;
        backLeft = HardwareMap.get(DcMotor.class, "backLeft");
        backRight = HardwareMap.get(DcMotor.class, "backRight");
        frontLeft = HardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = HardwareMap.get(DcMotor.class, "frontRight");
        slideExt = HardwareMap.get(DcMotor.class, "slideExt");// max-min = +8k
        slideRot = HardwareMap.get(DcMotor.class, "slideRot"); // max-min = +5k
        gripper = HardwareMap.get(Servo.class, "gripper");
        imu = HardwareMap.get(IMU.class, "imu");

        slideExt.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRot.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LogoFacingDirection.UP,
                UsbFacingDirection.RIGHT)));

        drivetrain = new Drivetrain(backLeft, backRight, frontLeft, frontRight);
        viperSlide = new ViperSlide(slideExt, slideRot, gamepad1, gamepad2, gripper);

    }

    public double getYawDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getYawRadians(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    public void initAutoDrive(Telemetry telemetry){
        autoDrive = new AutoDrive(backLeft, backRight, frontLeft, frontRight, imu, telemetry);
    }
}