package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (robot.imu) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal robot.imu interface so it will work with either the BNO055, or BHI260 robot.imu.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

public class AutoDrive {

    private final Telemetry telemetry;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;

    private final IMU imu;
    private double headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;

    //will need to remove leftSpeed, rightSpeed, leftTarget, rightTarget and add these on
    private double strafeSpeed   = 0;
    private double frontLeftSpeed = 0;
    private double frontRightSpeed = 0;
    private double backLeftSpeed = 0;
    private double backRightSpeed = 0;
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backLeftTarget = 0;
    private int backRightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;   // need to make sure this is correct for 435 rpm yellow jacket motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.09 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static double STRAFE_EFFICIENCY_FACTOR = 0.90;
    static final double     STRAFE_COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI * STRAFE_EFFICIENCY_FACTOR)); //change to the right factor

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.03;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.0001;     // Larger is more responsive, but also less stable.
    static final double     P_STRAFE_GAIN           = 0.02;     // Larger is more responsive, but also less stable.

    private enum Motion{
        STRAIGHT,
        STRAFE,
        TURN,
        PID,
        NONE
    }

    private Motion currentMotion = Motion.NONE;

    //Initializing custom PID instances
    private CustomPID pidX = new CustomPID(0.02, 0, 0.05); // Adjust coefficients
    public CustomPID pidY = new CustomPID(0.005, 0, 0); // Adjust coefficients, TODO: MAKE PRIVATE AGAIN
    //kP = 0.05, kl = 0.01, kD = 0.01 - setting to zero to make sure driveforward, strafe, and drivediagonal work


    public AutoDrive(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, IMU imu, Telemetry telemetry){

        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.imu = imu;
        this.telemetry = telemetry;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed-loop speed control, and reset the heading
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();
    }


    // Reverse movement is obtained by setting a negative distance (not speed)
    public void driveStraightPID(double speed, double distance, double heading) {
        double targetTicksY = distance * COUNTS_PER_INCH;
        
        int frontLeftInitial = frontLeft.getCurrentPosition();
        int frontRightInitial = frontRight.getCurrentPosition();
        int backLeftInitial = backLeft.getCurrentPosition();
        int backRightInitial = backRight.getCurrentPosition();

        frontLeftTarget = (int)(frontLeft.getCurrentPosition() + targetTicksY);
        frontRightTarget = (int)(frontRight.getCurrentPosition() + targetTicksY);
        backLeftTarget = (int)(backLeft.getCurrentPosition() + targetTicksY);
        backRightTarget = (int)(backRight.getCurrentPosition() + targetTicksY);

        pidY.updateController(); // Reset PID controller
        currentMotion = Motion.PID;
        

        while (Math.abs(frontLeft.getCurrentPosition() - frontLeftTarget) > 10 || 
                Math.abs(frontRight.getCurrentPosition() - frontRightTarget) > 10 ||
                Math.abs(backRight.getCurrentPosition() - backRightTarget) > 10 ||
                Math.abs(backLeft.getCurrentPosition() - backLeftTarget) > 10 ) {
            
            // Get current encoder values for Y direction
            double currentY = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() -
                    frontLeftInitial - frontRightInitial) / 2.0;

            // Compute PID output for straight driving
            double frontLeftPID = pidY.cycleController(frontLeftTarget, frontLeft.getCurrentPosition()) * speed;
            double frontRightPID = pidY.cycleController(frontRightTarget, frontRight.getCurrentPosition()) * speed;
            double backRightPID = pidY.cycleController(backRightTarget, backRight.getCurrentPosition()) * speed;
            double backLeftPID = pidY.cycleController(backLeftTarget, backLeft.getCurrentPosition()) * speed;
            
            // Add heading correction
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // Apply motor powers
            frontLeftSpeed = Range.clip(frontLeftPID + turnSpeed, -1.0, 1.0);
            frontRightSpeed = Range.clip(frontRightPID - turnSpeed, -1.0, 1.0);
            backLeftSpeed = Range.clip(backLeftPID + turnSpeed, -1.0, 1.0);
            backRightSpeed = Range.clip(backRightPID - turnSpeed, -1.0, 1.0);

            frontLeft.setPower(frontLeftSpeed);
            frontRight.setPower(frontRightSpeed);
            backLeft.setPower(backLeftSpeed);
            backRight.setPower(backRightSpeed);

            // Telemetry for debugging
            telemetry.addData("Motion", "Straight");
            telemetry.addData("Target Y:Current Y", "%.2f:%.2f", targetTicksY, currentY);
            telemetry.update();
            sendTelemetry();
        }

        // Stop motors
        moveRobot(0, 0, 0);
        currentMotion = Motion.NONE;
        sendTelemetry();
    }
    
    public void driveStraightEncoder(double speed, double distance, double heading) {
        double targetTicksY = distance*COUNTS_PER_INCH;
        
        frontLeftTarget = (int) (frontLeft.getCurrentPosition() + targetTicksY);
        frontRightTarget = (int) (frontRight.getCurrentPosition() + targetTicksY);
        backRightTarget = (int) (backRight.getCurrentPosition() + targetTicksY);
        backLeftTarget = (int) (backLeft.getCurrentPosition() + targetTicksY);
        
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backRight.setTargetPosition(backRightTarget);
        backLeft.setTargetPosition(backLeftTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentMotion = Motion.STRAIGHT;

        while(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()){
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
            moveRobot(speed, turnSpeed, 0);
            sendTelemetry();
        }

        currentMotion = Motion.NONE;
        moveRobot(0,0,0);
        sendTelemetry();
        
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    
    public void driveStrafeEncoder(double speed, double distance, double heading){
        double targetTicksX = distance*STRAFE_COUNTS_PER_INCH;

        frontLeftTarget = (int) (frontLeft.getCurrentPosition() + targetTicksX);
        frontRightTarget = (int) (frontRight.getCurrentPosition() - targetTicksX);
        backRightTarget = (int) (backRight.getCurrentPosition() + targetTicksX);
        backLeftTarget = (int) (backLeft.getCurrentPosition() - targetTicksX);

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backRight.setTargetPosition(backRightTarget);
        backLeft.setTargetPosition(backLeftTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentMotion = Motion.STRAFE;

        while(frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            turnSpeed = getSteeringCorrection(heading, P_STRAFE_GAIN);
            moveRobot(0, turnSpeed, speed);
            sendTelemetry();

        }

        moveRobot(0,0,0);
        currentMotion = Motion.NONE;
        sendTelemetry();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveStrafePID(double speed, double distance, double heading) {
        double targetTicksX = distance * STRAFE_COUNTS_PER_INCH;

        int frontLeftInitial = frontLeft.getCurrentPosition();
        int frontRightInitial = frontRight.getCurrentPosition();
        int backLeftInitial = backLeft.getCurrentPosition();
        int backRightInitial = backRight.getCurrentPosition();

        frontLeftTarget = (int)(frontLeft.getCurrentPosition() + targetTicksX);
        frontRightTarget = (int)(frontRight.getCurrentPosition() - targetTicksX);
        backLeftTarget = (int)(backLeft.getCurrentPosition() - targetTicksX);
        backRightTarget = (int)(backRight.getCurrentPosition() + targetTicksX);

        pidX.updateController(); // Reset PID controller

        while (Math.abs(frontLeft.getCurrentPosition() - frontLeftTarget) > 10 ||
                Math.abs(frontRight.getCurrentPosition() - frontRightTarget) > 10 ||
                Math.abs(backRight.getCurrentPosition() - backRightTarget) > 10 ||
                Math.abs(backLeft.getCurrentPosition() - backLeftTarget) > 10 ) {

            // Get current encoder values for Y direction
            double currentX = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition() -
                    frontLeftInitial - backRightInitial) / 2.0;

            // Compute PID output for straight driving
            double frontLeftPID = pidX.cycleController(frontLeftTarget, frontLeft.getCurrentPosition()) * speed;
            double frontRightPID = pidX.cycleController(frontRightTarget, frontRight.getCurrentPosition()) * speed;
            double backRightPID = pidX.cycleController(backRightTarget, backRight.getCurrentPosition()) * speed;
            double backLeftPID = pidX.cycleController(backLeftTarget, backLeft.getCurrentPosition()) * speed;

            // Add heading correction
            turnSpeed = getSteeringCorrection(heading, P_STRAFE_GAIN);

            // Apply motor powers
            frontLeftSpeed = Range.clip(frontLeftPID + turnSpeed, -1.0, 1.0);
            frontRightSpeed = Range.clip(frontRightPID - turnSpeed, -1.0, 1.0);
            backLeftSpeed = Range.clip(backLeftPID + turnSpeed, -1.0, 1.0);
            backRightSpeed = Range.clip(backRightPID - turnSpeed, -1.0, 1.0);

            frontLeft.setPower(frontLeftSpeed);
            frontRight.setPower(frontRightSpeed);
            backLeft.setPower(backLeftSpeed);
            backRight.setPower(backRightSpeed);

            // Telemetry for debugging
            telemetry.addData("Motion", "Straight");
            telemetry.addData("Target Y:Current Y", "%.2f:%.2f", targetTicksX, currentX);
            telemetry.update();
            sendTelemetry();

        }

        // Stop motors
        moveRobot(0, 0, 0);
        currentMotion = Motion.NONE;
        sendTelemetry();
    }

    public void driveDiagonal(double speed, double distanceX, double distanceY, double timeoutSeconds) {
        double targetTicksX = distanceX * STRAFE_COUNTS_PER_INCH;
        double targetTicksY = distanceY * COUNTS_PER_INCH;

        pidX.updateController(); // Reset PID controllers
        pidY.updateController();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < timeoutSeconds) {
            // Get current encoder values
            double currentX = (frontLeft.getCurrentPosition() - frontRight.getCurrentPosition()) / 2.0;
            double currentY = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 2.0;

            // Compute PID outputs for diagonal driving
            double powerX = pidX.cycleController(targetTicksX, currentX) * speed;
            double powerY = pidY.cycleController(targetTicksY, currentY) * speed;

            // Apply motor powers
            double frontLeftPower = Range.clip(powerY + powerX, -1.0, 1.0);
            double backLeftPower = Range.clip(powerY - powerX, -1.0, 1.0);
            double frontRightPower = Range.clip(powerY - powerX, -1.0, 1.0);
            double backRightPower = Range.clip(powerY + powerX, -1.0, 1.0);

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Diagonal Move", "Target X: %.2f, Target Y: %.2f", targetTicksX, targetTicksY);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Power X", powerX);
            telemetry.addData("Power Y", powerY);
            telemetry.update();

            // Exit loop if both errors are within acceptable ranges
            if (Math.abs(targetTicksX - currentX) < 10 && Math.abs(targetTicksY - currentY) < 10) {
                break;
            }
        }

        // Stop motors
        moveRobot(0, 0, 0);
    }

    public void turnToHeadingPID(double speed, double heading) {
        //TODO: fix this
        pidX.updateController();
        double MIN_TURN_POWER = 0.0; // Starting with zero

        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        while ((Math.abs(getHeading() - heading) > HEADING_THRESHOLD) &&
                (turnTimer.seconds() < 5.0)) {
            double currentHeading = getHeading();
            double turnPower = pidX.cycleController(heading, currentHeading) * speed;

            if (Math.abs(turnPower) < 0.02) {
                turnPower = Math.copySign(0.02, turnPower);
            }

            turnPower = Range.clip(turnPower, -1.0, 1.0);
            moveRobot(0, turnPower, 0);

            telemetry.addData("turnToHeadingPID", "Target: %.2f, Current: %.2f", heading, currentHeading);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }

        moveRobot(0, 0, 0);
    }

    public void turnToHeadingEncoder(double maxTurnSpeed, double heading) {


        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        currentMotion = Motion.TURN;

        // Keep looping while OpMode is active and the heading error is greater than the threshold
        while ((Math.abs(headingError) > HEADING_THRESHOLD)) {


            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);


            // Clip the turn speed to the maximum permitted value
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);


            // Pivot in place by applying the turning correction to all motors
            moveRobot(0, turnSpeed, 0); // 0 for no forward/backward motion


            // Display drive status for the driver
            sendTelemetry();
        }


        // Stop all motion
        moveRobot(0, 0, 0);
        currentMotion = Motion.NONE;
        sendTelemetry();

    }


    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        currentMotion = Motion.TURN;
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // Keep looping while OpMode is active and we have time remaining
        while ((holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Apply turning correction to all four motors
            moveRobot(0, turnSpeed,0); // No forward/backward or strafing, only turning

            // Display drive status for the driver
            sendTelemetry();
        }

        // Stop all motion
        moveRobot(0, 0,0);
        currentMotion = Motion.NONE;
        sendTelemetry();

    }

    // **********  LOW Level driving functions.  ********************

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
                           //90

        // Determine the heading current error
        headingError = targetHeading - getHeading();

                       //90-0

        // Normalize the error to be within +/- 180 degrees
        headingError = normalizeHeading(headingError);

        // Calculate the turning power using proportional gain
        double turnPower = -headingError * proportionalGain;

        // Clip the turning power to ensure it stays within the range [-1, 1]
        return Range.clip(turnPower, -1.0, 1.0);
    }

    /**
     * Normalize a heading error to be within the range [-180, 180] degrees.
     *
     * @param error  The heading error in degrees.
     * @return       The normalized heading error.
     */
    private double normalizeHeading(double error) {
        if (error > 180) {
            error -= 360;
        } else if (error <= -180) {
            error += 360;
        }
        return error;
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the four wheel motors.
     * @param drive forward/reverse motor speed
     * @param turn  clockwise/counterclockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn, double strafe) {
        driveSpeed = drive;     // Save this value as a class member for telemetry
        turnSpeed  = turn;      // Save this value as a class member for telemetry
        strafeSpeed = strafe;   // Save this value as a class member for telemetry

        // Calculate motor powers for mecanum drive
        frontLeftSpeed = drive + strafe + turn;
        frontRightSpeed = drive - strafe - turn;
        backLeftSpeed = drive - strafe + turn;
        backRightSpeed = drive + strafe - turn;

        // Scale motor powers if any exceed +/- 1.0
        double maxPower = Math.max(Math.abs(frontLeftSpeed),
                Math.max(Math.abs(frontRightSpeed),
                        Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed))));
        if (maxPower > 1.0) {
            frontLeftSpeed /= maxPower;
            frontRightSpeed /= maxPower;
            backLeftSpeed /= maxPower;
            backRightSpeed /= maxPower;
        }

        // Apply the calculated powers to the motors
        frontLeft.setPower(frontLeftSpeed);
        frontRight.setPower(frontRightSpeed);
        backLeft.setPower(backLeftSpeed);
        backRight.setPower(backRightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     *
     */
    //Todo: make this argument simpler with enum
    public void sendTelemetry() {
        if (currentMotion == Motion.STRAIGHT) {
            telemetry.addData("Motion", "Straight");
            telemetry.addData("Target Pos FL:FR:BL:BR:", " %7d:%7d:%7d:%7d",
                    frontLeftTarget, frontRightTarget,
                    backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos FL:FR:BL:BR:", " %7d:%7d:%7d:%7d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        }
        else if(currentMotion == Motion.STRAFE){
            telemetry.addData("Motion", "Strafe");
            telemetry.addData("Target Pos FL:FR:BL:BR:", " %7d:%7d:%7d:%7d",
                    frontLeftTarget, frontRightTarget,
                    backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos FL:FR:BL:BR:", " %7d:%7d:%7d:%7d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        }
        else if(currentMotion == Motion.TURN) {
            telemetry.addData("Motion", "Turning");
        }
        else if(currentMotion == Motion.NONE){
            telemetry.addData("Motion", "None");
            getSteeringCorrection(getHeading(), 0); //set target heading and heading error for telemetry
        }
        else if(currentMotion == Motion.PID){
            //is handled in PID drive methods
        }

        telemetry.addData("Heading - Target : Current:", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error : Steer Pwr:", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData( "Wheel Speeds FL:FR:BL:BR:", " %5.2f : %5.2f : %5.2f : %5.2f",
                frontLeftSpeed, frontRightSpeed,
                backLeftSpeed, backRightSpeed);
        telemetry.update();
    }

    /**
     * Reads the robot's heading directly from the IMU in degrees.
     *
     * @return The current yaw angle (heading) in degrees. Returns 0 if IMU data is unavailable.
     */
    public double getHeading() {
        try {
            // Retrieve the yaw (heading) angle from the IMU
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);
        } catch (Exception e) {
            // Handle any errors (e.g., if the IMU is not initialized)
//            telemetry.addData("IMU Error", "Unable to retrieve heading: " + e.getMessage());
//            telemetry.update();
            return 0.0; // Default to 0 degrees if IMU data is unavailable
        }
    }
}