package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
@Config
public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.002955;
        TwoWheelConstants.strafeTicksToInches = 0.002955;
        TwoWheelConstants.forwardY = -0.5;
        TwoWheelConstants.strafeX = -4.25;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "forwardEncoder";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "strafeEncoder";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE ;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }
}




