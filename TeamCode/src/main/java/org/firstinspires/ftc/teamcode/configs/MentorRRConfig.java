package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class MentorRRConfig extends RRConfig {
	public static String rightOdomString = "RO";
	public static String leftOdomString = MentorDriveConfig.frontLeftDriveString;
	public static String frontOdomString = MentorDriveConfig.frontRightDriveString;

	public String[] getOdomStrings() {
		return new String[] {
				rightOdomString,
				leftOdomString,
				frontOdomString
		};
	}

	public static boolean rightOdomReverse = true;
	public static boolean leftOdomReverse = false;
	public static boolean frontOdomReverse = true;

	public boolean[] getOdomReversals() {
		return new boolean[] {
				rightOdomReverse,
				leftOdomReverse,
				frontOdomReverse
		};
	}

	public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
	public static RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

	public RevHubOrientationOnRobot.LogoFacingDirection getLogoDirection() {
		return logoDirection;
	}

	public RevHubOrientationOnRobot.UsbFacingDirection getUSBDirection() {
		return usbDirection;
	}

	// drive model parameters
	public static double inPerTick = (112 - 12) / 33846.0;
	public static double lateralInPerTick = 0.0023261044003389103;
	public static double trackWidthTicks = 3719.4989843068233;

	public double[] getDriveModelParameters() {
		return new double[] {
			inPerTick,
			lateralInPerTick,
			trackWidthTicks
		};
	}

	// feedforward parameters (in tick units)
	public static double kS = 1.428796935440296;
	public static double kV = 0.0005975382084278895;
	public static double kA = 0.00013;

	public double[] getFeedforwardParameters() {
		return new double[] {
			kS,
			kV,
			kA
		};
	}

	// path profile parameters (in inches)
	public static double maxWheelVel = 50;
	public static double minProfileAccel = -30;
	public static double maxProfileAccel = 50;

	public double[] getPathProfileParameters() {
		return new double[] {
			maxWheelVel,
			minProfileAccel,
			maxProfileAccel
		};
	}

	// turn profile parameters (in radians)
	public static double maxAngVel = Math.PI; // shared with path
	public static double maxAngAccel = Math.PI;

	public double[] getTurnProfileParameters() {
		return new double[] {
			maxAngVel,
			maxAngAccel
		};
	}

	// path controller gains
	public static double axialGain = 3;
	public static double lateralGain = 2.5;
	public static double headingGain = 3.5; // shared with turn

	public static double axialVelGain = 0.0;
	public static double lateralVelGain = 0.0;
	public static double headingVelGain = 0.0; // shared with turn

	public double[] getPathControllerGains() {
		return new double[] {
			axialGain,
			lateralGain,
			headingGain,
			axialVelGain,
			lateralVelGain,
			headingVelGain
		};
	}

	//Dead Wheel Localizer
	public static double par0YTicks = 2052.769511678376; // y position of the first parallel encoder (in tick units)
	public static double par1YTicks = -1904.9188041808352; // y position of the second parallel encoder (in tick units)
	public static double perpXTicks = 1718.5269172557105; // x position of the perpendicular encoder (in tick units)

	public double[] getLocalizerVals() {
		return new double[] {
				par0YTicks,
				par1YTicks,
				perpXTicks
		};
	}
}
