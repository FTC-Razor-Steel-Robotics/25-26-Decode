package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class CompRRConfig extends RRConfig {
	public static String rightOdomString = CompShooterConfig.intakeString;
	public static String leftOdomString = CompDriveConfig.frontLeftDriveString;
	public static String backOdomString = CompShooterConfig.shooterString;

	public String[] getOdomStrings() {
		return new String[] {
				rightOdomString,
				leftOdomString,
				backOdomString
		};
	}

	public static boolean rightOdomReverse = true;
	public static boolean leftOdomReverse = true;
	public static boolean backOdomReverse = true;

	public boolean[] getOdomReversals() {
		return new boolean[] {
				rightOdomReverse,
				leftOdomReverse,
				backOdomReverse
		};
	}

	public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
	public static RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

	public RevHubOrientationOnRobot.LogoFacingDirection getLogoDirection() {
		return logoDirection;
	}

	public RevHubOrientationOnRobot.UsbFacingDirection getUSBDirection() {
		return usbDirection;
	}

	// drive model parameters
	public static double inPerTick = 120 / 61000.0;
	public static double lateralInPerTick = 89.29099050274306;
	public static double trackWidthTicks = 6150.0861944927665;

	public double[] getDriveModelParameters() {
		return new double[] {
				inPerTick,
				lateralInPerTick,
				trackWidthTicks
		};
	}

	// feedforward parameters (in tick units)
	public static double kS = 0.9775425545276475;
	public static double kV = 0.0003677962896007073;
	public static double kA = 0.0001;

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
	public static double axialGain = 4;
	public static double lateralGain = .5;
	public static double headingGain = 1; // shared with turn

	public static double axialVelGain = 2.0;
	public static double lateralVelGain = 0.5;
	public static double headingVelGain = 1.0; // shared with turn

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
	public static double par0YTicks = 1; // y position of the first parallel encoder (in tick units)
	public static double par1YTicks = 1; // y position of the second parallel encoder (in tick units)
	public static double perpXTicks = 1; // x position of the perpendicular encoder (in tick units)

	public double[] getLocalizerVals() {
		return new double[] {
				par0YTicks,
				par1YTicks,
				perpXTicks
		};
	}
}
