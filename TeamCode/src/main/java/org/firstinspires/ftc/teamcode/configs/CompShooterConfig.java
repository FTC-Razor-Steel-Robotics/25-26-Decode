package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CompShooterConfig extends ShooterConfig {
	public static String shooterString = "Shooter/BO";
	public static String intakeString = "Intake/RO";
	public static String triggerString = "Trigger";
	public static String guardString = "Guard";

	public static boolean shooterReversed = true;

	public boolean getShooterReversed() {
		return shooterReversed;
	}

	public static double[] shooterSpeeds = {0.875, 0.8, 0.75};
	public static double[] shooterVoltages = {12.5, 12.5, 12.5};

	public static double speedOverDistance = 0.012228260869565;
	public static double speedIntercept = 9.372282608695652;
	public static double testDistance = 40;

	public double getSpeedOverDistance() {
		return speedOverDistance;
	}

	public double getSpeedIntercept() {
		return speedIntercept;
	}

	public static double triggerUpPos = 1;
	public static double triggerDownPos = 0;

	public String getShooterString() {
		return shooterString;
	}

	public double[] getShooterSpeeds() {
		return shooterSpeeds;
	}

	public double[] getShooterVoltages() {
		return shooterVoltages;
	}

	public static double kP = 0.03;
	public static double kI = 0;
	public static double kD = 0.3;
}
