package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CompShooterConfig extends ShooterConfig {
	public static String shooterString = "Shooter/BO";
	public static String intakeString = "Intake/RO";
	public static String triggerString = "Trigger";

	public static double[] shooterSpeeds = {0.8, 0.75, 0.7};
	public static double[] shooterVoltages = {11, 11, 11};

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
}
