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

	public static double[] shooterSpeeds = {0.645, 0.55, 0.5};
	public static double[] shooterVoltages = {12.5, 12.5, 12.5};

	public static double triggerUpPos = 1;
	public static double triggerDownPos = 0;
    public static double GuardUp=1;
    public static double GuardDown=0;
    public static double guardDelay = 150;

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
