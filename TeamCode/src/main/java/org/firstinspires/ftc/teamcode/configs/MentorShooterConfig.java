package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MentorShooterConfig extends ShooterConfig {
	public static String intakeString = "intake";
	public static String shooterString = "shooter";
	public static String shooterDeliverString = "shooterDeliver";
	public static String carouselDeliverString = "carouselDeliver";
	public static String carouselString = "carousel";

	@Override
	public String getShooterString() {
		return shooterString;
	}

	public static boolean shooterReversed = true;

	public boolean getShooterReversed() {
		return shooterReversed;
	}

	public static double[] shooterSpeeds = {0.8, 0.75, 0.7};
	public static double[] shooterVoltages = {11, 11, 11};

	public static double[] carouselPositions = {0, 0.5, 1};
	public static double[] carouselDeliverPositions = {0, 0.5, 1};
	public static double[] shooterDeliverPositions = {0, 0.5};

	public static long intakeTime = 2000;

	public double[] getShooterSpeeds() {
		return shooterSpeeds;
	}

	public double[] getShooterVoltages() {
		return shooterVoltages;
	}
}
