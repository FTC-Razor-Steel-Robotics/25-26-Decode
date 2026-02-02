package org.firstinspires.ftc.teamcode.configs;

public abstract class ShooterConfig {
	public abstract String getShooterString();
	public abstract double[] getShooterSpeeds();
	public abstract double[] getShooterVoltages();
	//TODO: Replace this with an equation once we get the camera working
	public double[] getShooterSpeedsCompensated() {
		double[] shooterSpeeds = getShooterSpeeds();
		double[] shooterVoltages = getShooterVoltages();

		int numSpeeds = shooterSpeeds.length;
		double[] compensated = new double[numSpeeds];

		for (int i = 0; i < numSpeeds; i++) {
			compensated[i] = shooterSpeeds[i] * shooterVoltages[i];
		}

		return compensated;
	}
}
