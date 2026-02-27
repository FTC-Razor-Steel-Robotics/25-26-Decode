package org.firstinspires.ftc.teamcode.configs;

public class PIDController {

	private double kP;
	private double kI;
	private double kD;

	private double setpoint;

	private double integral;
	private double previousError;

	private double minOutput = Double.NEGATIVE_INFINITY;
	private double maxOutput = Double.POSITIVE_INFINITY;

	private double integralMin = Double.NEGATIVE_INFINITY;
	private double integralMax = Double.POSITIVE_INFINITY;

	private boolean firstRun = true;

	public PIDController(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}

	public double getSetpoint() {
		return setpoint;
	}

	public void setOutputLimits(double min, double max) {
		this.minOutput = min;
		this.maxOutput = max;
	}

	public void setIntegralLimits(double min, double max) {
		this.integralMin = min;
		this.integralMax = max;
	}

	public void reset() {
		integral = 0;
		previousError = 0;
		firstRun = true;
	}

	public double calculate(double measurement, double deltaTime) {
		if (deltaTime <= 0) {
			throw new IllegalArgumentException("deltaTime must be > 0");
		}

		double error = setpoint - measurement;

		// Proportional
		double proportional = kP * error;

		// Integral with anti-windup clamp
		integral += error * deltaTime;
		integral = clamp(integral, integralMin, integralMax);
		double integralTerm = kI * integral;

		// Derivative (avoid derivative kick on first run)
		double derivative = 0;
		if (!firstRun) {
			derivative = (error - previousError) / deltaTime;
		} else {
			firstRun = false;
		}
		double derivativeTerm = kD * derivative;

		previousError = error;

		double output = proportional + integralTerm + derivativeTerm;

		return clamp(output, minOutput, maxOutput);
	}

	private double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}

	// Optional: getters and setters for gains
	public void setPID(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public double getKP() { return kP; }
	public double getKI() { return kI; }
	public double getKD() { return kD; }
}