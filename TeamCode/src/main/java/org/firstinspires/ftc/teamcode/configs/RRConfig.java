package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public abstract class RRConfig {
	//TODO: Make sure that the parallel odometry wheels are on ports 0 and 3 for best performance
	public abstract String[] getOdomStrings();
	public abstract boolean[] getOdomReversals();
	public abstract RevHubOrientationOnRobot.LogoFacingDirection getLogoDirection();
	public abstract RevHubOrientationOnRobot.UsbFacingDirection getUSBDirection();

	public abstract double[] getDriveModelParameters();
	public abstract double[] getFeedforwardParameters();
	public abstract double[] getPathProfileParameters();
	public abstract double[] getTurnProfileParameters();
	public abstract double[] getPathControllerGains();

	public abstract double[] getLocalizerVals();
}
