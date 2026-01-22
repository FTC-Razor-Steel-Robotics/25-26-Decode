package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CompDriveConfig extends DriveConfig {
	public static String frontLeftDriveString = "FL/LO";
	public static String backLeftDriveDriveString = "RL";
	public static String frontRightDriveString = "FR/RO";
	public static String backRightDriveString = "RR/BO";

	public String[] getDriveStrings() {
		return new String[0];
	}

	public boolean[] getDriveReversals() {
		return new boolean[0];
	}
}
