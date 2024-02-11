package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Shooter subsystem. PIDF stuff is in rad/s
 */
public class Shooter extends SubsystemBase {
	private static final int kTopShooterCanID = 32;
	private static final int kBottomShooterCanID = 33;
	private static final int kFeederCanID = 31;

	private static final double kPosEncFac = 2.0 * Math.PI;
	private static final double kVelEncFac = (2.0 * Math.PI) / 60.0;

	// Our gainz
	private static final double kSff = 0.0;
	private static final double kVff = 0.12928;
	private static final double kAff = 0.10706;
	private static final double kPpid = 0.00018357;
	private static final double kDpid = 0.0;

	//SparkMax controllers
	private final CANSparkMax mShooterTop = new CANSparkMax(Shooter.kTopShooterCanID, MotorType.kBrushless);
	private final RelativeEncoder mTopEncoder;
	private final CANSparkMax mShooterBottom = new CANSparkMax(Shooter.kBottomShooterCanID, MotorType.kBrushless);
	private final CANSparkMax mFeedMotor = new CANSparkMax(Shooter.kFeederCanID, MotorType.kBrushless);

	private final SimpleMotorFeedforward mTopFeedf = new SimpleMotorFeedforward(kSff, kVff, kAff);
	private final SparkPIDController mTopPid;

	private static final String kMotorSpeedName = "Motor Speed";
	private static final String kFeedSpeedName = "Feed Speed";
	private static final String kSpinupTimeName = "Spinup Time";
	private static final String kStatusName = "Shooter Status";
	private static final String kVelocityName = "Shooter Velocity";

	public Shooter() {
		this.mShooterBottom.follow(mShooterTop, true);

		this.mTopEncoder = this.mShooterTop.getEncoder();
		this.mTopEncoder.setPositionConversionFactor(kPosEncFac);
		this.mTopEncoder.setVelocityConversionFactor(kVelEncFac);

		this.mTopPid = this.mShooterTop.getPIDController();
		this.mTopPid.setP(kPpid);
		this.mTopPid.setI(0.0);
		this.mTopPid.setD(kDpid);

		SmartDashboard.setDefaultNumber(kMotorSpeedName, 0);
		SmartDashboard.setDefaultNumber(kFeedSpeedName, 0);
		SmartDashboard.setDefaultNumber(kSpinupTimeName, 2.0);
		SmartDashboard.setDefaultString(kStatusName, "");
		SmartDashboard.setDefaultNumber(kVelocityName, 0.0);

		this.setDefaultCommand(this.run(() -> {
			this.mShooterTop.stopMotor();
			this.mFeedMotor.stopMotor();
			SmartDashboard.putString(kStatusName, "Stopped shooting");
		}));

		// Print out expected CAN IDs
		System.out.println(
			"Can IDs:" +
			"\n\tTop shooter motor: " + kTopShooterCanID +
			"\n\tBottom shooter motor: " + kBottomShooterCanID +
			"\n\tFeed motor: " + kFeederCanID
		);
	}

	private Command deferredWaitCommand(DoubleSupplier time) {
		return this.defer(() -> new WaitCommand(time.getAsDouble()));
	}
	
	public Command cmdShoot() {
		final Command shootCmd = new SequentialCommandGroup(
			this.runOnce(() -> {
				//Restarts timer and sets the speed
			}),
			new FunctionalCommand(
				() -> {},
				() -> {
					double setpoint = SmartDashboard.getNumber(kMotorSpeedName, 0.0);
					SmartDashboard.putString(kStatusName, "Spinning up...");
					this.mTopPid.setReference(setpoint, ControlType.kVelocity, 0, this.mTopFeedf.calculate(setpoint), ArbFFUnits.kVoltage);
					SmartDashboard.putNumber(kVelocityName, this.mTopEncoder.getVelocity());
				},
				(Boolean inter) -> {},
				() -> false,
				this
			),
			this.deferredWaitCommand(() -> SmartDashboard.getNumber(kSpinupTimeName, 2.0)),
			//Sets speed of feed motor
			this.runOnce(() -> {
				this.mFeedMotor.set(SmartDashboard.getNumber(kFeedSpeedName, 0));
				SmartDashboard.putString(kStatusName, "Feeding");
			}),
			new RepeatCommand(new InstantCommand())
		);

		shootCmd.addRequirements(this);

		return shootCmd;
	}
}
