package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SDSConstants;

public class DrivetrainSubsystem extends SubsystemBase {
    //Base Swerve Requirements
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = SDSConstants.MAX_VELOCITY_METERS_PER_SECOND;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

    private final SwerveDriveOdometry odometry;

    public final Pigeon2 gyroscope = new Pigeon2(SDSConstants.DRIVETRAIN_PIGEON_ID);

    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    //Auto Rotate
    private final PIDController RotatePID;

    private double RotateSet;

    private boolean rotateLock = false;

    //Create Field for visualizing PathPlanner and Odometery
    private Field2d field = new Field2d();

    public DrivetrainSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        // Create the swerve modules
        frontLeftModule = new MkSwerveModuleBuilder()
            .withDriveMotor(MotorType.NEO, SDSConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, SDSConstants.FRONT_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(SDSConstants.FRONT_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(SDSConstants.FRONT_LEFT_MODULE_STEER_OFFSET)
            .withLayout(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0))
            .build();

        frontRightModule = new MkSwerveModuleBuilder()
            .withDriveMotor(MotorType.NEO, SDSConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, SDSConstants.FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(SDSConstants.FRONT_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(SDSConstants.FRONT_RIGHT_MODULE_STEER_OFFSET)
            .withLayout(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0))
            .build();

        backLeftModule = new MkSwerveModuleBuilder()
            .withDriveMotor(MotorType.NEO, SDSConstants.BACK_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, SDSConstants.BACK_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(SDSConstants.BACK_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(SDSConstants.BACK_LEFT_MODULE_STEER_OFFSET)
            .withLayout(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0))
            .build();

        backRightModule = new MkSwerveModuleBuilder()
            .withDriveMotor(MotorType.NEO, SDSConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, SDSConstants.BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(SDSConstants.BACK_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(SDSConstants.BACK_RIGHT_MODULE_STEER_OFFSET)
            .withLayout(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0))
            .build();


        // Zero the swerve Gyro
        gyroscope.setYaw(0.0);

        // Create the odometry
        odometry = new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(getGyroYaw()),
        new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          backLeftModule.getPosition(),
          backRightModule.getPosition()
        });

        //Report Odometry to Shuffleboard
        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());

        //Auto Rotate PID
        RotatePID = new PIDController(0.01, 0.00, 0.00);
        RotatePID.enableContinuousInput(-180.0f,  180.0f);
        RotatePID.setTolerance(2);

        RotateSet = 0;
        this.rotateLock = false;

        //PathPlanner
        // Configure AutoBuilder
        AutoBuilder.configureHolonomic(
        this::robotPose, 
        this::resetOdometry, 
        this::getSpeeds, 
        this::driveRobotRelative, 
        PathPlannerConstants.pathFollowerConfig,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
        );

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
    }

    public void zeroGyroscope() {
        odometry.resetPosition(
            Rotation2d.fromDegrees(getGyroYaw()), 
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }, 
            new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
        );
    }

    public double getRotationInDeg() {
        return getRotation().getDegrees();
    }

    public Pose2d robotPose() {
        return odometry.getPoseMeters();  //new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0));
    }

    public double getGyroYaw() {
        StatusSignal<Double> gYaw = gyroscope.getYaw();
        gYaw.refresh();
        return gYaw.getValue();
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public Pose2d getPos() {
        return odometry.getPoseMeters();
    }

    // Auto Drive
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        this.chassisSpeeds = targetSpeeds;
    }

    //Teleop Drive
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public ChassisSpeeds getSpeeds() {
        return this.chassisSpeeds;
    }

    public SwerveModuleState[] robotModuleStates(){
        return kinematics.toSwerveModuleStates(this.chassisSpeeds);
    }

    public void setPIDRotateValue(double RotateSet) {
        this.RotateSet = RotateSet;
    }

    public double getPIDRotateValue() {
        return this.RotateSet;
    }

    public double rotatePIDCalculation() {
        double futureRotatePID = MathUtil.clamp(RotatePID.calculate(getRotation().getDegrees(), RotateSet), -1, 1);

        return futureRotatePID;
    }

    public void setRotateLock(boolean rotateLock) {
        this.rotateLock = rotateLock;
    }

    public boolean getRotateLock() {
        return this.rotateLock;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(getGyroYaw()), 
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }, 
            pose
        );
    }

    @Override
    public void periodic() {

        rotatePIDCalculation();

        dashboard();

        // Update the odometry, Important otherwise robot will be "lost"
        odometry.update(
        Rotation2d.fromDegrees(getGyroYaw()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });

        field.setRobotPose(robotPose());

        setModuleStates(robotModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] states) {
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    public void stopModules() {
        //Stop Modules
        frontLeftModule.set(0, 0);
        frontRightModule.set(0, 0);
        backLeftModule.set(0, 0);
        backRightModule.set(0, 0);
    }

    private void dashboard() {
        SmartDashboard.putNumber("Rotate Set Point", getPIDRotateValue());
        SmartDashboard.putNumber("Rotate PID Calc", rotatePIDCalculation());
    }
}