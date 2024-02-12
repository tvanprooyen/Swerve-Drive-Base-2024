package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.limelight.LimelightHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

    public final Pigeon2 gyroscope;

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

    // Odometery Vision 
    //private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public DrivetrainSubsystem() {

        //Give devices time to start up (CANCoders, Pigeon, ect.)
        Timer.delay(0.5);

        gyroscope = new Pigeon2(SDSConstants.DRIVETRAIN_PIGEON_ID);

        //Remove All Other StatusSignals, Only Using YAW
        gyroscope.optimizeBusUtilization();

        //Set at Default Frequency, This needs to happen in order for the getYaw to be used
        gyroscope.getYaw().setUpdateFrequency(100);

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        // Create the swerve modules
        frontLeftModule = new MkSwerveModuleBuilder()
        .withDriveMotor(MotorType.NEO, SDSConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, SDSConstants.FRONT_LEFT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(SDSConstants.FRONT_LEFT_MODULE_STEER_ENCODER)
        .withSteerOffset(SDSConstants.FRONT_LEFT_MODULE_STEER_OFFSET)
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withLayout(
            shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0)
        ).build();

        frontRightModule = new MkSwerveModuleBuilder()
        .withDriveMotor(MotorType.NEO, SDSConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, SDSConstants.FRONT_RIGHT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(SDSConstants.FRONT_RIGHT_MODULE_STEER_ENCODER)
        .withSteerOffset(SDSConstants.FRONT_RIGHT_MODULE_STEER_OFFSET)
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withLayout(
            shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0)
        ).build();

        backLeftModule = new MkSwerveModuleBuilder()
        .withDriveMotor(MotorType.NEO, SDSConstants.BACK_LEFT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, SDSConstants.BACK_LEFT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(SDSConstants.BACK_LEFT_MODULE_STEER_ENCODER)
        .withSteerOffset(SDSConstants.BACK_LEFT_MODULE_STEER_OFFSET)
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withLayout(
            shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0)
        ).build();
        

        backRightModule = new MkSwerveModuleBuilder()
        .withDriveMotor(MotorType.NEO, SDSConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, SDSConstants.BACK_RIGHT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(SDSConstants.BACK_RIGHT_MODULE_STEER_ENCODER)
        .withSteerOffset(SDSConstants.BACK_RIGHT_MODULE_STEER_OFFSET)
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withLayout(
            shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0)
        ).build();

        // Zero the swerve Gyro
        gyroscope.setYaw(0.0);

        //Get Latest Results from Limelight
        /* LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");

        Pose2d initialPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

        //Test if Limelight has a target
        if(LimelightHelpers.getTV("")) {
            //update the pose estimator with the vision measurement
            initialPose = llresults.targetingResults.getBotPose2d();
        }

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            Rotation2d.fromDegrees(getGyroYaw(true)), 
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }, 
            initialPose
        ); */

            

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
        RotatePID = new PIDController(0.006, 0.00, 0.0001);
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
        return gyroscope.getYaw().getValue();
    }

    public double getGyroYaw(boolean invert) {
        double yawV = gyroscope.getYaw().getValue();

        if(invert) {
            yawV = 360 - yawV;
        }

        return yawV;
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

    //TODO Double Check if causing issue with pose estimator
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

        /* swerveDrivePoseEstimator.update(
        Rotation2d.fromDegrees(getGyroYaw(true)), 
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });

        //Get Latest Results from Limelight
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");

        //Test if Limelight has a target
        if(LimelightHelpers.getTV("")) {
            //update the pose estimator with the vision measurement
            swerveDrivePoseEstimator.addVisionMeasurement(llresults.targetingResults.getBotPose2d(), llresults.targetingResults.timestamp_RIOFPGA_capture);
        } */

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