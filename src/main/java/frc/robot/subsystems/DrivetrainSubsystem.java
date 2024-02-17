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
    //private Field2d field2 = new Field2d();

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

        //Cameras
        /* 
         Setting Up LimeLight IP Address
         https://docs.limelightvision.io/docs/docs-limelight/getting-started/networking
         Use http://limelight.local:5801/ to setup a static IP address
        */
        /* ShuffleboardTab visionShuffleboardTab = Shuffleboard.getTab("Vision");
        visionShuffleboardTab.addCamera("Front Camera", "Front LimeLight", "10.8.58.11");
        visionShuffleboardTab.addCamera("Rear Camera", "Rear LimeLight", "10.8.58.12"); */

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

        /* //Get Latest Results from Limelight
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");

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

        dashboard();
    }

    /**
     * All Smartdashboard Data Here, This gets called once upon setup
    */
    private void dashboard() {

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);

        //SmartDashboard.putData("Field2", field2);

        SmartDashboard.putNumber("Rotate Set Point", getPIDRotateValue());
        SmartDashboard.putNumber("Rotate PID Calc", rotatePIDCalculation());
    }

    public void beforeauto() {
        setRotateLock(false);
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

    /**
     * Drives Field Relative Swerve
     * @param fieldRelativeSpeeds
     */
    private void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotPose().getRotation()));
    }

    /**
     * Drives Robot Relative Swerve
     * @param fieldRelativeSpeeds
     */
    private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        this.chassisSpeeds = targetSpeeds;
    }

    /**
     * Allows control of swerve drive system, this is defaults to Field Relitive Drive
     * @param chassisSpeeds used with new ChassisSpeeds(), this is for all the joystick data
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        driveFieldRelative(chassisSpeeds);
    }

    /**
     * Allows control of swerve drive system
     * @param chassisSpeeds used with new ChassisSpeeds(), this is for all the joystick data
     * @param robotRelative should the drive be robot relitive or field relitive?
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean robotRelative) {
        if(robotRelative) {driveRobotRelative(chassisSpeeds);} else {driveFieldRelative(chassisSpeeds);}
    }

    /**
     * Stops Swerve Drive
     */
    public void stop() {
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    //Auto Rotate
    public double autoRotate(double rotation, int POV) {
        return autoRotate(rotation, POV, false);
    }

    public double autoRotate(double rotation, int POV, boolean invertGyro) {

        //double rotationPercent = rotation;

        //Allows the Robot to angle its self if the POV is pressed and with in the deadband of the axis
        if(rotation < 0.05 && rotation > -0.05) {
            if(POV != -1) {

                //Allow to invert if gyro is upside down
                if(invertGyro) {
                    //-360 flips the axis
                    POV = 360 - POV;
                }

                setPIDRotateValue(POV);
                setRotateLock(true);
            }
        } else {
            setRotateLock(false);
        }

        if(getRotateLock()) {
            return rotatePIDCalculation();
        }

        return rotation;
    }

    private void setPIDRotateValue(double RotateSet) {
        this.RotateSet = RotateSet;
    }

    private double getPIDRotateValue() {
        return this.RotateSet;
    }

    private double rotatePIDCalculation() {
        double futureRotatePID = MathUtil.clamp(RotatePID.calculate(getRotation().getDegrees(), getPIDRotateValue()), -1, 1);

        //Prevent Over Speed
        futureRotatePID = MathUtil.clamp(futureRotatePID, -0.5, 0.5);

        return -futureRotatePID;
    }

    private void setRotateLock(boolean rotateLock) {
        this.rotateLock = rotateLock;
    }

    private boolean getRotateLock() {
        return this.rotateLock;
    }
    //Auto Rotate

    public ChassisSpeeds getSpeeds() {
        return this.chassisSpeeds;
    }

    public SwerveModuleState[] robotModuleStates(){
        return kinematics.toSwerveModuleStates(this.chassisSpeeds);
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

    public void zeroPose() {
        odometry.resetPosition(
            Rotation2d.fromDegrees(getGyroYaw()), 
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }, 
            new Pose2d()
        );
    }

    @Override
    public void periodic() {

        rotatePIDCalculation();

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
        Rotation2d.fromDegrees(getGyroYaw()), 
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
        }

        field.setRobotPose(robotPose()); */

        //field2.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());

        setModuleStates(robotModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] states) {
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}