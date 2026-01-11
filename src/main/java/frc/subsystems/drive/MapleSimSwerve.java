package frc.robot.subsystems.drive;

public class MapleSimSwerve implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;

    public MapleSimSwerve() {
        System.out.println("THIS IS A SIMULATED SWERVE")
        // For your own code, please configure your drivetrain properly according to the documentation
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging
        field2d = new Field2d();
        SmartDashboard.putData("simulation field", field2d);

        @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                new Translation2d(),
                fieldRelative,
                true);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    @Override
    public Rotation2d getGyroYaw() {
        return simulatedDrive.getRawGyroAngle();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulatedDrive.periodic();

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
    }
    }
}