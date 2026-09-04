package frc.team449.subsystems.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import org.littletonrobotics.junction.Logger

open class DriveSubsystem(
    private val frontleftIO: ModuleIO,
    private val frontrightIO: ModuleIO,
    private val backleftIO: ModuleIO,
    private val backrightIO: ModuleIO
) : SubsystemBase() {

    private val frontleftinputs = ModuleIOInputsAutoLogged()
    private val frontrightinputs = ModuleIOInputsAutoLogged()
    private val backleftinputs = ModuleIOInputsAutoLogged()
    private val backrightinputs = ModuleIOInputsAutoLogged()

    private val flturnController = PIDController(0.5, 0.0, 0.05)
    private val fldriveController = PIDController(0.5, 0.0, 0.05)

    private val frturnController = PIDController(0.5, 0.0, 0.05)
    private val frdriveController = PIDController(0.5, 0.0, 0.05)

    private val blturnController = PIDController(0.5, 0.0, 0.05)
    private val bldriveController = PIDController(0.5, 0.0, 0.05)

    private val brturnController = PIDController(0.5, 0.0, 0.05)
    private val brdriveController = PIDController(0.5, 0.0, 0.05)

    private val frontleftlocation = Translation2d(Constants.ROBOT_WIDTH_INCHES/2, Constants.ROBOT_LENGTH_INCHES/2)
    private val frontrightlocation = Translation2d(-Constants.ROBOT_WIDTH_INCHES/2, Constants.ROBOT_LENGTH_INCHES/2)
    private val backleftlocation = Translation2d(Constants.ROBOT_WIDTH_INCHES/2, -Constants.ROBOT_LENGTH_INCHES/2)
    private val backrightlocation = Translation2d(-Constants.ROBOT_WIDTH_INCHES/2, -Constants.ROBOT_LENGTH_INCHES/2)

    private val kinematics =
        SwerveDriveKinematics(frontleftlocation, frontrightlocation, backleftlocation, backrightlocation)

    init {
        flturnController.enableContinuousInput(0.0, 2*Math.PI)
        frturnController.enableContinuousInput(0.0, 2*Math.PI)
        blturnController.enableContinuousInput(0.0, 2*Math.PI)
        brturnController.enableContinuousInput(0.0, 2*Math.PI)
    }

    override fun periodic() {
        frontleftIO.updateInputs(frontleftinputs)
        frontrightIO.updateInputs(frontrightinputs)
        backleftIO.updateInputs(backleftinputs)
        backrightIO.updateInputs(backrightinputs)

        frontleftIO.setVoltageTurn(flturnController.calculate(frontleftinputs.turningAngle))
        frontrightIO.setVoltageTurn(frturnController.calculate(frontrightinputs.turningAngle))
        backleftIO.setVoltageTurn(blturnController.calculate(backleftinputs.turningAngle))
        backrightIO.setVoltageTurn(brturnController.calculate(backrightinputs.turningAngle))

        frontleftIO.setVoltageDrive(fldriveController.calculate(frontleftinputs.drivingVelocity))
        frontrightIO.setVoltageDrive(frdriveController.calculate(frontrightinputs.drivingVelocity))
        backleftIO.setVoltageDrive(bldriveController.calculate(backleftinputs.drivingVelocity))
        backrightIO.setVoltageDrive(brdriveController.calculate(backrightinputs.drivingVelocity))

        Logger.processInputs("Front left", frontleftinputs)
        Logger.processInputs("Front left", frontrightinputs)
        Logger.processInputs("Front left", backleftinputs)
        Logger.processInputs("Front left", backrightinputs)
    }

    fun setSpeeds(forward: Double, sideways: Double, rotation: Double): Command =
        run {
            val speeds = ChassisSpeeds(forward, sideways, rotation)
            val moduleStates = kinematics.toSwerveModuleStates(speeds)

            val optimizedfrontleft = SwerveModuleState.optimize(moduleStates[0], Rotation2d(frontleftinputs.turningAngle))
            val frontleftradspersec = optimizedfrontleft.speedMetersPerSecond * 2*Math.PI / Constants.DriveConstants.WHEEL_DIAMETER_METERS
            // ^ convert to radians per second
            val frontleftangle = optimizedfrontleft.angle.radians

            val optimizedfrontright = SwerveModuleState.optimize(moduleStates[1], Rotation2d(frontrightinputs.turningAngle))
            val frontrightradspersec = optimizedfrontright.speedMetersPerSecond * 2*Math.PI / Constants.DriveConstants.WHEEL_DIAMETER_METERS
            // ^ convert to radians per second
            val frontrightangle = optimizedfrontright.angle.radians

            val optimizedbackleft = SwerveModuleState.optimize(moduleStates[2], Rotation2d(backleftinputs.turningAngle))
            val backleftradspersec = optimizedbackleft.speedMetersPerSecond * 2*Math.PI / Constants.DriveConstants.WHEEL_DIAMETER_METERS
            // ^ convert to radians per second
            val backleftangle = optimizedbackleft.angle.radians

            val optimizedbackright = SwerveModuleState.optimize(moduleStates[3], Rotation2d(backrightinputs.turningAngle))
            val backrightradspersec = optimizedbackright.speedMetersPerSecond * 2*Math.PI / Constants.DriveConstants.WHEEL_DIAMETER_METERS
            // ^ convert to radians per second
            val backrightangle = optimizedbackright.angle.radians

            flturnController.setpoint = frontleftangle
            frturnController.setpoint = frontrightangle
            blturnController.setpoint = backleftangle
            brturnController.setpoint = backrightangle

            fldriveController.setpoint = frontleftradspersec
            frdriveController.setpoint = frontrightradspersec
            bldriveController.setpoint = backleftradspersec
            brdriveController.setpoint = backrightradspersec
        }

}
