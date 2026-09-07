package frc.team449.subsystems.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import kotlin.math.pow
import kotlin.math.sign

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

    private val flturnController = PIDController(0.5, 0.0, 0.06)
    private val fldriveController = PIDController(0.025, 0.0, 0.0006)

    private val frturnController = PIDController(0.5, 0.0, 0.06)
    private val frdriveController = PIDController(0.025, 0.0, 0.0006)

    private val blturnController = PIDController(0.5, 0.0, 0.06)
    private val bldriveController = PIDController(0.025, 0.0, 0.0006)

    private val brturnController = PIDController(0.5, 0.0, 0.06)
    private val brdriveController = PIDController(0.025, 0.0, 0.0006)

    private val frontleftlocation = Translation2d(Constants.ROBOT_WIDTH_INCHES / 2, Constants.ROBOT_LENGTH_INCHES / 2)
    private val frontrightlocation = Translation2d(-Constants.ROBOT_WIDTH_INCHES / 2, Constants.ROBOT_LENGTH_INCHES / 2)
    private val backleftlocation = Translation2d(Constants.ROBOT_WIDTH_INCHES / 2, -Constants.ROBOT_LENGTH_INCHES / 2)
    private val backrightlocation = Translation2d(-Constants.ROBOT_WIDTH_INCHES / 2, -Constants.ROBOT_LENGTH_INCHES / 2)

    private val kinematics =
        SwerveDriveKinematics(frontleftlocation, frontrightlocation, backleftlocation, backrightlocation)

    init {
        flturnController.enableContinuousInput(-Math.PI, Math.PI)
        frturnController.enableContinuousInput(-Math.PI, Math.PI)
        blturnController.enableContinuousInput(-Math.PI, Math.PI)
        brturnController.enableContinuousInput(-Math.PI, Math.PI)

        flturnController.setpoint = frontleftinputs.turningAngle.radians
        frturnController.setpoint = frontrightinputs.turningAngle.radians
        blturnController.setpoint = backleftinputs.turningAngle.radians
        brturnController.setpoint = backrightinputs.turningAngle.radians

        fldriveController.setpoint = 0.0
        frdriveController.setpoint = 0.0
        bldriveController.setpoint = 0.0
        brdriveController.setpoint = 0.0
    }

    override fun periodic() {
        frontleftIO.updateInputs(frontleftinputs)
        frontrightIO.updateInputs(frontrightinputs)
        backleftIO.updateInputs(backleftinputs)
        backrightIO.updateInputs(backrightinputs)

        val fldrivevoltage = fldriveController.calculate(frontleftinputs.drivingVelocity)
        val frdrivevoltage = frdriveController.calculate(frontrightinputs.drivingVelocity)
        val bldrivevoltage = bldriveController.calculate(backleftinputs.drivingVelocity)
        val brdrivevoltage = brdriveController.calculate(backrightinputs.drivingVelocity)

        frontleftIO.setVoltageDrive(fldrivevoltage)
        frontrightIO.setVoltageDrive(frdrivevoltage)
        backleftIO.setVoltageDrive(bldrivevoltage)
        backrightIO.setVoltageDrive(brdrivevoltage)

        frontleftIO.setVoltageTurn(flturnController.calculate(frontleftinputs.turningAngle.radians))
        frontrightIO.setVoltageTurn(frturnController.calculate(frontrightinputs.turningAngle.radians))
        backleftIO.setVoltageTurn(blturnController.calculate(backleftinputs.turningAngle.radians))
        backrightIO.setVoltageTurn(brturnController.calculate(backrightinputs.turningAngle.radians))

        Logger.processInputs("Front left", frontleftinputs)
        Logger.processInputs("Front right", frontrightinputs)
        Logger.processInputs("Back left", backleftinputs)
        Logger.processInputs("Back right", backrightinputs)
    }

    fun setSpeeds(forward: DoubleSupplier, sideways: DoubleSupplier, rotation: DoubleSupplier): Command =
        run {
            println()
            val speeds = ChassisSpeeds(
                forward.asDouble.pow(2.0) * sign(forward.asDouble) * Constants.DriveConstants.MAX_LINEAR_SPEED,
                sideways.asDouble.pow(2.0) * sign(sideways.asDouble) * Constants.DriveConstants.MAX_LINEAR_SPEED,
                rotation.asDouble.pow(2.0) * sign(rotation.asDouble) * Constants.DriveConstants.MAX_ROT_SPEED
            )
            val moduleStates = kinematics.toSwerveModuleStates(speeds)

            val optimizedfrontleft = SwerveModuleState.optimize(moduleStates[0], (frontleftinputs.turningAngle))
            val frontleftradspersec = optimizedfrontleft.speedMetersPerSecond * 2 * Math.PI / Constants.DriveConstants.WHEEL_DIAMETER_METERS
            // ^ convert to radians per second
            val frontleftangle = optimizedfrontleft.angle.radians

            val optimizedfrontright = SwerveModuleState.optimize(moduleStates[1], (frontrightinputs.turningAngle))
            val frontrightradspersec = optimizedfrontright.speedMetersPerSecond * 2 * Math.PI / Constants.DriveConstants.WHEEL_DIAMETER_METERS
            // ^ convert to radians per second
            val frontrightangle = optimizedfrontright.angle.radians

            val optimizedbackleft = SwerveModuleState.optimize(moduleStates[2], (backleftinputs.turningAngle))
            val backleftradspersec = optimizedbackleft.speedMetersPerSecond * 2 * Math.PI / Constants.DriveConstants.WHEEL_DIAMETER_METERS
            // ^ convert to radians per second
            val backleftangle = optimizedbackleft.angle.radians

            val optimizedbackright = SwerveModuleState.optimize(moduleStates[3], (backrightinputs.turningAngle))
            val backrightradspersec = optimizedbackright.speedMetersPerSecond * 2 * Math.PI / Constants.DriveConstants.WHEEL_DIAMETER_METERS
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
