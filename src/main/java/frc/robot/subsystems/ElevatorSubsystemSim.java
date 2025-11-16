package frc.robot.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Simulation logic for the ElevatorSubsystem. */
public class ElevatorSubsystemSim {
  // Simulation PID constants - tuned for smooth motion
  private static final double SIM_KP = 50.0;
  private static final double SIM_KD = 5.0;
  private static final double MOTOR_ROTATIONS_PER_METER = 19.68;

  private final ElevatorSim m_elevatorSim;
  private final TalonFXSimState m_motorOneSimState;
  private final TalonFXSimState m_motorTwoSimState;
  private final StructPublisher<Pose3d> m_posePublisher;

  private double simTargetHeightMeters = 0.0;
  private double simPreviousError = 0.0;

  /**
   * Constructor for the elevator simulation.
   *
   * @param elevatorSim The WPILib elevator simulation model
   * @param motorOneSimState Sim state for motor 1
   * @param motorTwoSimState Sim state for motor 2
   * @param posePublisher Publisher for visualization pose
   */
  public ElevatorSubsystemSim(
      ElevatorSim elevatorSim,
      TalonFXSimState motorOneSimState,
      TalonFXSimState motorTwoSimState,
      StructPublisher<Pose3d> posePublisher) {
    this.m_elevatorSim = elevatorSim;
    this.m_motorOneSimState = motorOneSimState;
    this.m_motorTwoSimState = motorTwoSimState;
    this.m_posePublisher = posePublisher;
  }

  /**
   * Set the target position for the simulated elevator.
   *
   * @param targetPositionRotations Target position in motor rotations
   */
  public void setTargetPosition(double targetPositionRotations) {
    simTargetHeightMeters = targetPositionRotations / MOTOR_ROTATIONS_PER_METER;
  }

  /**
   * Update the simulation. Should be called periodically.
   *
   * @param currentPositionRotations Current position reading from motor (for visualization)
   */
  public void updateSimulation(double currentPositionRotations) {
    // Calculate PD control to smoothly move to target position
    double currentHeightMeters = m_elevatorSim.getPositionMeters();
    double error = simTargetHeightMeters - currentHeightMeters;
    double errorRate = (error - simPreviousError) / 0.02; // derivative

    // PD control output
    double pidOutput = (SIM_KP * error) + (SIM_KD * errorRate);
    simPreviousError = error;

    // Apply voltage to simulation
    m_elevatorSim.setInputVoltage(pidOutput);
    m_elevatorSim.update(0.02); // 20ms periodic cycle

    // Get simulated position and convert to motor rotations
    double simHeightMeters = m_elevatorSim.getPositionMeters();
    double simPositionRotations = simHeightMeters * MOTOR_ROTATIONS_PER_METER;

    // Update motor sim states with realistic position
    m_motorOneSimState.setRawRotorPosition(simPositionRotations);
    m_motorTwoSimState.setRawRotorPosition(simPositionRotations);

    // Update visualization
    updateVisualization(currentPositionRotations);
  }

  /**
   * Update the 3D pose visualization for the elevator.
   *
   * @param currentPositionRotations Current position in motor rotations
   */
  private void updateVisualization(double currentPositionRotations) {
    double bottomZ = 0.2;
    double topZ = 1.55;
    double minPos = 0.0;
    double maxPos = 37.5;
    double targetZ =
        (bottomZ + ((currentPositionRotations - minPos) / (maxPos - minPos)) * (topZ - bottomZ));

    m_posePublisher.set(new Pose3d(0.2, 0.0, targetZ, new Rotation3d(0.0, 0.0, -135)));
  }
}
