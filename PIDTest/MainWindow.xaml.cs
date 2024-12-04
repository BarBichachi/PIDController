using System;
using System.Diagnostics.CodeAnalysis;
using System.Threading;
using System.Timers;
using System.Windows;
using System.Windows.Controls;

using Timer = System.Timers.Timer;

namespace PIDTest
{
    public partial class MainWindow : Window
    {
        private PIDController m_PidController;
        private PIDController.PhysicalObject m_PhysicalObject;
        private readonly SynchronizationContext r_SynchronizationContext;
        private Timer m_Timer;
        private double m_Time;
        private double m_PreviousCommand;
        private double m_TargetVelocity;
        private double m_CurrentVelocity;
        private double m_TotalDegreesMoved;
        private bool m_IsSimulationRunning;

        public MainWindow()
        {
            InitializeComponent();
            InitializeObjectAndController();
            InitializeTimer();

            // Capture the SynchronizationContext
            r_SynchronizationContext = SynchronizationContext.Current;
        }

        private void InitializeTimer()
        {
            m_Timer = new Timer(1);
            m_Timer.Elapsed += TimerElapsed;
            m_Timer.AutoReset = true;
        }

        private void InitializeObjectAndController()
        {
            m_PhysicalObject = new PIDController.PhysicalObject
                                   {
                                       Mass = 10,
                                       Damping = 0.5,
                                       MaxForce = 5,
                                       MinForce = -5,
                                       TimeStep = 0.1,
                                       Acceleration = 0,
                                       Velocity = 0
                                   };

            m_PidController = new PIDController(1.5, 0.0001, 5, m_PhysicalObject);
        }

        private void StartSimulation(object sender, RoutedEventArgs e)
        {
            if (!m_IsSimulationRunning)
            {
                m_IsSimulationRunning = true;
                m_Time = 0;
                StatusLabel.Content = "Running..";
                m_Timer.Start();
            }
        }

        private void StopSimulation(object sender, RoutedEventArgs e)
        {
            if (m_IsSimulationRunning)
            {
                m_IsSimulationRunning = false;
                m_Timer.Stop();
                StatusLabel.Content = "Stopped";
            }
        }

        private void ResetSimulation(object sender, RoutedEventArgs e)
        {
            // Stop the simulation if running and reset all variables
            m_IsSimulationRunning = false;
            m_Time = 0;
            m_PreviousCommand = 0;
            m_CurrentVelocity = 0;
            m_TotalDegreesMoved = 0;

            InitializeTimer();
            m_Timer.Stop();
            InitializeObjectAndController();

            // Reset the UI elements
            VelocityTextBox.Text = "0.00";
            DegreesMovedTextBox.Text = "0.00";
            ElapsedTimeTextBox.Text = "0.00";
            StatusLabel.Content = "Reset";
        }

        private void SetTargetVelocity(object sender, RoutedEventArgs e)
        {
            // Set the target velocity based on the clicked button
            Button clickedButton = sender as Button;
            if (clickedButton != null)
            {
                if (double.TryParse(clickedButton.Content.ToString(), out double newTarget))
                {
                    m_TargetVelocity = newTarget;
                    StatusLabel.Content = $"Target Velocity: {m_TargetVelocity}°/s";
                }
            }
        }

        private void TimerElapsed(object sender, ElapsedEventArgs e)
        {
            // Compute and adjust control output from PID
            double targetCommand = m_PidController.ComputeControlSignal(m_CurrentVelocity, m_TargetVelocity);
            double command = m_PreviousCommand + Math.Sign(targetCommand - m_PreviousCommand) * Math.Min(Math.Abs(targetCommand - m_PreviousCommand), m_PhysicalObject.MaxForce);
            command = Math.Max(Math.Min(command, m_PhysicalObject.MaxForce), m_PhysicalObject.MinForce);

            // Update the object state and total degrees moved
            m_CurrentVelocity = PIDController.UpdateObjectState(m_PhysicalObject, command);
            m_TotalDegreesMoved += m_CurrentVelocity / 100;

            // Use SynchronizationContext to update UI
            r_SynchronizationContext.Post(_ =>
                {
                    VelocityTextBox.Text = $"{m_PhysicalObject.Velocity:F2}";
                    DegreesMovedTextBox.Text = $"{m_TotalDegreesMoved:F2}";
                    ElapsedTimeTextBox.Text = $"{m_Time:F2}";

                    // Check if velocity has stabilized
                    if (Math.Abs(m_CurrentVelocity - m_TargetVelocity) < 0.001)
                    {
                        StatusLabel.Content = "Stabilized";
                        //r_Timer.Stop();
                        //m_IsSimulationRunning = false;
                    }
                }, null);


            // Update elapsed time and store previous command
            m_Time += m_PhysicalObject.TimeStep / 10;
            m_PreviousCommand = command;
        }
    }
}