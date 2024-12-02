using System;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using PIDControllerLib;

namespace PIDDemo
{
    public partial class MainWindow : Window
    {
        private PIDController m_PidController;
        private StringBuilder m_OutputLog;
        private CancellationTokenSource m_CancellationTokenSource;
        private bool m_IsRunning;

        public MainWindow()
        {
            InitializeComponent();
            InitializePIDSimulation();
        }

        private void InitializePIDSimulation()
        {
            m_PidController = new PIDController(1.0, 0.01, 0.1)
            {
                Setpoint = 50, // Target position
                DeltaTime = 0.1, // Simulation time step
                Tolerance = 0.0005, // Stopping precision
                MaxVelocity = 30.0, // Maximum velocity
                MaxAcceleration = 10.0
            };

            m_PidController.OnTargetReached += (sender, e) =>
            {
                AddLog($"Target reached at {m_PidController.Setpoint:F2} degrees!");
                m_IsRunning = false;
            };

            m_OutputLog = new StringBuilder();
            m_CancellationTokenSource = new CancellationTokenSource();

            // Start the simulation
            m_IsRunning = true;
            StartSimulation(m_CancellationTokenSource.Token);
        }

        private async void StartSimulation(CancellationToken i_CancellationToken)
        {
            await Task.Run(() =>
            {
                try
                {
                    for (int step = 0; step < 300; step++)
                    {
                        // Update PID controller state
                        m_PidController.Update();

                        if (i_CancellationToken.IsCancellationRequested || !m_IsRunning)
                        {
                            break;
                        }

                        // Log the simulation state
                        string logEntry = $"Step {step}: Position = {m_PidController.CurrentPosition:F5}°, Velocity = {m_PidController.Velocity:F5}°/s";
                        AddLog(logEntry);

                        // Simulate real-time step
                        Thread.Sleep((int)(m_PidController.DeltaTime * 100));
                    }
                }
                catch (Exception ex)
                {
                    AddLog($"Error: {ex.Message}");
                }
            });
        }


        private void AddLog(string log)
        {
            // Update log in UI thread
            Dispatcher.Invoke(() =>
            {
                m_OutputLog.AppendLine(log);
                OutputTextBox.Text = m_OutputLog.ToString();
                OutputTextBox.ScrollToEnd();
            });
        }

        protected override void OnClosed(EventArgs e)
        {
            m_CancellationTokenSource.Cancel();
            base.OnClosed(e);
        }
    }
}
