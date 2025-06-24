// System
using System;
using System.Threading;
using System.Collections;
using System.Diagnostics;
using System.Linq;
// Unity 
using UnityEngine;
using Debug = UnityEngine.Debug;
// OPC UA
using Opc.Ua;
using Opc.Ua.Client;
using Opc.Ua.Configuration;


public class OPC_UA_Client : MonoBehaviour
{
    public enum OPC_UA_Client_STATE_Enum
    {
        DISCONNECTED = 0,
        CONNECTED = 10
    }

    public static class G_OPC_UA_Client_Str
    {
        public static bool Is_Connected;

        /*
        Description:
            Client settings.
        */
        public static string Ip_Address;
        // OPC UA port number for communication.
        public const ushort port_number = 4840;
        // Communication speed in milliseconds.
        public static int time_step;
    }

    public static class G_OPC_UA_Client_Ctrl_UR_Str
    {
        // Variables to control Unity3D simulation from Siemens PLC.
        //  Note: PLC -> Unity3D Sim.
        public static NodeId Start_NodeId = "";
        public static bool Start;
        public static NodeId Stop_NodeId = "";
        public static bool Stop;
        public static NodeId Home_NodeId = "";
        public static bool Home;
        public static NodeId Speed_Percentage_NodeId = "";
        public static float Speed_Percentage
        {
            get { return Speed_Percentage; }
            set
            {
                if (value <= 0)
                    throw new ArgumentException("Value must be greater than 0.");
                Speed_Percentage = value;
            }
        }
        // Variables to provide feedback to the Siemens PLC from the Unity3D simulation.
        //  Note: Unity3D Sim. -> PLC
        public static string Is_Started_NodeId = "";
        public static bool Is_Started;
        public static string Is_Stopped_NodeId = "";
        public static bool Is_Stopped;
        public static string Is_Homed_NodeId = "";
        public static bool Is_Homed;
    }

    /*
    Description:
        Public variables.
    */
    public bool Connect;
    public bool Disconnect;

    /*
    Description:
        Global variables.
    */
    private OPC_UA_Client_STATE_Enum state_id;

    // Initialization of classes that will read and write data via OPC UA communication.
    private OPC_UA_Client_Read_Data_Cls OPC_UA_Client_R_Cls = new OPC_UA_Client_Read_Data_Cls();
    private OPC_UA_Client_Write_Data_Cls OPC_UA_Client_W_Cls = new OPC_UA_Client_Write_Data_Cls();

    // Information about whether the thread is alive or not.
    public static bool opc_ua_client_r_is_alive = false;
    public static bool opc_ua_client_w_is_alive = false;

    void Start()
    {
        // Set the initial parameters of the OPC UA client.
        G_OPC_UA_Client_Str.Ip_Address = "192.168.0.1";
        G_OPC_UA_Client_Str.time_step = 10;
    }

    void Update()
    {
        switch (state_id)
        {
            case OPC_UA_Client_STATE_Enum.DISCONNECTED:
                {
                    G_OPC_UA_Client_Str.Is_Connected = false;
                    if (Connect == true)
                    {
                        Disconnect = false;
                        OPC_UA_Client_R_Cls.Start(); OPC_UA_Client_W_Cls.Start();
                        state_id = OPC_UA_Client_STATE_Enum.CONNECTED;
                    }
                }
                break;

            case OPC_UA_Client_STATE_Enum.CONNECTED:
                {
                    G_OPC_UA_Client_Str.Is_Connected = true;
                    if (Disconnect == true)
                    {
                        Connect = false;
                        if (opc_ua_client_r_is_alive == true)
                        {
                            OPC_UA_Client_R_Cls.Stop();
                        }
                        if (opc_ua_client_w_is_alive == true)
                        {
                            OPC_UA_Client_W_Cls.Stop();
                        }

                        if (opc_ua_client_r_is_alive == false && opc_ua_client_w_is_alive == false)
                        {
                            state_id = OPC_UA_Client_STATE_Enum.DISCONNECTED;
                        }
                    }
                }
                break;
        }
    }

    /*
    Description:
        Help functions for the control.
    */
    void OnApplicationQuit()
    {
        try
        {
            // Destroy the threads used to read and write data via OPC UA communication.
            OPC_UA_Client_R_Cls.Destroy(); OPC_UA_Client_W_Cls.Destroy();

            Destroy(this);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
    }

    public static Session OPC_UA_Client_Create_Session(ApplicationConfiguration client_configuration, EndpointDescription client_end_point)
    {
        return Session.Create(client_configuration, new ConfiguredEndpoint(null, client_end_point, EndpointConfiguration.Create(client_configuration)), false, "", 10000, null, null).GetAwaiter().GetResult();
    }

    public static ApplicationConfiguration OPC_UA_Client_Configuration()
    {
        // Configuration OPCUA Client {W/R -> Data}
        var config = new ApplicationConfiguration()
        {
            // Initialization (Name, Uri, etc.)
            ApplicationName = "OPCUA_AS", // OPCUA AS (Automation Studio B&R)
            ApplicationUri = Utils.Format(@"urn:{0}:OPCUA_AS", System.Net.Dns.GetHostName()),
            // Type -> Client
            ApplicationType = ApplicationType.Client,
            SecurityConfiguration = new SecurityConfiguration
            {
                // Security Configuration - Certificate
                ApplicationCertificate = new CertificateIdentifier { StoreType = @"Directory", StorePath = @"%CommonApplicationData%\OPC Foundation\CertificateStores\MachineDefault", SubjectName = Utils.Format(@"CN={0}, DC={1}", "OPCUA_AS", System.Net.Dns.GetHostName()) },
                TrustedIssuerCertificates = new CertificateTrustList { StoreType = @"Directory", StorePath = @"%CommonApplicationData%\OPC Foundation\CertificateStores\UA Certificate Authorities" },
                TrustedPeerCertificates = new CertificateTrustList { StoreType = @"Directory", StorePath = @"%CommonApplicationData%\OPC Foundation\CertificateStores\UA Applications" },
                RejectedCertificateStore = new CertificateTrustList { StoreType = @"Directory", StorePath = @"%CommonApplicationData%\OPC Foundation\CertificateStores\RejectedCertificates" },
                AutoAcceptUntrustedCertificates = true,
                AddAppCertToTrustedStore = true
            },
            TransportConfigurations = new TransportConfigurationCollection(),
            TransportQuotas = new TransportQuotas { OperationTimeout = 10000 },
            ClientConfiguration = new ClientConfiguration { DefaultSessionTimeout = 50000 },
            TraceConfiguration = new TraceConfiguration()
        };
        config.Validate(ApplicationType.Client).GetAwaiter().GetResult();
        if (config.SecurityConfiguration.AutoAcceptUntrustedCertificates)
        {
            config.CertificateValidator.CertificateValidation += (s, e) => { e.Accept = (e.Error.StatusCode == StatusCodes.BadCertificateUntrusted); };
        }

        return config;
    }

    class OPC_UA_Client_Read_Data_Cls
    {
        // Initialization of the class variables.
        private Thread ctrl_thread = null;
        private bool exit_thread = false;
        // OPC UA client app. configuration class.
        ApplicationConfiguration app_configuration = new ApplicationConfiguration();

        public void Cls_Core_Thread()
        {
            try
            {
                // OPC UA Client configuration.
                app_configuration = OPC_UA_Client_Configuration();
                // Establishing communication.
                EndpointDescription end_point = CoreClientUtils.SelectEndpoint("opc.tcp://" + G_OPC_UA_Client_Str.Ip_Address + ":" + G_OPC_UA_Client_Str.port_number, useSecurity: false);
                // Create session.
                Session client_session = OPC_UA_Client_Create_Session(app_configuration, end_point);

                // Initialization timer.
                var t = new Stopwatch();

                while (exit_thread == false)
                {
                    // t_{0}: Timer start.
                    t.Start();

                    /*
                    Description:
                        Block used to read data from the server via the OPC UA communication.
                    */
                    G_OPC_UA_Client_Ctrl_UR_Str.Start = bool.Parse(client_session.ReadValue(G_OPC_UA_Client_Ctrl_UR_Str.Start_NodeId).ToString());
                    G_OPC_UA_Client_Ctrl_UR_Str.Stop = bool.Parse(client_session.ReadValue(G_OPC_UA_Client_Ctrl_UR_Str.Start_NodeId).ToString());
                    G_OPC_UA_Client_Ctrl_UR_Str.Home = bool.Parse(client_session.ReadValue(G_OPC_UA_Client_Ctrl_UR_Str.Start_NodeId).ToString());
                    G_OPC_UA_Client_Ctrl_UR_Str.Speed_Percentage = float.Parse(client_session.ReadValue(G_OPC_UA_Client_Ctrl_UR_Str.Speed_Percentage_NodeId).ToString());

                    // t_{1}: Timer stop.
                    t.Stop();

                    // Recalculate the time: t = t_{1} - t_{0} -> Elapsed Time in milliseconds.
                    if (t.ElapsedMilliseconds < G_OPC_UA_Client_Str.time_step)
                    {
                        Thread.Sleep(G_OPC_UA_Client_Str.time_step - (int)t.ElapsedMilliseconds);
                    }

                    // Reset (Restart) timer.
                    t.Restart();
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("Communication Problem: {0}", e);
            }
        }

        public void Start()
        {
            exit_thread = false;
            // Start a thread to read data from the OPC UA server.
            ctrl_thread = new Thread(new ThreadStart(Cls_Core_Thread));
            ctrl_thread.IsBackground = true;
            ctrl_thread.Start();

            // The thread is active.
            opc_ua_client_r_is_alive = true;
        }

        public void Stop()
        {
            exit_thread = true;
            ctrl_thread.Abort();
            // Stop a thread.
            Thread.Sleep(100);

            // The thread is inactive.
            opc_ua_client_r_is_alive = false;
        }

        public void Destroy()
        {
            // Stop a thread (OPC UA communication).
            Stop();
            Thread.Sleep(100);
        }
    }

    class OPC_UA_Client_Write_Data_Cls
    {
        // Initialization of the class variables.
        private Thread ctrl_thread = null;
        private bool exit_thread = false;
        // OPC UA client app. configuration class.
        ApplicationConfiguration app_configuration = new ApplicationConfiguration();

        public void Cls_Core_Thread()
        {
            try
            {
                // OPC UA Client configuration.
                app_configuration = OPC_UA_Client_Configuration();
                // Establishing communication
                EndpointDescription end_point = CoreClientUtils.SelectEndpoint("opc.tcp://" + G_OPC_UA_Client_Str.Ip_Address + ":" + G_OPC_UA_Client_Str.port_number, useSecurity: false);
                // Create session.
                Session client_session = OPC_UA_Client_Create_Session(app_configuration, end_point);

                // Initialization timer
                var t = new Stopwatch();

                while (exit_thread == false)
                {
                    // t_{0}: Timer start.
                    t.Start();

                    /*
                    Description:
                        Block used to write data to the server via the OPC UA communication.
                    */
                    OPC_UA_Client_Write_Value(client_session, G_OPC_UA_Client_Ctrl_UR_Str.Is_Started_NodeId,
                                              G_OPC_UA_Client_Ctrl_UR_Str.Is_Started.ToString());
                    OPC_UA_Client_Write_Value(client_session, G_OPC_UA_Client_Ctrl_UR_Str.Is_Stopped_NodeId,
                                              G_OPC_UA_Client_Ctrl_UR_Str.Is_Stopped.ToString());
                    OPC_UA_Client_Write_Value(client_session, G_OPC_UA_Client_Ctrl_UR_Str.Is_Homed_NodeId,
                                              G_OPC_UA_Client_Ctrl_UR_Str.Is_Homed.ToString());

                    // t_{1}: Timer stop.
                    t.Stop();

                    // Recalculate the time: t = t_{1} - t_{0} -> Elapsed Time in milliseconds
                    if (t.ElapsedMilliseconds < G_OPC_UA_Client_Str.time_step)
                    {
                        Thread.Sleep(G_OPC_UA_Client_Str.time_step - (int)t.ElapsedMilliseconds);
                    }

                    // Reset (Restart) timer.
                    t.Restart();
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("Communication Problem: {0}", e);
            }
        }

        bool OPC_UA_Client_Write_Value(Session client_session, string node_id, string value_write)
        {
            NodeId init_node = NodeId.Parse(node_id);
            try
            {
                Node node = client_session.NodeCache.Find(init_node) as Node;
                DataValue init_data_value = client_session.ReadValue(node.NodeId);

                // Preparation of the data to be written.
                WriteValue value = new WriteValue()
                {
                    NodeId = init_node,
                    AttributeId = Attributes.Value,
                    Value = new DataValue(new Variant(Convert.ChangeType(value_write, init_data_value.Value.GetType()))),
                };

                WriteValueCollection init_write = new WriteValueCollection();
                init_write.Add(value);

                StatusCodeCollection results = null;
                DiagnosticInfoCollection diagnostic_info = null;

                // Write data.
                client_session.Write(null, init_write, out results, out diagnostic_info);

                // Check the status of the results.
                return (results[0] == StatusCodes.Good) ? true : false;

            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);

                return false;
            }
        }

        public void Start()
        {
            exit_thread = false;
            // Start a thread to write data to the OPC UA server.
            ctrl_thread = new Thread(new ThreadStart(Cls_Core_Thread));
            ctrl_thread.IsBackground = true;
            ctrl_thread.Start();

            // The thread is active.
            opc_ua_client_w_is_alive = true;
        }

        public void Stop()
        {
            exit_thread = true;
            ctrl_thread.Abort();
            // Stop a thread.
            Thread.Sleep(100);

            // The thread is inactive.
            opc_ua_client_w_is_alive = false;
        }

        public void Destroy()
        {
            // Stop a thread (OPC UA communication).
            Stop();
            Thread.Sleep(100);
        }
    }
}