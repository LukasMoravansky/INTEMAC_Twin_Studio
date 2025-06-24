// System
using System;
using System.Diagnostics;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;
// Unity 
using UnityEngine;
using Debug = UnityEngine.Debug;

public class UR_Client_Interface : MonoBehaviour
{

    public enum UR_Client_STATE_Enum
    {
        DISCONNECTED = 0,
        CONNECTED = 10
    }

    public static class G_UR_Client_Str
    {
        public static bool Is_Connected;

        /*
        Description:
            Client settings.
        */
        // IP Port Number and IP Address
        public static string ip_address;
        //  Real-time (Read Only)
        public const ushort port_number = 30013;
        // Comunication Speed (ms)
        public static int time_step;
        // Joint Space:
        //  Orientation {J1 .. J6} (rad)
        public static double[] J_Orientation = new double[6];
        // Class thread information (is alive or not)
        public static bool is_alive = false;
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
    private UR_Client_STATE_Enum state_id;

    // Initialization of classes that will read data via TCP/IP communication.
    private UR_Client_Read_Data_Cls UR_Client_R_Cls = new UR_Client_Read_Data_Cls();

    // Information about whether the thread is alive or not.
    public static bool ur_client_r_is_alive = false;

    // Start is called before the first frame update
    void Start()
    {
        // Set the initial parameters of the TCP/IP client.
        G_UR_Client_Str.ip_address = "127.0.0.1";
        //  Communication speed: CB-Series 125 Hz (8 ms), E-Series 500 Hz (2 ms).
        G_UR_Client_Str.time_step = 2;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        switch (state_id)
        {
            case UR_Client_STATE_Enum.DISCONNECTED:
                {
                    G_UR_Client_Str.Is_Connected = false;

                    if (Connect == true)
                    {
                        Disconnect = false;
                        UR_Client_R_Cls.Start();
                        state_id = UR_Client_STATE_Enum.CONNECTED;
                    }
                }
                break;
            case UR_Client_STATE_Enum.CONNECTED:
                {
                    G_UR_Client_Str.Is_Connected = true;

                    if (Disconnect == true)
                    {
                        Connect = false;
                        if (ur_client_r_is_alive == true)
                        {
                            UR_Client_R_Cls.Stop();
                        }

                        if (ur_client_r_is_alive == false)
                        {
                            state_id = UR_Client_STATE_Enum.DISCONNECTED;
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
            // Destroy the threads used to read data via TCP/IP communication.
            UR_Client_R_Cls.Destroy();

            Destroy(this);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
    }

    class UR_Client_Read_Data_Cls
    {
        public enum UR_Client_R_STATE_Enum
        {
            INIT = 0,
            READ = 10
        }

        // Initialization of the class variables.
        private Thread ctrl_thread = null;
        private bool exit_thread = false;
        // TCP/IP client app. configuration class.
        private TcpClient Client_Cls = new TcpClient();
        private NetworkStream network_stream = null;
        // Packet Buffer: Read data.
        private byte[] packet = new byte[1116];
        // Identification number of the class state machine.
        private UR_Client_R_STATE_Enum state_id;

        // Offset: Size of first packet in bytes (Integer).
        private const byte first_packet_size = 4;
        // Offset: Size of other packets in bytes (Double).
        private const byte offset = 8;

        // Total message length in bytes.
        private static List<UInt32> msg_length_list = new List<UInt32>();
        private static UInt32 total_msg_length = 0;

        public void Cls_Core_Thread()
        {
            try
            {
                if (Client_Cls.Connected == false)
                {
                    // Connect to controller.
                    // Note: If the controller is disconnected.
                    Client_Cls.Connect(G_UR_Client_Str.ip_address, G_UR_Client_Str.port_number);
                }

                // Initialization TCP/IP Communication to read the data.
                network_stream = Client_Cls.GetStream();

                // Initialization timer.
                var t = new Stopwatch();

                while (exit_thread == false)
                {
                    switch (state_id)
                    {
                        case UR_Client_R_STATE_Enum.INIT:
                            {
                                // Getting the total message length from several runs of reading data.
                                if (network_stream.Read(packet, 0, packet.Length) != 0)
                                {
                                    if (msg_length_list.Count == 10)
                                    {
                                        msg_length_list.Sort();
                                        total_msg_length = msg_length_list[msg_length_list.Count - 1];
                                        state_id = UR_Client_R_STATE_Enum.READ;
                                    }
                                    else
                                    {
                                        msg_length_list.Add(BitConverter.ToUInt32(packet, first_packet_size - 4));
                                    }
                                }

                            }
                            break;

                        case UR_Client_R_STATE_Enum.READ:
                            {
                                // Get the data from the robot.
                                if (network_stream.Read(packet, 0, packet.Length) != 0)
                                {
                                    if (BitConverter.ToUInt32(packet, first_packet_size - 4) == total_msg_length)
                                    {
                                        // t_{0}: Timer start.
                                        t.Start();

                                        // Reverses the order of elements in a one-dimensional array or part of an array.
                                        Array.Reverse(packet);

                                        // Note:
                                        //  For more information on values 32... 37, etc., see the UR Client Interface document.
                                        // Read Joint Values in radians
                                        G_UR_Client_Str.J_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (32 * offset));
                                        G_UR_Client_Str.J_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (33 * offset));
                                        G_UR_Client_Str.J_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (34 * offset));
                                        G_UR_Client_Str.J_Orientation[3] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (35 * offset));
                                        G_UR_Client_Str.J_Orientation[4] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (36 * offset));
                                        G_UR_Client_Str.J_Orientation[5] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (37 * offset));

                                        // t_{1}: Timer stop.
                                        t.Stop();

                                        // Recalculate the time: t = t_{1} - t_{0} -> Elapsed Time in milliseconds.
                                        if (t.ElapsedMilliseconds < G_UR_Client_Str.time_step)
                                        {
                                            Thread.Sleep(G_UR_Client_Str.time_step - (int)t.ElapsedMilliseconds);
                                        }

                                        // Reset (Restart) timer.
                                        t.Restart();
                                    }
                                }
                            }
                            break;
                    }
                }
            }
            catch (SocketException e)
            {
                Debug.LogException(e);
            }
        }

        public void Start()
        {
            exit_thread = false;

            // Start a thread and listen to incoming messages.
            ctrl_thread = new Thread(new ThreadStart(Cls_Core_Thread));
            ctrl_thread.IsBackground = true;
            ctrl_thread.Start();

            // Thread is active.
            ur_client_r_is_alive = true;
        }
        public void Stop()
        {
            exit_thread = true;
            ctrl_thread.Abort();
            // Stop a thread.
            Thread.Sleep(100);

            // The thread is inactive.
            ur_client_r_is_alive = ctrl_thread.IsAlive;
        }
        public void Destroy()
        {
            if (Client_Cls.Connected == true)
            {
                // Stop a thread (TCP/IP Communication).
                network_stream.Dispose();
                Client_Cls.Close();
            }
            Thread.Sleep(100);
        }
    }
}
