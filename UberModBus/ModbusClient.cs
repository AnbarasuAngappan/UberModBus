using System;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace UberModBus
{
    public class ModbusClient
    {
        #region VariableDeclaration
        private bool debug = false;
        private string ipAddress = "127.0.0.1";
        private int port = 502;
        private uint transactionIdentifierInternal = 0;
        private byte[] transactionIdentifier = new byte[2];
        private byte[] protocolIdentifier = new byte[2];
        private byte[] crc = new byte[2];
        private byte[] length = new byte[2];
        private byte unitIdentifier = 1;
        private byte[] startingAddress = new byte[2];
        private byte[] quantity = new byte[2];
        private bool udpFlag = false;
        private int baudRate = 9600;
        private int connectTimeout = 1000;
        private Parity parity = Parity.Even;
        private StopBits stopBits = StopBits.One;
        private bool connected = false;
        private bool dataReceived = false;
        private bool receiveActive = false;
        private byte[] readBuffer = new byte[256];
        private int bytesToRead = 0;
        private TcpClient tcpClient;
        private byte functionCode;
        private int portOut;
        public byte[] receiveData;
        public byte[] sendData;
        private SerialPort serialport;
        private NetworkStream stream;

        public event ModbusClient.ReceiveDataChanged receiveDataChanged;

        public event ModbusClient.SendDataChanged sendDataChanged;

        public event ModbusClient.ConnectedChanged connectedChanged;

        #endregion

        public ModbusClient(string ipAddress, int port)
        {
            if (this.debug)
                StoreLogData.Instance.Store("EasyModbus library initialized for Modbus-TCP, IPAddress: " + ipAddress + ", Port: " + (object)port, DateTime.Now);
            Console.WriteLine("UberModbus Client Library Version: " + Assembly.GetExecutingAssembly().GetName().Version.ToString());
            //Console.WriteLine("Copyright (c) Stefan Rossmann Engineering Solutions");
            Console.WriteLine();
            this.ipAddress = ipAddress;
            this.port = port;
        }

        public ModbusClient(string serialPort)
        {
            if (this.debug)
                StoreLogData.Instance.Store("EasyModbus library initialized for Modbus-RTU, COM-Port: " + serialPort, DateTime.Now);
            Console.WriteLine("EasyModbus Client Library Version: " + Assembly.GetExecutingAssembly().GetName().Version.ToString());
            Console.WriteLine("Copyright (c) Stefan Rossmann Engineering Solutions");
            Console.WriteLine();
            this.serialport = new SerialPort();
            this.serialport.PortName = serialPort;
            this.serialport.BaudRate = this.baudRate;
            this.serialport.Parity = this.parity;
            this.serialport.StopBits = this.stopBits;
            this.serialport.WriteTimeout = 10000;
            this.serialport.ReadTimeout = this.connectTimeout;
            this.serialport.DataReceived += new SerialDataReceivedEventHandler(this.DataReceivedHandler);
        }

        public ModbusClient()
        {
            if (this.debug)
                StoreLogData.Instance.Store("EasyModbus library initialized for Modbus-TCP", DateTime.Now);
            Console.WriteLine("EasyModbus Client Library Version: " + Assembly.GetExecutingAssembly().GetName().Version.ToString());
            Console.WriteLine("Copyright (c) Stefan Rossmann Engineering Solutions");
            Console.WriteLine();
        }

        public int[] ReadInputRegisters(int startingAddress, int quantity)
        {
            if (this.debug)
                StoreLogData.Instance.Store("FC4 (Read Input Registers from Master device), StartingAddress: " + (object)startingAddress + ", Quantity: " + (object)quantity, DateTime.Now);
            checked { ++this.transactionIdentifierInternal; }
            if (this.serialport != null && !this.serialport.IsOpen)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("SerialPortNotOpenedException Throwed", DateTime.Now);
                //throw new SerialPortNotOpenedException("serial port not opened");
            }
            if (this.tcpClient == null & !this.udpFlag & this.serialport == null)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ConnectionException Throwed", DateTime.Now);
                //throw new ConnectionException("connection error");
            }
            if (startingAddress > (int)ushort.MaxValue | quantity > 125)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ArgumentException Throwed", DateTime.Now);
                throw new ArgumentException("Starting address must be 0 - 65535; quantity must be 0 - 125");
            }
            this.transactionIdentifier = BitConverter.GetBytes(this.transactionIdentifierInternal);
            this.protocolIdentifier = BitConverter.GetBytes(0);
            this.length = BitConverter.GetBytes(6);
            this.functionCode = (byte)4;
            this.startingAddress = BitConverter.GetBytes(startingAddress);
            this.quantity = BitConverter.GetBytes(quantity);
            byte[] numArray1 = new byte[14]
            {
                this.transactionIdentifier[1],
                this.transactionIdentifier[0],
                this.protocolIdentifier[1],
                this.protocolIdentifier[0],
                this.length[1],
                this.length[0],
                this.unitIdentifier,
                this.functionCode,
                this.startingAddress[1],
                this.startingAddress[0],
                this.quantity[1],
                this.quantity[0],
                this.crc[0],
                this.crc[1]
            };
            this.crc = BitConverter.GetBytes(ModbusClient.calculateCRC(numArray1, (ushort)6, 6));
            numArray1[12] = this.crc[0];
            numArray1[13] = this.crc[1];
            if (this.serialport != null)
            {
                this.dataReceived = false;
                this.bytesToRead = checked(5 + 2 * quantity);
                this.serialport.Write(numArray1, 6, 8);
                if (this.debug)
                {
                    byte[] numArray2 = new byte[8];
                    Array.Copy((Array)numArray1, 6, (Array)numArray2, 0, 8);
                    if (this.debug)
                        StoreLogData.Instance.Store("Send Serial-Data: " + BitConverter.ToString(numArray2), DateTime.Now);
                }
                // ISSUE: reference to a compiler-generated field
                if (this.sendDataChanged != null)
                {
                    this.sendData = new byte[8];
                    Array.Copy((Array)numArray1, 6, (Array)this.sendData, 0, 8);
                    // ISSUE: reference to a compiler-generated field
                    this.sendDataChanged((object)this);
                }
                DateTime now1 = DateTime.Now;
                byte maxValue = byte.MaxValue;
                while (true)
                {
                    int num1 = (int)maxValue != (int)this.unitIdentifier ? 1 : 0;
                    DateTime now2 = DateTime.Now;
                    int num2 = checked(now2.Ticks - now1.Ticks) <= checked(10000L * (long)this.connectTimeout) ? 1 : 0;
                    if ((num1 & num2) != 0)
                    {
                        while (true)
                        {
                            int num3 = !this.dataReceived ? 1 : 0;
                            now2 = DateTime.Now;
                            int num4 = checked(now2.Ticks - now1.Ticks) <= checked(10000L * (long)this.connectTimeout) ? 1 : 0;
                            if ((num3 & num4) != 0)
                                Thread.Sleep(1);
                            else
                                break;
                        }
                        numArray1 = new byte[2100];
                        Array.Copy((Array)this.readBuffer, 0, (Array)numArray1, 6, this.readBuffer.Length);
                        maxValue = numArray1[6];
                    }
                    else
                        break;
                }
                if ((int)maxValue != (int)this.unitIdentifier)
                    numArray1 = new byte[2100];
            }
            else if (this.tcpClient.Client.Connected | this.udpFlag)
            {
                if (this.udpFlag)
                {
                    UdpClient udpClient = new UdpClient();
                    IPEndPoint endPoint = new IPEndPoint(System.Net.IPAddress.Parse(this.ipAddress), this.port);
                    udpClient.Send(numArray1, checked(numArray1.Length - 2), endPoint);
                    this.portOut = ((IPEndPoint)udpClient.Client.LocalEndPoint).Port;
                    udpClient.Client.ReceiveTimeout = 5000;
                    IPEndPoint remoteEP = new IPEndPoint(System.Net.IPAddress.Parse(this.ipAddress), this.portOut);
                    numArray1 = udpClient.Receive(ref remoteEP);
                }
                else
                {
                    this.stream.Write(numArray1, 0, checked(numArray1.Length - 2));
                    if (this.debug)
                    {
                        byte[] numArray2 = new byte[checked(numArray1.Length - 2)];
                        Array.Copy((Array)numArray1, 0, (Array)numArray2, 0, checked(numArray1.Length - 2));
                        if (this.debug)
                            StoreLogData.Instance.Store("Send ModbusTCP-Data: " + BitConverter.ToString(numArray2), DateTime.Now);
                    }
                    // ISSUE: reference to a compiler-generated field
                    if (this.sendDataChanged != null)
                    {
                        this.sendData = new byte[checked(numArray1.Length - 2)];
                        Array.Copy((Array)numArray1, 0, (Array)this.sendData, 0, checked(numArray1.Length - 2));
                        // ISSUE: reference to a compiler-generated field
                        this.sendDataChanged((object)this);
                    }
                    numArray1 = new byte[2100];
                    int length = this.stream.Read(numArray1, 0, numArray1.Length);
                    // ISSUE: reference to a compiler-generated field
                    if (this.receiveDataChanged != null)
                    {
                        this.receiveData = new byte[length];
                        Array.Copy((Array)numArray1, 0, (Array)this.receiveData, 0, length);
                        if (this.debug)
                            StoreLogData.Instance.Store("Receive ModbusTCP-Data: " + BitConverter.ToString(this.receiveData), DateTime.Now);
                        // ISSUE: reference to a compiler-generated field
                        this.receiveDataChanged((object)this);
                    }
                }
            }
            if (numArray1[7] == (byte)132 & numArray1[8] == (byte)1)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("FunctionCodeNotSupportedException Throwed", DateTime.Now);
                //throw new FunctionCodeNotSupportedException("Function code not supported by master");
            }
            if (numArray1[7] == (byte)132 & numArray1[8] == (byte)2)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("StartingAddressInvalidException Throwed", DateTime.Now);
                //throw new StartingAddressInvalidException("Starting address invalid or starting address + quantity invalid");
            }
            if (numArray1[7] == (byte)132 & numArray1[8] == (byte)3)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("QuantityInvalidException Throwed", DateTime.Now);
                //throw new QuantityInvalidException("quantity invalid");
            }
            if (numArray1[7] == (byte)132 & numArray1[8] == (byte)4)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ModbusException Throwed", DateTime.Now);
                //throw new ModbusException("error reading");
            }
            if (this.serialport != null)
            {
                this.crc = BitConverter.GetBytes(ModbusClient.calculateCRC(numArray1, checked((ushort)((int)numArray1[8] + 3)), 6));
                if (((int)this.crc[0] != (int)numArray1[checked((int)numArray1[8] + 9)] | (int)this.crc[1] != (int)numArray1[checked((int)numArray1[8] + 10)]) & this.dataReceived)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("CRCCheckFailedException Throwed", DateTime.Now);
                    //throw new CRCCheckFailedException("Response CRC check failed");
                }
                if (!this.dataReceived)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("TimeoutException Throwed", DateTime.Now);
                    throw new TimeoutException("No Response from Modbus Slave");
                }
            }
            int[] numArray3 = new int[quantity];
            int index = 0;
            while (index < quantity)
            {
                byte num1 = numArray1[checked(9 + index * 2)];
                byte num2 = numArray1[checked(9 + index * 2 + 1)];
                numArray1[checked(9 + index * 2)] = num2;
                numArray1[checked(9 + index * 2 + 1)] = num1;
                numArray3[index] = (int)BitConverter.ToInt16(numArray1, checked(9 + index * 2));
                checked { ++index; }
            }
            return numArray3;
        }

        public int[] ReadHoldingRegisters(int startingAddress, int quantity)
        {
            if (this.debug)
                StoreLogData.Instance.Store("FC3 (Read Holding Registers from Master device), StartingAddress: " + (object)startingAddress + ", Quantity: " + (object)quantity, DateTime.Now);
            checked { ++this.transactionIdentifierInternal; }
            if (this.serialport != null && !this.serialport.IsOpen)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("SerialPortNotOpenedException Throwed", DateTime.Now);
                throw new Exception("serial port not opened");//throw new SerialPortNotOpenedException("serial port not opened");
            }
            if (this.tcpClient == null & !this.udpFlag & this.serialport == null)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ConnectionException Throwed", DateTime.Now);
                throw new Exception("connection error"); //throw new ConnectionException("connection error");
            }
            if (startingAddress > (int)ushort.MaxValue | quantity > 125)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ArgumentException Throwed", DateTime.Now);
                throw new ArgumentException("Starting address must be 0 - 65535; quantity must be 0 - 125");
            }
            this.transactionIdentifier = BitConverter.GetBytes(this.transactionIdentifierInternal);
            this.protocolIdentifier = BitConverter.GetBytes(0);
            this.length = BitConverter.GetBytes(6);
            this.functionCode = (byte)3;
            this.startingAddress = BitConverter.GetBytes(startingAddress);
            this.quantity = BitConverter.GetBytes(quantity);
            byte[] numArray1 = new byte[14]
            {
                this.transactionIdentifier[1],
                this.transactionIdentifier[0],
                this.protocolIdentifier[1],
                this.protocolIdentifier[0],
                this.length[1],
                this.length[0],
                this.unitIdentifier,
                this.functionCode,
                this.startingAddress[1],
                this.startingAddress[0],
                this.quantity[1],
                this.quantity[0],
                this.crc[0],
                this.crc[1]
            };
            this.crc = BitConverter.GetBytes(ModbusClient.calculateCRC(numArray1, (ushort)6, 6));
            numArray1[12] = this.crc[0];
            numArray1[13] = this.crc[1];
            if (this.serialport != null)
            {
                this.dataReceived = false;
                this.bytesToRead = checked(5 + 2 * quantity);
                this.serialport.Write(numArray1, 6, 8);
                if (this.debug)
                {
                    byte[] numArray2 = new byte[8];
                    Array.Copy((Array)numArray1, 6, (Array)numArray2, 0, 8);
                    if (this.debug)
                        StoreLogData.Instance.Store("Send Serial-Data: " + BitConverter.ToString(numArray2), DateTime.Now);
                }
                // ISSUE: reference to a compiler-generated field
                if (this.sendDataChanged != null)
                {
                    this.sendData = new byte[8];
                    Array.Copy((Array)numArray1, 6, (Array)this.sendData, 0, 8);
                    // ISSUE: reference to a compiler-generated field
                    this.sendDataChanged((object)this);
                }
                DateTime now1 = DateTime.Now;
                byte maxValue = byte.MaxValue;
                while (true)
                {
                    int num1 = (int)maxValue != (int)this.unitIdentifier ? 1 : 0;
                    DateTime now2 = DateTime.Now;
                    int num2 = checked(now2.Ticks - now1.Ticks) <= checked(10000L * (long)this.connectTimeout) ? 1 : 0;
                    if ((num1 & num2) != 0)
                    {
                        while (true)
                        {
                            int num3 = !this.dataReceived ? 1 : 0;
                            now2 = DateTime.Now;
                            int num4 = checked(now2.Ticks - now1.Ticks) <= checked(10000L * (long)this.connectTimeout) ? 1 : 0;
                            if ((num3 & num4) != 0)
                                Thread.Sleep(1);
                            else
                                break;
                        }
                        numArray1 = new byte[2100];
                        Array.Copy((Array)this.readBuffer, 0, (Array)numArray1, 6, this.readBuffer.Length);
                        maxValue = numArray1[6];
                    }
                    else
                        break;
                }
                if ((int)maxValue != (int)this.unitIdentifier)
                    numArray1 = new byte[2100];
            }
            else if (this.tcpClient.Client.Connected | this.udpFlag)
            {
                if (this.udpFlag)
                {
                    UdpClient udpClient = new UdpClient();
                    IPEndPoint endPoint = new IPEndPoint(System.Net.IPAddress.Parse(this.ipAddress), this.port);
                    udpClient.Send(numArray1, checked(numArray1.Length - 2), endPoint);
                    this.portOut = ((IPEndPoint)udpClient.Client.LocalEndPoint).Port;
                    udpClient.Client.ReceiveTimeout = 5000;
                    IPEndPoint remoteEP = new IPEndPoint(System.Net.IPAddress.Parse(this.ipAddress), this.portOut);
                    numArray1 = udpClient.Receive(ref remoteEP);
                }
                else
                {
                    this.stream.Write(numArray1, 0, checked(numArray1.Length - 2));
                    if (this.debug)
                    {
                        byte[] numArray2 = new byte[checked(numArray1.Length - 2)];
                        Array.Copy((Array)numArray1, 0, (Array)numArray2, 0, checked(numArray1.Length - 2));
                        if (this.debug)
                            StoreLogData.Instance.Store("Send ModbusTCP-Data: " + BitConverter.ToString(numArray2), DateTime.Now);
                    }
                    // ISSUE: reference to a compiler-generated field
                    if (this.sendDataChanged != null)
                    {
                        this.sendData = new byte[checked(numArray1.Length - 2)];
                        Array.Copy((Array)numArray1, 0, (Array)this.sendData, 0, checked(numArray1.Length - 2));
                        // ISSUE: reference to a compiler-generated field
                        this.sendDataChanged((object)this);
                    }
                    numArray1 = new byte[256];
                    int length = this.stream.Read(numArray1, 0, numArray1.Length);
                    // ISSUE: reference to a compiler-generated field
                    if (this.receiveDataChanged != null)
                    {
                        this.receiveData = new byte[length];
                        Array.Copy((Array)numArray1, 0, (Array)this.receiveData, 0, length);
                        if (this.debug)
                            StoreLogData.Instance.Store("Receive ModbusTCP-Data: " + BitConverter.ToString(this.receiveData), DateTime.Now);
                        // ISSUE: reference to a compiler-generated field
                        this.receiveDataChanged((object)this);
                    }
                }
            }
            if (numArray1[7] == (byte)131 & numArray1[8] == (byte)1)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("FunctionCodeNotSupportedException Throwed", DateTime.Now);
                throw new Exception("Function code not supported by master"); //throw new FunctionCodeNotSupportedException("Function code not supported by master");
            }
            if (numArray1[7] == (byte)131 & numArray1[8] == (byte)2)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("StartingAddressInvalidException Throwed", DateTime.Now);
                throw new Exception("Starting address invalid or starting address + quantity invalid"); //throw new StartingAddressInvalidException("Starting address invalid or starting address + quantity invalid");
            }
            if (numArray1[7] == (byte)131 & numArray1[8] == (byte)3)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("QuantityInvalidException Throwed", DateTime.Now);
                throw new Exception("quantity invalid"); //throw new QuantityInvalidException("quantity invalid");
            }
            if (numArray1[7] == (byte)131 & numArray1[8] == (byte)4)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ModbusException Throwed", DateTime.Now);
                throw new Exception("error reading"); //throw new ModbusException("error reading");
            }
            if (this.serialport != null)
            {
                this.crc = BitConverter.GetBytes(ModbusClient.calculateCRC(numArray1, checked((ushort)((int)numArray1[8] + 3)), 6));
                if (((int)this.crc[0] != (int)numArray1[checked((int)numArray1[8] + 9)] | (int)this.crc[1] != (int)numArray1[checked((int)numArray1[8] + 10)]) & this.dataReceived)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("CRCCheckFailedException Throwed", DateTime.Now);
                    throw new Exception("Response CRC check failed"); //throw new CRCCheckFailedException("Response CRC check failed");
                }
                if (!this.dataReceived)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("TimeoutException Throwed", DateTime.Now);
                    throw new TimeoutException("No Response from Modbus Slave");
                }
            }
            int[] numArray3 = new int[quantity];
            int index = 0;
            while (index < quantity)
            {
                byte num1 = numArray1[checked(9 + index * 2)];
                byte num2 = numArray1[checked(9 + index * 2 + 1)];
                numArray1[checked(9 + index * 2)] = num2;
                numArray1[checked(9 + index * 2 + 1)] = num1;
                numArray3[index] = (int)BitConverter.ToInt16(numArray1, checked(9 + index * 2));
                checked { ++index; }
            }
            return numArray3;
        }

        public bool[] ReadCoils(int startingAddress, int quantity)
        {
            if (this.debug)
                StoreLogData.Instance.Store("FC1 (Read Coils from Master device), StartingAddress: " + (object)startingAddress + ", Quantity: " + (object)quantity, DateTime.Now);
            checked { ++this.transactionIdentifierInternal; }
            if (this.serialport != null && !this.serialport.IsOpen)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("SerialPortNotOpenedException Throwed", DateTime.Now);
                throw new Exception("serial port not opened");//throw new SerialPortNotOpenedException("serial port not opened");
            }
            if (this.tcpClient == null & !this.udpFlag & this.serialport == null)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ConnectionException Throwed", DateTime.Now);
                throw new Exception("connection error");// throw new ConnectionException("connection error");
            }
            if (startingAddress > (int)ushort.MaxValue | quantity > 2000)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ArgumentException Throwed", DateTime.Now);
                throw new ArgumentException("Starting address must be 0 - 65535; quantity must be 0 - 2000");
            }
            this.transactionIdentifier = BitConverter.GetBytes(this.transactionIdentifierInternal);
            this.protocolIdentifier = BitConverter.GetBytes(0);
            this.length = BitConverter.GetBytes(6);
            this.functionCode = (byte)1;
            this.startingAddress = BitConverter.GetBytes(startingAddress);
            this.quantity = BitConverter.GetBytes(quantity);
            byte[] numArray1 = new byte[14]
            {
                this.transactionIdentifier[1],
                this.transactionIdentifier[0],
                this.protocolIdentifier[1],
                this.protocolIdentifier[0],
                this.length[1],
                this.length[0],
                this.unitIdentifier,
                this.functionCode,
                this.startingAddress[1],
                this.startingAddress[0],
                this.quantity[1],
                this.quantity[0],
                this.crc[0],
                this.crc[1]
            };
            this.crc = BitConverter.GetBytes(ModbusClient.calculateCRC(numArray1, (ushort)6, 6));
            numArray1[12] = this.crc[0];
            numArray1[13] = this.crc[1];
            if (this.serialport != null)
            {
                this.dataReceived = false;
                this.bytesToRead = quantity % 8 != 0 ? checked(6 + unchecked(quantity / 8)) : checked(5 + unchecked(quantity / 8));
                this.serialport.Write(numArray1, 6, 8);
                if (this.debug)
                {
                    byte[] numArray2 = new byte[8];
                    Array.Copy((Array)numArray1, 6, (Array)numArray2, 0, 8);
                    if (this.debug)
                        StoreLogData.Instance.Store("Send Serial-Data: " + BitConverter.ToString(numArray2), DateTime.Now);
                }
                // ISSUE: reference to a compiler-generated field
                if (this.sendDataChanged != null)
                {
                    this.sendData = new byte[8];
                    Array.Copy((Array)numArray1, 6, (Array)this.sendData, 0, 8);
                    // ISSUE: reference to a compiler-generated field
                    this.sendDataChanged((object)this);
                }
                DateTime now1 = DateTime.Now;
                byte maxValue = byte.MaxValue;
                while (true)
                {
                    int num1 = (int)maxValue != (int)this.unitIdentifier ? 1 : 0;
                    DateTime now2 = DateTime.Now;
                    int num2 = checked(now2.Ticks - now1.Ticks) <= checked(10000L * (long)this.connectTimeout) ? 1 : 0;
                    if ((num1 & num2) != 0)
                    {
                        while (true)
                        {
                            int num3 = !this.dataReceived ? 1 : 0;
                            now2 = DateTime.Now;
                            int num4 = checked(now2.Ticks - now1.Ticks) <= checked(10000L * (long)this.connectTimeout) ? 1 : 0;
                            if ((num3 & num4) != 0)
                                Thread.Sleep(1);
                            else
                                break;
                        }
                        numArray1 = new byte[2100];
                        Array.Copy((Array)this.readBuffer, 0, (Array)numArray1, 6, this.readBuffer.Length);
                        maxValue = numArray1[6];
                    }
                    else
                        break;
                }
                if ((int)maxValue != (int)this.unitIdentifier)
                    numArray1 = new byte[2100];
            }
            else if (this.tcpClient.Client.Connected | this.udpFlag)
            {
                if (this.udpFlag)
                {
                    UdpClient udpClient = new UdpClient();
                    IPEndPoint endPoint = new IPEndPoint(System.Net.IPAddress.Parse(this.ipAddress), this.port);
                    udpClient.Send(numArray1, checked(numArray1.Length - 2), endPoint);
                    this.portOut = ((IPEndPoint)udpClient.Client.LocalEndPoint).Port;
                    udpClient.Client.ReceiveTimeout = 5000;
                    IPEndPoint remoteEP = new IPEndPoint(System.Net.IPAddress.Parse(this.ipAddress), this.portOut);
                    numArray1 = udpClient.Receive(ref remoteEP);
                }
                else
                {
                    this.stream.Write(numArray1, 0, checked(numArray1.Length - 2));
                    if (this.debug)
                    {
                        byte[] numArray2 = new byte[checked(numArray1.Length - 2)];
                        Array.Copy((Array)numArray1, 0, (Array)numArray2, 0, checked(numArray1.Length - 2));
                        if (this.debug)
                            StoreLogData.Instance.Store("Send MocbusTCP-Data: " + BitConverter.ToString(numArray2), DateTime.Now);
                    }
                    // ISSUE: reference to a compiler-generated field
                    if (this.sendDataChanged != null)
                    {
                        this.sendData = new byte[checked(numArray1.Length - 2)];
                        Array.Copy((Array)numArray1, 0, (Array)this.sendData, 0, checked(numArray1.Length - 2));
                        // ISSUE: reference to a compiler-generated field
                        this.sendDataChanged((object)this);
                    }
                    numArray1 = new byte[2100];
                    int length = this.stream.Read(numArray1, 0, numArray1.Length);
                    // ISSUE: reference to a compiler-generated field
                    if (this.receiveDataChanged != null)
                    {
                        this.receiveData = new byte[length];
                        Array.Copy((Array)numArray1, 0, (Array)this.receiveData, 0, length);
                        if (this.debug)
                            StoreLogData.Instance.Store("Receive ModbusTCP-Data: " + BitConverter.ToString(this.receiveData), DateTime.Now);
                        // ISSUE: reference to a compiler-generated field
                        this.receiveDataChanged((object)this);
                    }
                }
            }
            if (numArray1[7] == (byte)129 & numArray1[8] == (byte)1)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("FunctionCodeNotSupportedException Throwed", DateTime.Now);
                throw new Exception("Function code not supported by master");//throw new FunctionCodeNotSupportedException("Function code not supported by master");
            }
            if (numArray1[7] == (byte)129 & numArray1[8] == (byte)2)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("StartingAddressInvalidException Throwed", DateTime.Now);
                throw new Exception("Starting address invalid or starting address + quantity invalid"); //throw new StartingAddressInvalidException("Starting address invalid or starting address + quantity invalid");
            }
            if (numArray1[7] == (byte)129 & numArray1[8] == (byte)3)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("QuantityInvalidException Throwed", DateTime.Now);
                throw new Exception("quantity invalid"); //throw new QuantityInvalidException("quantity invalid");
            }
            if (numArray1[7] == (byte)129 & numArray1[8] == (byte)4)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ModbusException Throwed", DateTime.Now);
                throw new Exception("error reading"); //throw new ModbusException("error reading");
            }
            if (this.serialport != null)
            {
                this.crc = BitConverter.GetBytes(ModbusClient.calculateCRC(numArray1, checked((ushort)((int)numArray1[8] + 3)), 6));
                if (((int)this.crc[0] != (int)numArray1[checked((int)numArray1[8] + 9)] | (int)this.crc[1] != (int)numArray1[checked((int)numArray1[8] + 10)]) & this.dataReceived)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("CRCCheckFailedException Throwed", DateTime.Now);
                    throw new Exception("Response CRC check failed"); //throw new CRCCheckFailedException("Response CRC check failed");
                }
                if (!this.dataReceived)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("TimeoutException Throwed", DateTime.Now);
                    throw new TimeoutException("No Response from Modbus Slave");
                }
            }
            bool[] flagArray = new bool[quantity];
            int index = 0;
            while (index < quantity)
            {
                int num = (int)numArray1[checked(9 + unchecked(index / 8))];
                int int32 = Convert.ToInt32(Math.Pow(2.0, (double)(index % 8)));
                flagArray[index] = Convert.ToBoolean((num & int32) / int32);
                checked { ++index; }
            }
            return flagArray;
        }

        public bool[] ReadDiscreteInputs(int startingAddress, int quantity)
        {
            if (this.debug)
                StoreLogData.Instance.Store("FC2 (Read Discrete Inputs from Master device), StartingAddress: " + (object)startingAddress + ", Quantity: " + (object)quantity, DateTime.Now);
            checked { ++this.transactionIdentifierInternal; }
            if (this.serialport != null && !this.serialport.IsOpen)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("SerialPortNotOpenedException Throwed", DateTime.Now);
                throw new Exception("serial port not opened");// throw new SerialPortNotOpenedException("serial port not opened");
            }
            if (this.tcpClient == null & !this.udpFlag & this.serialport == null)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ConnectionException Throwed", DateTime.Now);
                throw new Exception("connection error"); //throw new ConnectionException("connection error");
            }
            if (startingAddress > (int)ushort.MaxValue | quantity > 2000)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ArgumentException Throwed", DateTime.Now);
                throw new ArgumentException("Starting address must be 0 - 65535; quantity must be 0 - 2000");
            }
            this.transactionIdentifier = BitConverter.GetBytes(this.transactionIdentifierInternal);
            this.protocolIdentifier = BitConverter.GetBytes(0);
            this.length = BitConverter.GetBytes(6);
            this.functionCode = (byte)2;
            this.startingAddress = BitConverter.GetBytes(startingAddress);
            this.quantity = BitConverter.GetBytes(quantity);
            byte[] numArray1 = new byte[14]
            {
                this.transactionIdentifier[1],
                this.transactionIdentifier[0],
                this.protocolIdentifier[1],
                this.protocolIdentifier[0],
                this.length[1],
                this.length[0],
                this.unitIdentifier,
                this.functionCode,
                this.startingAddress[1],
                this.startingAddress[0],
                this.quantity[1],
                this.quantity[0],
                this.crc[0],
                this.crc[1]
            };
            this.crc = BitConverter.GetBytes(ModbusClient.calculateCRC(numArray1, (ushort)6, 6));
            numArray1[12] = this.crc[0];
            numArray1[13] = this.crc[1];
            if (this.serialport != null)
            {
                this.dataReceived = false;
                this.bytesToRead = quantity % 8 != 0 ? checked(6 + unchecked(quantity / 8)) : checked(5 + unchecked(quantity / 8));
                this.serialport.Write(numArray1, 6, 8);
                if (this.debug)
                {
                    byte[] numArray2 = new byte[8];
                    Array.Copy((Array)numArray1, 6, (Array)numArray2, 0, 8);
                    if (this.debug)
                        StoreLogData.Instance.Store("Send Serial-Data: " + BitConverter.ToString(numArray2), DateTime.Now);
                }
                // ISSUE: reference to a compiler-generated field
                if (this.sendDataChanged != null)
                {
                    this.sendData = new byte[8];
                    Array.Copy((Array)numArray1, 6, (Array)this.sendData, 0, 8);
                    // ISSUE: reference to a compiler-generated field
                    this.sendDataChanged((object)this);
                }
                DateTime now1 = DateTime.Now;
                byte maxValue = byte.MaxValue;
                while (true)
                {
                    int num1 = (int)maxValue != (int)this.unitIdentifier ? 1 : 0;
                    DateTime now2 = DateTime.Now;
                    int num2 = checked(now2.Ticks - now1.Ticks) <= checked(10000L * (long)this.connectTimeout) ? 1 : 0;
                    if ((num1 & num2) != 0)
                    {
                        while (true)
                        {
                            int num3 = !this.dataReceived ? 1 : 0;
                            now2 = DateTime.Now;
                            int num4 = checked(now2.Ticks - now1.Ticks) <= checked(10000L * (long)this.connectTimeout) ? 1 : 0;
                            if ((num3 & num4) != 0)
                                Thread.Sleep(1);
                            else
                                break;
                        }
                        numArray1 = new byte[2100];
                        Array.Copy((Array)this.readBuffer, 0, (Array)numArray1, 6, this.readBuffer.Length);
                        maxValue = numArray1[6];
                    }
                    else
                        break;
                }
                if ((int)maxValue != (int)this.unitIdentifier)
                    numArray1 = new byte[2100];
            }
            else if (this.tcpClient.Client.Connected | this.udpFlag)
            {
                if (this.udpFlag)
                {
                    UdpClient udpClient = new UdpClient();
                    IPEndPoint endPoint = new IPEndPoint(System.Net.IPAddress.Parse(this.ipAddress), this.port);
                    udpClient.Send(numArray1, checked(numArray1.Length - 2), endPoint);
                    this.portOut = ((IPEndPoint)udpClient.Client.LocalEndPoint).Port;
                    udpClient.Client.ReceiveTimeout = 5000;
                    IPEndPoint remoteEP = new IPEndPoint(System.Net.IPAddress.Parse(this.ipAddress), this.portOut);
                    numArray1 = udpClient.Receive(ref remoteEP);
                }
                else
                {
                    this.stream.Write(numArray1, 0, checked(numArray1.Length - 2));
                    if (this.debug)
                    {
                        byte[] numArray2 = new byte[checked(numArray1.Length - 2)];
                        Array.Copy((Array)numArray1, 0, (Array)numArray2, 0, checked(numArray1.Length - 2));
                        if (this.debug)
                            StoreLogData.Instance.Store("Send ModbusTCP-Data: " + BitConverter.ToString(numArray2), DateTime.Now);
                    }
                    // ISSUE: reference to a compiler-generated field
                    if (this.sendDataChanged != null)
                    {
                        this.sendData = new byte[checked(numArray1.Length - 2)];
                        Array.Copy((Array)numArray1, 0, (Array)this.sendData, 0, checked(numArray1.Length - 2));
                        // ISSUE: reference to a compiler-generated field
                        this.sendDataChanged((object)this);
                    }
                    numArray1 = new byte[2100];
                    int length = this.stream.Read(numArray1, 0, numArray1.Length);
                    // ISSUE: reference to a compiler-generated field
                    if (this.receiveDataChanged != null)
                    {
                        this.receiveData = new byte[length];
                        Array.Copy((Array)numArray1, 0, (Array)this.receiveData, 0, length);
                        if (this.debug)
                            StoreLogData.Instance.Store("Receive ModbusTCP-Data: " + BitConverter.ToString(this.receiveData), DateTime.Now);
                        // ISSUE: reference to a compiler-generated field
                        this.receiveDataChanged((object)this);
                    }
                }
            }
            if (numArray1[7] == (byte)130 & numArray1[8] == (byte)1)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("FunctionCodeNotSupportedException Throwed", DateTime.Now);
                throw new Exception("Function code not supported by master"); //throw new FunctionCodeNotSupportedException("Function code not supported by master");
            }
            if (numArray1[7] == (byte)130 & numArray1[8] == (byte)2)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("StartingAddressInvalidException Throwed", DateTime.Now);
                throw new Exception("Starting address invalid or starting address + quantity invalid"); //throw new StartingAddressInvalidException("Starting address invalid or starting address + quantity invalid");
            }
            if (numArray1[7] == (byte)130 & numArray1[8] == (byte)3)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("QuantityInvalidException Throwed", DateTime.Now);
                throw new Exception("quantity invalid"); //throw new QuantityInvalidException("quantity invalid");
            }
            if (numArray1[7] == (byte)130 & numArray1[8] == (byte)4)
            {
                if (this.debug)
                    StoreLogData.Instance.Store("ModbusException Throwed", DateTime.Now);
                throw new Exception("error reading");// throw new ModbusException("error reading");
            }
            if (this.serialport != null)
            {
                this.crc = BitConverter.GetBytes(ModbusClient.calculateCRC(numArray1, checked((ushort)((int)numArray1[8] + 3)), 6));
                if (((int)this.crc[0] != (int)numArray1[checked((int)numArray1[8] + 9)] | (int)this.crc[1] != (int)numArray1[checked((int)numArray1[8] + 10)]) & this.dataReceived)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("CRCCheckFailedException Throwed", DateTime.Now);
                    throw new Exception("Response CRC check failed"); //throw new CRCCheckFailedException("Response CRC check failed");
                }
                if (!this.dataReceived)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("TimeoutException Throwed", DateTime.Now);
                    throw new TimeoutException("No Response from Modbus Slave");
                }
            }
            bool[] flagArray = new bool[quantity];
            int index = 0;
            while (index < quantity)
            {
                int num = (int)numArray1[checked(9 + unchecked(index / 8))];
                int int32 = Convert.ToInt32(Math.Pow(2.0, (double)(index % 8)));
                flagArray[index] = Convert.ToBoolean((num & int32) / int32);
                checked { ++index; }
            }
            return flagArray;
        }      

        public void Connect()
        {
            if (this.serialport != null)
            {
                if (this.serialport.IsOpen)
                    return;
                if (this.debug)
                    StoreLogData.Instance.Store("Open Serial port " + this.serialport.PortName, DateTime.Now);
                this.serialport.BaudRate = this.baudRate;
                this.serialport.Parity = this.parity;
                this.serialport.StopBits = this.stopBits;
                this.serialport.WriteTimeout = 10000;
                this.serialport.ReadTimeout = this.connectTimeout;
                this.serialport.Open();
                this.connected = true;
            }
            else
            {
                if (!this.udpFlag)
                {
                    if (this.debug)
                        StoreLogData.Instance.Store("Open TCP-Socket, IP-Address: " + this.ipAddress + ", Port: " + (object)this.port, DateTime.Now);
                    this.tcpClient = new TcpClient();
                    IAsyncResult asyncResult = this.tcpClient.BeginConnect(this.ipAddress, this.port, (AsyncCallback)null, (object)null);
                    if (!asyncResult.AsyncWaitHandle.WaitOne(this.connectTimeout))
                        throw new Exception("connection timed out");
                    this.tcpClient.EndConnect(asyncResult);
                    this.stream = this.tcpClient.GetStream();
                    this.stream.ReadTimeout = this.connectTimeout;
                    this.connected = true;
                }
                else
                {
                    this.tcpClient = new TcpClient();
                    this.connected = true;
                }
                // ISSUE: reference to a compiler-generated field
                if (this.connectedChanged == null)
                    return;
                // ISSUE: reference to a compiler-generated field
                this.connectedChanged((object)this);
            }
        }

        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            while (this.receiveActive | this.dataReceived)
                Thread.Sleep(10);
            this.receiveActive = true;
            SerialPort serialPort = (SerialPort)sender;
            if (this.bytesToRead == 0)
            {
                serialPort.DiscardInBuffer();
                this.receiveActive = false;
            }
            else
            {
                this.readBuffer = new byte[256];
                int length = 0;
                DateTime now1 = DateTime.Now;
                DateTime now2;
                do
                {
                    try
                    {
                        now1 = DateTime.Now;
                        while (serialPort.BytesToRead == 0)
                        {
                            Thread.Sleep(1);
                            now2 = DateTime.Now;
                            if (checked(now2.Ticks - now1.Ticks) > 20000000L)
                                break;
                        }
                        int bytesToRead = serialPort.BytesToRead;
                        byte[] buffer = new byte[bytesToRead];
                        serialPort.Read(buffer, 0, bytesToRead);
                        Array.Copy((Array)buffer, 0, (Array)this.readBuffer, length, buffer.Length);
                        checked { length += buffer.Length; }
                    }
                    catch (Exception ex)
                    {
                    }
                    if (!(ModbusClient.DetectValidModbusFrame(this.readBuffer, length) | this.bytesToRead < length))
                        now2 = DateTime.Now;
                    else
                        break;
                }
                while (checked(now2.Ticks - now1.Ticks) < 20000000L);
                this.bytesToRead = 0;
                this.dataReceived = true;
                this.receiveActive = false;
                serialPort.DiscardInBuffer();
                this.receiveData = new byte[length];
                Array.Copy((Array)this.readBuffer, 0, (Array)this.receiveData, 0, length);
                if (this.debug)
                    StoreLogData.Instance.Store("Received Serial-Data: " + BitConverter.ToString(this.readBuffer), DateTime.Now);
                // ISSUE: reference to a compiler-generated field
                if (this.receiveDataChanged == null)
                    return;
                // ISSUE: reference to a compiler-generated field
                this.receiveDataChanged((object)this);
            }
        }

        public static bool DetectValidModbusFrame(byte[] readBuffer, int length)
        {
            if (length < 6 || readBuffer[0] < (byte)1 | readBuffer[0] > (byte)247)
                return false;
            byte[] numArray = new byte[2];
            byte[] bytes = BitConverter.GetBytes(ModbusClient.calculateCRC(readBuffer, checked((ushort)(length - 2)), 0));
            return !((int)bytes[0] != (int)readBuffer[checked(length - 2)] | (int)bytes[1] != (int)readBuffer[checked(length - 1)]);
        }

        public static ushort calculateCRC(byte[] data, ushort numberOfBytes, int startByte)
        {
            byte[] numArray1 = new byte[256]
            {
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 1,
        (byte) 192,
        (byte) 128,
        (byte) 65,
        (byte) 0,
        (byte) 193,
        (byte) 129,
        (byte) 64
            };
            byte[] numArray2 = new byte[256]
            {
        (byte) 0,
        (byte) 192,
        (byte) 193,
        (byte) 1,
        (byte) 195,
        (byte) 3,
        (byte) 2,
        (byte) 194,
        (byte) 198,
        (byte) 6,
        (byte) 7,
        (byte) 199,
        (byte) 5,
        (byte) 197,
        (byte) 196,
        (byte) 4,
        (byte) 204,
        (byte) 12,
        (byte) 13,
        (byte) 205,
        (byte) 15,
        (byte) 207,
        (byte) 206,
        (byte) 14,
        (byte) 10,
        (byte) 202,
        (byte) 203,
        (byte) 11,
        (byte) 201,
        (byte) 9,
        (byte) 8,
        (byte) 200,
        (byte) 216,
        (byte) 24,
        (byte) 25,
        (byte) 217,
        (byte) 27,
        (byte) 219,
        (byte) 218,
        (byte) 26,
        (byte) 30,
        (byte) 222,
        (byte) 223,
        (byte) 31,
        (byte) 221,
        (byte) 29,
        (byte) 28,
        (byte) 220,
        (byte) 20,
        (byte) 212,
        (byte) 213,
        (byte) 21,
        (byte) 215,
        (byte) 23,
        (byte) 22,
        (byte) 214,
        (byte) 210,
        (byte) 18,
        (byte) 19,
        (byte) 211,
        (byte) 17,
        (byte) 209,
        (byte) 208,
        (byte) 16,
        (byte) 240,
        (byte) 48,
        (byte) 49,
        (byte) 241,
        (byte) 51,
        (byte) 243,
        (byte) 242,
        (byte) 50,
        (byte) 54,
        (byte) 246,
        (byte) 247,
        (byte) 55,
        (byte) 245,
        (byte) 53,
        (byte) 52,
        (byte) 244,
        (byte) 60,
        (byte) 252,
        (byte) 253,
        (byte) 61,
        byte.MaxValue,
        (byte) 63,
        (byte) 62,
        (byte) 254,
        (byte) 250,
        (byte) 58,
        (byte) 59,
        (byte) 251,
        (byte) 57,
        (byte) 249,
        (byte) 248,
        (byte) 56,
        (byte) 40,
        (byte) 232,
        (byte) 233,
        (byte) 41,
        (byte) 235,
        (byte) 43,
        (byte) 42,
        (byte) 234,
        (byte) 238,
        (byte) 46,
        (byte) 47,
        (byte) 239,
        (byte) 45,
        (byte) 237,
        (byte) 236,
        (byte) 44,
        (byte) 228,
        (byte) 36,
        (byte) 37,
        (byte) 229,
        (byte) 39,
        (byte) 231,
        (byte) 230,
        (byte) 38,
        (byte) 34,
        (byte) 226,
        (byte) 227,
        (byte) 35,
        (byte) 225,
        (byte) 33,
        (byte) 32,
        (byte) 224,
        (byte) 160,
        (byte) 96,
        (byte) 97,
        (byte) 161,
        (byte) 99,
        (byte) 163,
        (byte) 162,
        (byte) 98,
        (byte) 102,
        (byte) 166,
        (byte) 167,
        (byte) 103,
        (byte) 165,
        (byte) 101,
        (byte) 100,
        (byte) 164,
        (byte) 108,
        (byte) 172,
        (byte) 173,
        (byte) 109,
        (byte) 175,
        (byte) 111,
        (byte) 110,
        (byte) 174,
        (byte) 170,
        (byte) 106,
        (byte) 107,
        (byte) 171,
        (byte) 105,
        (byte) 169,
        (byte) 168,
        (byte) 104,
        (byte) 120,
        (byte) 184,
        (byte) 185,
        (byte) 121,
        (byte) 187,
        (byte) 123,
        (byte) 122,
        (byte) 186,
        (byte) 190,
        (byte) 126,
        (byte) 127,
        (byte) 191,
        (byte) 125,
        (byte) 189,
        (byte) 188,
        (byte) 124,
        (byte) 180,
        (byte) 116,
        (byte) 117,
        (byte) 181,
        (byte) 119,
        (byte) 183,
        (byte) 182,
        (byte) 118,
        (byte) 114,
        (byte) 178,
        (byte) 179,
        (byte) 115,
        (byte) 177,
        (byte) 113,
        (byte) 112,
        (byte) 176,
        (byte) 80,
        (byte) 144,
        (byte) 145,
        (byte) 81,
        (byte) 147,
        (byte) 83,
        (byte) 82,
        (byte) 146,
        (byte) 150,
        (byte) 86,
        (byte) 87,
        (byte) 151,
        (byte) 85,
        (byte) 149,
        (byte) 148,
        (byte) 84,
        (byte) 156,
        (byte) 92,
        (byte) 93,
        (byte) 157,
        (byte) 95,
        (byte) 159,
        (byte) 158,
        (byte) 94,
        (byte) 90,
        (byte) 154,
        (byte) 155,
        (byte) 91,
        (byte) 153,
        (byte) 89,
        (byte) 88,
        (byte) 152,
        (byte) 136,
        (byte) 72,
        (byte) 73,
        (byte) 137,
        (byte) 75,
        (byte) 139,
        (byte) 138,
        (byte) 74,
        (byte) 78,
        (byte) 142,
        (byte) 143,
        (byte) 79,
        (byte) 141,
        (byte) 77,
        (byte) 76,
        (byte) 140,
        (byte) 68,
        (byte) 132,
        (byte) 133,
        (byte) 69,
        (byte) 135,
        (byte) 71,
        (byte) 70,
        (byte) 134,
        (byte) 130,
        (byte) 66,
        (byte) 67,
        (byte) 131,
        (byte) 65,
        (byte) 129,
        (byte) 128,
        (byte) 64
            };
            ushort num1 = numberOfBytes;
            byte maxValue = byte.MaxValue;
            byte num2 = byte.MaxValue;
            int num3 = 0;
            while (num1 > (ushort)0)
            {
                checked { --num1; }
                int index = (int)num2 ^ (int)data[checked(num3 + startByte)];
                num2 = checked((byte)((int)maxValue ^ (int)numArray1[index]));
                maxValue = numArray2[index];
                checked { ++num3; }
            }
            return checked((ushort)((int)maxValue << 8 | (int)num2));
        }

        ~ModbusClient()
        {
            if (this.debug)
                StoreLogData.Instance.Store("Destructor called - automatically disconnect", DateTime.Now);
            if (this.serialport != null)
            {
                if (!this.serialport.IsOpen)
                    return;
                this.serialport.Close();
            }
            else
            {
                if (!(this.tcpClient != null & !this.udpFlag))
                    return;
                if (this.stream != null)
                    this.stream.Close();
                this.tcpClient.Close();
            }
        }

        public bool Connected
        {
            get
            {
                if (this.serialport != null)
                    return this.serialport.IsOpen;
                if (this.udpFlag & this.tcpClient != null)
                    return true;
                if (this.tcpClient == null)
                    return false;
                return this.tcpClient.Connected;
            }
        }

        public void Disconnect()
        {
            if (this.debug)
                StoreLogData.Instance.Store(nameof(Disconnect), DateTime.Now);
            if (this.serialport != null)
            {
                if (!(this.serialport.IsOpen & !this.receiveActive))
                    return;
                this.serialport.Close();
            }
            else
            {
                if (this.stream != null)
                    this.stream.Close();
                if (this.tcpClient != null)
                    this.tcpClient.Close();
                this.connected = false;
                // ISSUE: reference to a compiler-generated field
                if (this.connectedChanged == null)
                    return;
                // ISSUE: reference to a compiler-generated field
                this.connectedChanged((object)this);
            }
        }

        public string IPAddress
        {
            get
            {
                return this.ipAddress;
            }
            set
            {
                this.ipAddress = value;
            }
        }

        public int Port
        {
            get
            {
                return this.port;
            }
            set
            {
                this.port = value;
            }
        }

        public bool UDPFlag
        {
            get
            {
                return this.udpFlag;
            }
            set
            {
                this.udpFlag = value;
            }
        }

        public byte UnitIdentifier
        {
            get
            {
                return this.unitIdentifier;
            }
            set
            {
                this.unitIdentifier = value;
            }
        }

        public int Baudrate
        {
            get
            {
                return this.baudRate;
            }
            set
            {
                this.baudRate = value;
            }
        }

        public Parity Parity
        {
            get
            {
                if (this.serialport != null)
                    return this.parity;
                return Parity.Even;
            }
            set
            {
                if (this.serialport == null)
                    return;
                this.parity = value;
            }
        }

        public StopBits StopBits
        {
            get
            {
                if (this.serialport != null)
                    return this.stopBits;
                return StopBits.One;
            }
            set
            {
                if (this.serialport == null)
                    return;
                this.stopBits = value;
            }
        }

        public int ConnectionTimeout
        {
            get
            {
                return this.connectTimeout;
            }
            set
            {
                this.connectTimeout = value;
            }
        }

        public string LogFileFilename
        {
            get
            {
                return StoreLogData.Instance.Filename;
            }
            set
            {
                StoreLogData.Instance.Filename = value;
                if (StoreLogData.Instance.Filename != null)
                    this.debug = true;
                else
                    this.debug = false;
            }
        }

        public enum RegisterOrder
        {
            LowHigh,
            HighLow,
        }

        public delegate void ReceiveDataChanged(object sender);

        public delegate void SendDataChanged(object sender);

        public delegate void ConnectedChanged(object sender);
    }
}
