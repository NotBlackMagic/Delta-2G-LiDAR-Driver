using System;
using System.IO.Ports;
using System.Threading;

public class LiDARConnection {
	//Serial Port Variables
	string port = "COM3";
	int baudrate = 115200;
	bool rxUSBThreadRun = true;
	Thread rxUSBThread;
	SerialPort serialPort = new SerialPort();

	//Scan variables
	int scanSamplesIndex;
	double[] scanSamplesSignalQuality;
	double[] scanSamplesRange;

	//Delta-2G Frame Structure
	struct Delta2GFrame {
		public int frameHeader;			//Frame Header: 1 byte
		public int frameLength;			//Frame Length: 2 bytes, from frame header to checksum (excluded)
		public int protocolVersion;		//Protocol Version: 1 byte
		public int frameType;			//Frame Type: 1 byte
		public int commandWord;			//Command Word: 1 byte, identifier to distinguish parameters
		public int parameterLength;		//Parameter Length: 2 bytes, length of the parameter field
		public byte[] parameters;		//Parameter Field
		public int checksum;			//Checksum: 2 bytes
	}

	//Delta-2G Frame Characteristics
	//Constant Frame Parts values
	const int frameHeader = 0xAA;                       //Frame Header value
	const int protocolVersion = 0x01;                   //Protocol Version value
	const int frameType = 0x61;							//Frame Type value
	//Scan Characteristics
	const int scanSteps = 15;							//How many steps/frames each full 360deg scan is composed of
	//Received value scaling
	const double rotationSpeedScale = 0.05 * 60;        //Convert received value to RPM (LSB: 0.05 rps)
	const double angleScale = 0.01;                     //Convert received value to degrees (LSB: 0.01 degrees)
	const double rangeScale = 0.25 * 0.001;				//Convert received value to meters (LSB: 0.25 mm)

	Action<double[], double[], int> onNewScanReceiveCallback;
	Action<int> onNewRPMReceiveCallback;

	public LiDARConnection() {

	}

	~LiDARConnection() {
		this.Disconnect();
	}

	public bool Connect(string port, int baudrate, Action<double[], double[], int> scanCallback, Action<int> rpmCallback) {
		if (serialPort.IsOpen == false) {
			try {
				serialPort.PortName = port;
				serialPort.BaudRate = baudrate;
				serialPort.DataBits = 8;
				serialPort.StopBits = StopBits.One;
				serialPort.Parity = Parity.None;
				serialPort.NewLine = "\n";
				serialPort.Open();

				this.port = port;
				this.baudrate = baudrate;
				this.onNewScanReceiveCallback = scanCallback;
				this.onNewRPMReceiveCallback = rpmCallback;

				rxUSBThreadRun = true;
				rxUSBThread = new Thread(USBRXThread);
				rxUSBThread.Start();
			}
			catch (System.Exception ex) {
				Console.WriteLine("ERROR: Serial Connect Error" + ex.Message);
				return false;
			}
		}
		else {
			return false;
		}
		return true;
	}

	public bool Disconnect() {
		if (serialPort.IsOpen == true) {
			//Tell RX thread to stop
			rxUSBThreadRun = false;
			//Wait for thread to stop
			rxUSBThread.Join();
			//Close serial port
			serialPort.Close();
		}
		else {
			return false;
		}
		return true;
	}

	public bool IsConnected() {
		return serialPort.IsOpen;
	}

	private void USBRXThread() {
		UInt16 checksum = 0;
		int status = 0;
		int index = 0;
		Delta2GFrame lidarFrame = new Delta2GFrame() { frameLength = 0, parameterLength = 0};

		byte[] rxBuffer = new byte[2048];
		while (rxUSBThreadRun) {
			if (serialPort.IsOpen) {
				int rxLength = 0;
				try {
					rxLength = serialPort.BaseStream.Read(rxBuffer, 0, rxBuffer.Length);
				}
				catch {
					Console.WriteLine("ERROR: serialPort.BaseStream.Read()");
					return;
				}

				for (int i = 0; i < rxLength; i++) {
					switch (status) {
						case 0:
							//1st frame byte: Frame Header
							lidarFrame.frameHeader = rxBuffer[i];
							if(lidarFrame.frameHeader == frameHeader) {
								//Valid Header
								status = 1;
							}
							else {
								Console.WriteLine("ERROR: Frame Header Failed");
							}
							//Reset checksum, new frame start
							checksum = 0;
							break;
						case 1:
							//2nd frame byte: Frame Length MSB
							lidarFrame.frameLength = (rxBuffer[i] << 8);
							status = 2;
							break;
						case 2:
							//3rd frame byte: Frame Length LSB
							lidarFrame.frameLength += rxBuffer[i];
							status = 3;
							break;
						case 3:
							//4th frame byte: Protocol Version
							lidarFrame.protocolVersion = rxBuffer[i];
							if(lidarFrame.protocolVersion == protocolVersion) {
								//Valid Protocol Version
								status = 4;
							}
							else {
								Console.WriteLine("ERROR: Frame Protocol Version Failed");
								status = 0;
							}
							break;
						case 4:
							//5th frame byte: Frame Type
							lidarFrame.frameType = rxBuffer[i];
							if (lidarFrame.frameType == frameType) {
								//Valid Frame Type
								status = 5;
							}
							else {
								Console.WriteLine("ERROR: Frame Type Failed");
								status = 0;
							}
							break;
						case 5:
							//6th frame byte: Command Word
							lidarFrame.commandWord = rxBuffer[i];
							status = 6;
							break;
						case 6:
							//7th frame byte: Parameter Length MSB
							lidarFrame.parameterLength = (rxBuffer[i] << 8);
							status = 7;
							break;
						case 7:
							//8th frame byte: Parameter Length LSB
							lidarFrame.parameterLength += rxBuffer[i];
							lidarFrame.parameters = new byte[lidarFrame.parameterLength];
							index = 0;
							status = 8;
							break;
						case 8:
							//9th+ frame bytes: Parameters
							lidarFrame.parameters[index++] = rxBuffer[i];
							if(index == lidarFrame.parameterLength) {
								//End of parameter frame bytes
								status = 9;
							}
							break;
						case 9:
							//N+1 frame byte: Checksum MSB
							lidarFrame.checksum = (rxBuffer[i] << 8);
							status = 10;
							break;
						case 10:
							//N+2 frame byte: Checksum LSB
							lidarFrame.checksum += rxBuffer[i];
							//End of frame reached
							//Compare received and calculated frame checksum
							if(lidarFrame.checksum == checksum) {
								//Checksum match: Valid frame
								LiDARFrameProcessing(lidarFrame);
							}
							else {
								//Checksum missmatach: Invalid frame
								Console.WriteLine("ERROR: Frame Checksum Failed");
							}
							status = 0;
							break;
						default:
							status = 0;
							break;
					}

					//Calculate current frame checksum, all bytes excluding the last 2, which are the checksum
					if(status < 10) {
						checksum += rxBuffer[i];
					}
				}
			}
		}
	}

	private void LiDARFrameProcessing(Delta2GFrame frame) {
		int cmdCode = frame.commandWord;
		switch (cmdCode) {
			case 0xAE: {
				//Device Health Information: Speed Failure
				int rpm = (int)(frame.parameters[0] * rotationSpeedScale);
				onNewRPMReceiveCallback(rpm);
				break;
			}
			case 0xAD: {
				//1st: Rotation speed (1 byte)
				int rpm = (int)(frame.parameters[0] * rotationSpeedScale);
				onNewRPMReceiveCallback(rpm);

				//2nd: Zero Offset angle (2 bytes)
				int offsetAngleInt = (frame.parameters[1] << 8) + (frame.parameters[2]);
				double offsetAngle = offsetAngleInt * angleScale;

				//3rd: Start angle of current data freame (2 bytes)
				int startAngleInt = (frame.parameters[3] << 8) + frame.parameters[4];
				double startAngle = startAngleInt * angleScale;

				//Calculate number of samples in current frame
				int samplesCnt = (frame.parameterLength - 5) / 3;

				//Calculate current angle index of a full frame: For Delta-2G each full rotation has 15 frames
				int frameIndex = (int)(startAngle / (360.0 / scanSteps));

				if (frameIndex == 0) {
					//New scan started
					scanSamplesRange = new double[samplesCnt * scanSteps];
					scanSamplesSignalQuality = new double[samplesCnt * scanSteps];
					scanSamplesIndex = 0;
				}

				//4th: LiDAR samples, each sample has: Signal Value/Quality (1 byte), Distance Value (2 bytes)
				for (int i = 0; i < samplesCnt; i++) {
					int signalQuality = frame.parameters[5 + (i * 3)];
					int distance = (frame.parameters[5 + (i * 3) + 1] << 8) + frame.parameters[5 + (i * 3) + 2];
					if (scanSamplesRange != null && scanSamplesIndex < scanSamplesRange.Length) {
						scanSamplesSignalQuality[scanSamplesIndex] = signalQuality;
						scanSamplesRange[scanSamplesIndex++] = distance * rangeScale;
					}
				}

				if (frameIndex == (scanSteps - 1)) {
					//Scan complete
					if (scanSamplesRange != null && scanSamplesIndex == scanSamplesRange.Length) {
						//Valid scan array
						onNewScanReceiveCallback(scanSamplesRange, scanSamplesSignalQuality, scanSamplesRange.Length);
					}
				}
				break;
			}
		}
	}
}
