#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<Kinect.h>

#include<windows.h>
#include<process.h>
#include<Ole2.h>
#include<iostream>
#include<stdio.h>
#include<signal.h>
#include<conio.h>
#include<string>
#include<math.h>

#define WAVEFORMAT_PCM 0x0003 //0x0003 is float
#define CHANNEL 1
#define SAMPLE_RATE 16000
#define BIT_RATE 32 //32 is ieee754 32bit float

using namespace std;
using namespace cv;

IKinectSensor* sensor;
//color
IColorFrameReader* reader;
IColorFrameSource* framesource;
HANDLE hThread;
//depth
IDepthFrameReader* dReader;
IDepthFrameSource* dFrameSource;
//audio
IAudioSource* aSource;
IAudioBeamFrameReader* aReader;
WAITABLE_HANDLE aFrameEvent;
HANDLE aTerminalEvent;
HANDLE aThread;


int recordCheck = 0; // if recordCheck == 0, no record. 1: need to initalize video file. 2: while recording, 3: needs to be turned off.
int monitorCheck = 0; //0: monitor turns off, 1: need to be turned on, and after thread creating, become 2. 2: is on.
int exitCheck = 0;
int windowCount = 0;
int aRecordCheck = 0;

int width = 0;
int height = 0;
VideoWriter writer;

bool InitKinect();
Mat getKinectData(bool* check);
unsigned WINAPI kinectFunc(void* arg); //kinect function
unsigned WINAPI keyboardControl(void* arg);
DWORD WINAPI audioThread(void* arg);
void processAudio(IAudioBeamSubFrame* pAudioBeamSubFrame);

HANDLE vRecord;
HANDLE aRecord;

typedef struct {
	unsigned char ChunkID[4]; //contains the letters "RIFF" ascii code.
	unsigned int ChunkSize;
	unsigned char Format[4];
} RIFF;

typedef struct {
	unsigned char ChunkID[4];
	unsigned int ChunkSize;
	unsigned short AudioFormat;
	unsigned short NumChannels;
	unsigned int SampleRate;
	unsigned int AvgByteRate;
	unsigned short BlockAlign;
	unsigned short BitPerSample;
} FMT;

typedef struct {
	char ChunkID[4];
	unsigned int ChunkSize;
} DATA;

typedef struct {
	RIFF Riff;
	FMT Fmt;
	DATA Data;
} HEADER;

class headerWriter {
public:
	char* fname;
	//DWORD sampleCount;
	unsigned int sampleCount;
	FILE* fd;
	HEADER* header;
	headerWriter() {
		fname = NULL;
		sampleCount = 0;
		fd = NULL;
		header = (HEADER*)malloc(sizeof(HEADER));
	}
	headerWriter(char* s) {
		fname = s;
		sampleCount = 0;
		fd = NULL;
		header = (HEADER*)malloc(sizeof(HEADER));
	}
	~headerWriter() {
		//free(header);
	}
	bool open() {
		fd = fopen(fname, "w");
		if (fd != NULL)
			return true;
		return false;
	}
	void writeHeader() {
		//cout << sampleCount << endl;
		//RIFF
		memcpy(header->Riff.ChunkID, "RIFF", 4);
		header->Riff.ChunkSize = sizeof(float)*sampleCount+36;
		memcpy(header->Riff.Format, "WAVE", 4);
		
		//FMT
		memcpy(header->Fmt.ChunkID, "fmt ", 4);
		header->Fmt.ChunkSize = 0x10;
		header->Fmt.AudioFormat = WAVEFORMAT_PCM;
		header->Fmt.NumChannels = CHANNEL;
		header->Fmt.SampleRate = SAMPLE_RATE;
		header->Fmt.AvgByteRate = SAMPLE_RATE * CHANNEL * BIT_RATE / 8;
		header->Fmt.BlockAlign = CHANNEL * BIT_RATE / 8;
		header->Fmt.BitPerSample = BIT_RATE;
		//need to revise one sample block to 16 bit.

		//DATA
		memcpy(header->Data.ChunkID, "data", 4);
		header->Data.ChunkSize = sizeof(float)*sampleCount;
		fwrite(header, sizeof(HEADER), 1, fd);
		
	}

	void writeData(char* dataNm) {//It may be called when, record needs to be turned off. man
		FILE* fData = fopen(dataNm, "r");
		//FILE* fData = fopen("C:\\Users\\Owner\\Desktop\\test.pcm", "r");
		//sampleCount = 80000;
		int temp = (int)sampleCount;
		if (fData == NULL) {
			cout << "data open failed in writeData" << endl;
			return;
		}
		while (true) {
			//float buffer[20];
			float* buffer = (float*)malloc(sizeof(float) * 20);
			if (temp == 0) break;
			int size = min(temp, 20);
			fread(&buffer[0], sizeof(float), size, fData);
			//need to -1~+1, IEEE754 32 bit float to -32768~32767 short int. but i do this in the process audio.
			fwrite(&buffer[0], sizeof(float), size, fd);
			temp -= size;
			free(buffer);
		}
		//free(buffer);
		fclose(fData);
		fclose(fd);
		cout << "audio record stop!" << endl;
	}
};

headerWriter hw = headerWriter();
FILE* pcmFd;
char* pcmFn;

int main() {
	//keyboardControl thread start.
	//cout << getBuildInformation() << endl;
	//unsigned threadID1;
	//unsigned threadID2;;
	if (InitKinect())
		cout << "init kinect success" << endl;
	else {
		cout << "init kinect failed" << endl;
		return -1;
	}


	hThread = (HANDLE)_beginthreadex(NULL, 0, keyboardControl, NULL, 0, NULL);
	if (hThread == 0) {
		cerr << "ERROR : failed making keyboardControl thread" << endl;
		return -1;
	}
	while (1) {
		if (exitCheck == 1)
			break;
		if (monitorCheck == 1) {
			windowCount++;
			hThread = (HANDLE)_beginthreadex(NULL, 0, kinectFunc, NULL, 0, NULL);
			if (hThread == 0) {
				cerr << "ERROR : failed making kinectFunc thread" << endl;
				return -1;
			}
			/*aThread = CreateThread(NULL, 0, &audioThread, NULL, 0, NULL);
			if (aThread == NULL) {
				cerr << "ERROR : failed making aThread"<<endl;
				return -1;
			}*/
			//thread begin, function is kinectFunc.
			monitorCheck = 2;
		}
		if (aRecordCheck == 1) {
			char* fn = "C:\\Users\\Owner\\Desktop\\output.wav";
			pcmFn = "C:\\Users\\Owner\\Desktop\\output.pcm";
			hw = headerWriter(fn);
			hw.open();
			pcmFd = fopen(pcmFn, "w");
			aRecordCheck = 2;
		}
		else if (aRecordCheck == 4) {
			hw.writeHeader();
			hw.writeData(pcmFn);
			//save the recordfile.
			aRecordCheck = 0;
		}
		if (recordCheck == 1) {
			//video file initializing.
			Size size = Size(1920, 1080);
			double fps = 15.0;
			writer.open("C:\\Users\\Owner\\Desktop\\output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, size, true);
			//VideoWriter video("outcpp.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, size, true);
			//writer.open("output.avi", -1, fps, size);
			recordCheck = 2;
		}
		else if (recordCheck == 4) {//kinectFunc is first finished.
			//save the recordfile.
			writer.release();
			recordCheck = 0;
		}
	}
	cout << "wait for the exit..." << endl;
	Sleep(1000);
	return 0;
}

unsigned WINAPI kinectFunc(void *arg) {
	//InitKinect(); //kinect initializing
	while (1) { //frame printing.
		bool check = false;
		Mat ret = getKinectData(&check);
		if (check) {
			string s = "test";
			s.append(to_string(windowCount));
			namedWindow(s, WINDOW_AUTOSIZE);
			if (recordCheck == 2) {
				Mat temp;
				cvtColor(ret, temp, COLOR_BGRA2RGBA);
				writer.write(temp);
			}
			imshow(s, ret);
			if (waitKey(10) == VK_ESCAPE)
				break;
		}
		/*if (recordCheck == 2) {
			Mat temp;
			cvtColor(ret, temp, COLOR_BGRA2RGBA);
			writer.write(temp);
		}*/
		if (recordCheck == 3) {//stop recording.
			recordCheck = 4;
		}
		if (monitorCheck == 0) {
			break;
		}
	}
	int result = 0;
	//sensor->Release();
	//reader->Release();
	return 0;
}

DWORD WINAPI audioThread(void* arg) {
	HRESULT hr = S_OK;
	DWORD timeout = 2000; // In msec. If we don't get a new frame event by this time, display an error message. Any number of seconds will do here.
	UINT32 subFrameCount = 0;
	IAudioBeamFrameArrivedEventArgs* pAudioBeamFrameArrivedEventArgs = NULL;
	IAudioBeamFrameReference* pAudioBeamFrameReference = NULL;
	IAudioBeamFrameList* pAudioBeamFrameList = NULL;
	IAudioBeamFrame* pAudioBeamFrame = NULL;
	HANDLE handles[] = { aTerminalEvent, (HANDLE)aFrameEvent};

	int count = 0;
	while (1)
	{
		//if (monitorCheck == 0)
		//	break;
		// Wait for a new audio frame
		DWORD result = WaitForMultipleObjects(_countof(handles), handles, FALSE, timeout); //if 2 value is on, then first value returns.
		if (WAIT_OBJECT_0 == result) {//Frame is arrived. The byte array is a mono 32-bit IEEE floating point PCM stream sampled at 16 kHz 
			cout << "1" << endl;
			break;
		} else if (WAIT_OBJECT_0+1 ==result/*&&aRecordCheck==2*/) {
			//cout << "2" << endl;
			hr = aReader->GetFrameArrivedEventData(aFrameEvent, &pAudioBeamFrameArrivedEventArgs);
			if (FAILED(hr))
			{
				cout << "1: getFrameArrivedEventData error" << endl;
				break;
			}
			hr = pAudioBeamFrameArrivedEventArgs->get_FrameReference(&pAudioBeamFrameReference);

			if (FAILED(hr))
			{
				cout << "2: get_FrameReference error" << endl;
				pAudioBeamFrameArrivedEventArgs->Release();
				break;
			}
			hr = pAudioBeamFrameReference->AcquireBeamFrames(&pAudioBeamFrameList);
			if (FAILED(hr))
			{
				// Only one audio beam is currently supported
				cout << "3: AcquireBeamFrames error" << endl;
				pAudioBeamFrameReference->Release();
				pAudioBeamFrameArrivedEventArgs->Release();
				break;
			}
			hr = pAudioBeamFrameList->OpenAudioBeamFrame(0, &pAudioBeamFrame);
			if (FAILED(hr))
			{
				cout << "4: openAudioBeamFrame error" << endl;
				pAudioBeamFrameList->Release();
				pAudioBeamFrameReference->Release();
				pAudioBeamFrameArrivedEventArgs->Release();
				break;
			}
			hr = pAudioBeamFrame->get_SubFrameCount(&subFrameCount);
			if (SUCCEEDED(hr) && subFrameCount > 0)
			{
				
				for (UINT32 i = 0; i < subFrameCount; i++)
				{
					// Process all subframes
					IAudioBeamSubFrame* pAudioBeamSubFrame = NULL;

					hr = pAudioBeamFrame->GetSubFrame(i, &pAudioBeamSubFrame);
					if (aRecordCheck == 3) {
						fclose(pcmFd);
						aRecordCheck = 4;
					}
					else if (SUCCEEDED(hr)&&aRecordCheck==2)
					{
						//count++;
						//cout << count << endl;
						processAudio(pAudioBeamSubFrame);
					}
					
					pAudioBeamSubFrame->Release();
				}
			}

			pAudioBeamFrame->Release();
			pAudioBeamFrameList->Release();
			pAudioBeamFrameReference->Release();
			pAudioBeamFrameArrivedEventArgs->Release();

		}
		else if (result == WAIT_TIMEOUT) {//error
			cout << "audio timeout!" << endl;
			hr = E_FAIL;
			break;
		} else {
			hr = E_FAIL;
			break;
		}
	}
	//when audio frame comes or terminal signal is on, process them.
	return 0;
}

void processAudio(IAudioBeamSubFrame* pAudioBeamSubFrame) { //monitor check and record check is needed.
	HRESULT hr = S_OK;
	float* videoBuffer = NULL; //buffer
	UINT cbRead = 0; //byte number

	hr = pAudioBeamSubFrame->AccessUnderlyingBuffer(&cbRead, (BYTE **)&videoBuffer);
	if (FAILED(hr)) {
		cout << "accessunderlyingBuffer failed!" << endl;
		return;
	}
	else if (cbRead > 0) {
		DWORD sampleCount = cbRead / (sizeof(float));
		unsigned int tt = (unsigned int)sampleCount;
		//write part.
		hw.sampleCount += tt;
		fwrite(videoBuffer, sizeof(float), sampleCount, pcmFd);
	}
	return;
}


unsigned WINAPI keyboardControl(void* arg) {
	while (1) {
		//cout << "keyboard Control" << endl;
		if (exitCheck == 1)
			break;
		if (_kbhit()) {
			int key = _getch();
			switch (key) {
			case 'r':
				//mutex get part.
				if (recordCheck == 0) { //no record
					cout << "starting video recording" << endl;
					recordCheck = 1;
				}
				else if(recordCheck == 2) {//on the record
					cout << "stoping recording" << endl;
					recordCheck = 3;
				}
				if (aRecordCheck == 0) {
					cout << "starting audio recording" << endl;
					aRecordCheck = 1;
				}
				else if (aRecordCheck == 2) {
					cout << "stopping audio recording" << endl;
					aRecordCheck = 3;
				}
				break;
				//mutex release part
			case 's':
				//mutex get part
				if (monitorCheck == 0) {
					cout << "open the monitor" << endl;
					monitorCheck = 1;
				}
				else {
					cout << "close the monitor" << endl;
					//if(hThread != NULL)
					//	CloseHandle(hThread);
					/*if(aTerminalEvent != NULL) {
						BOOL s = SetEvent(aTerminalEvent);
						if (s == TRUE) cout << "success" << endl;
						else cout << "no" << endl;
					}*/
					destroyAllWindows();
					monitorCheck = 0;
				}
				//mutex release part
				break;
			case 'x':
				exitCheck = 1;
				break;
			default :
				break;
			}
		}
	}
	return 0;
}

Mat getKinectData(bool* check) {
	IColorFrame* frame = NULL;
	check[0] = false;
	Mat ret;
	HRESULT hr = reader->AcquireLatestFrame(&frame);
	if (SUCCEEDED(hr)) {
		IFrameDescription* frDes = NULL;
		TIMESPAN t;
		frame->get_RelativeTime(&t);
		int width = 0;
		int height = 0;
		hr = frame->get_FrameDescription(&frDes);
		if (FAILED(hr)) return ret;
		frDes->get_Height(&height); //1080
		frDes->get_Width(&width); //1920
		ret = Mat(height, width, CV_8UC4);
		hr = frame->CopyConvertedFrameDataToArray(width*height * 4, reinterpret_cast<BYTE *>(ret.data), ColorImageFormat_Bgra);
		if (FAILED(hr)) return ret;
		check[0] = true;
		frame->Release();
		frDes->Release();
		frDes = NULL;
	}
	return ret;
}

bool InitKinect() {//init kinect varaibles.
	if (FAILED(GetDefaultKinectSensor(&sensor))) return false; //function error
	if (sensor) {
		HRESULT hr = sensor->Open();
		framesource = NULL;
		sensor->get_ColorFrameSource(&framesource);
		hr = framesource->OpenReader(&reader);
		if (framesource) {
			framesource->Release();
			framesource = NULL;
		}
		hr = sensor->get_AudioSource(&aSource);
		if (FAILED(hr)) {
			cout << "get_AudioSource failed" << endl;
			return false;
		}
		hr = aSource->OpenReader(&aReader);
		if (FAILED(hr)) {
			cout << "audio openReader failed" << endl;
			return false;
		}
		hr = aReader->SubscribeFrameArrived(&aFrameEvent);
		if (FAILED(hr)) {
			cout << "subscribeFrameArrived error" << endl;
			return false;
		}
		aTerminalEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
		if(aTerminalEvent==NULL)
			hr = HRESULT_FROM_WIN32(GetLastError());
		if (FAILED(hr)) {
			cout << "createEvent failed" << endl;
			return false;
		}
		aThread = CreateThread(NULL, 0, &audioThread, NULL, 0, NULL);
		if (aThread == NULL) {
			cerr << "ERROR : failed making aThread" << endl;
			return -1;
		}
		return true;
	} else return false;
}