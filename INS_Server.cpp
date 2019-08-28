#include "MyDefProject.h"
#include <unordered_set>
#include "INS_worker.h"
#include "Integrator.h"
#include "Settings.h"
#include "SensorFilter.h"
#include "Transformator_DoubledBino.h"

#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib,"Ws2_32.lib")


#define  G                     9.8155

#define  gravityEpsilon        0.55
#define  angVelEpsilon         0.1
#define  tiltPeriod            60
#define  maxTiltError          0.05
#define  minTiltError          0.005

#define  maxVar                0.07
#define  maxNumberofDeviations 58.0          

enum Calibration
{
	Simple = 1,
	Complex = 2
};

enum Body_id
{
	Head = 1,
	Right_Upper_Arm = 2,
	Right_Forearm = 3,
	Right_Hand = 4,
	Left_Upper_Arm = 5,
	Left_Forearm = 6,
	Left_Hand = 7,
	Head2 = 8
};

struct MMF_Data {
	struct Body {
		vector3 pos; quat rot; int IsEnabled;
	};
	std::unordered_map<string, Body> bodies;
	void Write(byte* data) {
		binary_stream bout(data + 4);
		bout << short(bodies.size());
		for (auto kv : bodies) {
			bout << kv.first;
			bout << kv.second.pos.x << kv.second.pos.y << kv.second.pos.z;
			bout << kv.second.rot.w << kv.second.rot.x << kv.second.rot.y << kv.second.rot.z;
			bout << kv.second.IsEnabled;
		}
		binary_stream(data) << (bout.pos + 4);
	}
};

vector<INS_worker> INS_swarm;
unordered_map<INT64, Integrator> INS_swarm_res;
Settings settings;
MMF_Data MMF_data;
MemoryMappedFile MMF;
int Prev;
int Next;

unordered_map<INT64, atomic_bool> AlertFlags;
unordered_map<int, INT64> IndexPCB;
unordered_map<INT64, string> PCB;
unordered_map<INT64, Transformator_DoubledBino> transformators;
unordered_map<INT64, SensorFilter> FAccW;
unordered_map<INT64, SensorFilter> FiltRawAcc;
unordered_map<INT64, SensorFilter> Gyroscopes;

unordered_map<INT64, std::wofstream> Output;

bool InitSystem() {
	cout << "Read settings .. ";
	if (!settings.Read("settings.ini")) { cout << "FAIL (file settings.ini not found)\n"; } else { cout << "OK\n"; }
	Integrator::AngVelEpsilon = settings.AngVelEpsilon; Integrator::GravityEpsilon = settings.GravityEpsilon;
	INS_swarm.clear();
	cout << "Read PCB Settings" << endl;

	INT64 Id0 = 0, Id1 = 0;
	std::ifstream PCBSettings("PCB_Settings.txt");

		PCBSettings >> Id0 >> Id1;
		PCB[Id0] = "Left_Ocular";
		PCB[Id1] = "Right_Ocular";

	cout << "Start enumeration ...";
	auto ins_enum = INS_worker::Enumerate(settings.HelmIsNeeded);
	cout << " found "<<ins_enum.size()<<" ins\nInitializing ins " << endl;
	std::unordered_set<INT64> test_set;
	INS_swarm.resize(ins_enum.size());
	int pos = 0;

	Prev = settings.PredictionPrev;
	Next = settings.PredictionNext;
	int numberPCB = 0;
	for (INT64 num : ins_enum) {
		if (test_set.find(num) != test_set.end()) { cout << "\nError dublicate serial numbers\n"; return false; }
		if (!INS_swarm[pos].OpenDevice(num)) { cout << "\nInternal error with serial numbers (cannot open device)\n"; return false; }

		else { INS_swarm[pos].SetFlagsRaw(settings.INS_MODE); cout << PCB[num]; }

		INS_swarm[pos].SetParamsRaw(settings.INS_acc_max, settings.INS_gyr_max, settings.INS_mag_max); 
		INS_swarm_res[num] = Integrator();
		FAccW[num] = SensorFilter(20);
		FiltRawAcc[num] = SensorFilter(110);
		Gyroscopes[num] = SensorFilter(110);
		transformators[num] = Transformator_DoubledBino(num);
		AlertFlags[num] = false;
		IndexPCB[numberPCB] = num;
		
		numberPCB++;

		std::wstring NUM = std::to_wstring(num);
		Output[num] = std::wofstream(L"Output_" + NUM + L".txt");
		Output[num] << std::fixed << std::setw(10) << std::setprecision(6);

		cout << " ." << endl; pos++;
	}
	cout << endl << " OK\n";

	cout << "Try to create MMF .. ";
	if (!MMF.Create("Local\\INS_tracker_data", 4096)) {
		cout << "FAIL\nTry to open existing MMF .. ";
		if (!MMF.Open("Local\\INS_tracker_data")) {
			cout << "FAIL\n";
			return false;
		} else { cout << "OK\n"; }
	} else { cout << "OK\n"; }
	return true;
}

bool Work(INS_Data& data, INS_worker& ins) {

	int S = 0;
	if (!ins.TakeData(data, S, true))
	{
		cout << PCB[ins.serial_number] << " - Cannot take data!" << endl;
		return false;
	}		
		if (settings.INS_MODE == INS_worker::INS_flags::Flag_RawMode) { transformators[ins.serial_number].SimpleTransform(data); }

		double magbufX = data.mag.x;
		double magbufY = data.mag.y;
		double magbufZ = data.mag.z;
		data.mag.x = -magbufX;
		data.mag.y = magbufZ;
		data.mag.z = magbufY;
		double ang = data.mag.Normalized().Angle(data.acc.Normalized());

		Gyroscopes[ins.serial_number].AddElement(data.gyr);
		FiltRawAcc[ins.serial_number].AddElement(data.acc);
		//Output[ins.serial_number] << data.acc.x << "\t" << data.acc.y << "\t" << data.acc.z << "\t" << data.gyr.x << "\t" << data.gyr.y << "\t" << data.gyr.z << "\t" << data.mag.x << "\t" << data.mag.y << "\t" << data.mag.z << endl;

	INS_swarm_res[ins.serial_number].Step(data);
	Gyroscopes[ins.serial_number].IntegrateAddition();
	Gyroscopes[ins.serial_number].IntegrateLinearPrediction_Holt_5(Next);

	MMF_data.bodies[ins.GetSerialString()] = { INS_swarm_res[ins.serial_number].pos, INS_swarm_res[ins.serial_number].q * Gyroscopes[ins.serial_number].GetAdd(), 1};
	quat q = INS_swarm_res[ins.serial_number].q * Gyroscopes[ins.serial_number].GetAdd() * Gyroscopes[ins.serial_number].GetPrediction();

	Gyroscopes[ins.serial_number].SetPredictionToZero();

	return true;
}
#include <conio.h>
#include <thread>

void TestThread(INS_worker& dev) {
	int number = 0;
	int lc = 0;
	time_t rawtime;

	int timeflag = 0;
	int startTime = 0;

	double TiltErrorAngle = 0.0;
	vector3 TiltErrorAxis = { 0.0, 0.0, 0.0 };
	int TiltCondCount = 0;

	while (true) {

		INS_Data data;
		
		time(&rawtime);

		if (!Work(data, dev))
		{
			if (AlertFlags[dev.serial_number] == false) { AlertFlags[dev.serial_number] = true; }
			for (int k = 0; k < 100; ++k)
			{
				if (INS_swarm[k].serial_number == dev.serial_number) { number = k; break; }
			}
			
			continue; 
		}
		else 
		{
			if (AlertFlags[dev.serial_number] == true)
			{
				INS_swarm[number].OpenDevice(dev.serial_number);
				INS_swarm[number].SetFlagsRaw(settings.INS_MODE);
				INS_swarm[number].SetParamsRaw(settings.INS_acc_max, settings.INS_gyr_max, settings.INS_mag_max);
				AlertFlags[dev.serial_number] = false;
			}
		}

		if (timeflag == 0)
		{
			startTime = data.recv_timestamp;
			timeflag = 1;
		}

	}
}

int DisablesDetector()
{
	while (true)
	{
		for (int i = 0; i < INS_swarm.size(); i++) {
			if (AlertFlags[IndexPCB[i]]) { cout << "Disabled: " << IndexPCB[i] << "  " << PCB[IndexPCB[i]] << endl; }
			else { cout << "Enabled:  " << IndexPCB[i] << "  " << PCB[IndexPCB[i]] << endl; }
		}
		cout << endl;
		sleep_for(milliseconds(1000));
	}
}

int ClientForVizualization()
{
	WSADATA wsaData;
	SOCKET SendRecvSocket;  
	sockaddr_in ServerAddr; 
	int err, maxlen = 512;  
	char buf[128];
	string totalMessage;
	string localBuf;
	
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	SendRecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	ServerAddr.sin_family = AF_INET;
	inet_pton(AF_INET, "127.0.0.1", &(ServerAddr.sin_addr));
	ServerAddr.sin_port = htons(10990);

	int bodiesCount = 0;

	while (true) {
		totalMessage.clear();
		std::fill(std::begin(buf), std::end(buf), 0);				
		bodiesCount = MMF_data.bodies.size(); 
		localBuf = std::to_string(bodiesCount); totalMessage += localBuf; totalMessage += " ";

		for (auto body : MMF_data.bodies) {
			totalMessage += body.first; totalMessage += " ";

			localBuf = std::to_string(body.second.pos.x); totalMessage += localBuf; totalMessage += " ";
			localBuf = std::to_string(body.second.pos.y); totalMessage += localBuf; totalMessage += " ";
			localBuf = std::to_string(body.second.pos.z); totalMessage += localBuf; totalMessage += " ";

			localBuf = std::to_string(body.second.rot.w); totalMessage += localBuf; totalMessage += " ";
			localBuf = std::to_string(body.second.rot.x); totalMessage += localBuf; totalMessage += " ";
			localBuf = std::to_string(body.second.rot.y); totalMessage += localBuf; totalMessage += " ";
			localBuf = std::to_string(body.second.rot.z); totalMessage += localBuf; totalMessage += " ";

			localBuf = std::to_string(body.second.IsEnabled); totalMessage += localBuf; totalMessage += " ";
		}	

		strncpy(buf, totalMessage.c_str(), sizeof(buf));

		int a = sendto(SendRecvSocket, buf, sizeof(buf), 0, (sockaddr *)&ServerAddr, sizeof(ServerAddr));
		sleep_for(milliseconds(30));
	}

	return 0;
}

int main() {
	CHECKMACROS(InitSystem());
	vector<std::thread> workers(INS_swarm.size());
	for (int i = 0; i < INS_swarm.size(); i++) {
		workers[i] = std::thread(TestThread, std::ref(INS_swarm[i]));
	}

	std::thread DisablesDetector(DisablesDetector);
	std::thread testObject(ClientForVizualization);

	while (true) {
		sleep_for(milliseconds(100));

		if (_kbhit())
			switch (_getch()) {
			case 27: abort();
			case 32:
				for (auto& ins_res : INS_swarm_res) {
					cout << " cor len = " << ins_res.second.avg_g_last.Norm() << endl;
					ins_res.second.q = quat::RotationFromTo(ins_res.second.avg_g_last, ins_res.second.grav);
				}
				break;
			case 49: { Prev -= 10; if (Prev == 20) { Prev = 30; } cout << " Prev = " << Prev << endl; }
			case 50: { Prev += 10; if (Prev == 110) { Prev = 100; } cout << " Prev = " << Prev << endl; }
			case 51: { Next -= 10; if (Next == 0) { Next = 10; } cout << " Next = " << Next << endl; }
			case 52: { Next += 10; if (Next == 510) { Next = 500; } cout << " Next = " << Next << endl; }
		}
	}

}
int main2(){
	CHECKMACROS(InitSystem());
	unordered_map<int, int> counters,lastcounters;
	for (auto& ins : INS_swarm) {
		counters[ins.serial_number] = 0;
		lastcounters[ins.serial_number] = 0;
	}
	while (true) {
		bool is_str = 0;
		int cc = 0;
		for (auto& ins : INS_swarm) {
			INS_Data data;
			if (!Work(data, ins))continue;

			int lnum = lastcounters[ins.serial_number];
			int num = counters[ins.serial_number];
			if (lnum > data.recv_timestamp) lnum -= 65536;
			lastcounters[ins.serial_number] = data.recv_timestamp;
			if (data.recv_timestamp - lnum != 1) {
				cout << data.recv_timestamp - lnum << "(" << cc << ")\t";
				is_str = true;
			}
			cc++;
		}
		MMF.WorkOnData(std::bind(&MMF_Data::Write, &MMF_data, std::placeholders::_1), MemoryMappedFile::Write);
		if (is_str) cout << "\n";
		if (_kbhit())
			switch (_getch()) {
			case 27: return 0;
			case 32:
				for (auto& ins_res : INS_swarm_res)
					ins_res.second.q = quat::RotationFromTo(ins_res.second.avg_g, ins_res.second.grav);
				break;
		}

	}
	return 0;
}

