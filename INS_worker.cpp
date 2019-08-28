#include "INS_worker.h"
#include "MyDefProject.h"

using namespace std;
using byte = INS_worker::byte; using word = INS_worker::word;
const INS_worker::word INS_worker::AccelRangeRamp[] = { 2, 4, 8, 16 };//count of g
const INS_worker::word INS_worker::GyroRangeRamp[] = { 250, 500, 1000, 2000 };// degrees per second
const INS_worker::word INS_worker::MagRangeRamp[] = { 880, 1300, 1900, 2500 };
const double INS_worker::TempScale = 1e-2f;
const double INS_worker::AccScale = 1e-4f;
const double INS_worker::GyrScale = 1e-4f;
const double INS_worker::MagScale = 1e-4f;

struct TrackerSample {
	int acc_x, acc_y, acc_z;
	int gyr_x, gyr_y, gyr_z;
};
struct TrackerPackage {
	byte was_samples, recv_samples;
	unsigned short	timestamp;
	unsigned short	last_command_id;
	short	temperature;
	TrackerSample samples[3];
	short	mag_x, mag_y, mag_z;
};


void INS_worker::SendKeepAliveMsg(word time) {
	byte buf[5];
	buf[0] = 8; buf[1] = 0; buf[2] = 0;
	buf[3] = INS_worker::byte(time & 0xFF); buf[4] = byte(time >> 8);
	hid_send_feature_report(handle, buf, 5);
	last_time = HighPerformanceTimer();
}

void UnpackSensor(const INS_worker::byte* buf, int* x, int* y, int* z) {
	// Sign extending trick
	// from http://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
	struct { int x : 21; } s;

	*x = s.x = (buf[0] << 13) | (buf[1] << 5) | ((buf[2] & 0xF8) >> 3);
	*y = s.x = ((buf[2] & 0x07) << 18) | (buf[3] << 10) | (buf[4] << 2) |
		((buf[5] & 0xC0) >> 6);
	*z = s.x = ((buf[5] & 0x3F) << 15) | (buf[6] << 7) | (buf[7] >> 1);
}
word DecodeWord(const byte* buffer) { return (word(buffer[1]) << 8) | word(buffer[0]); }
short DecodeShort(const byte* buffer) { return (short(buffer[1]) << 8) | short(buffer[0]); }

bool DecodeRawINSData(byte* buf, int size, TrackerPackage& pack, int& S) {
	::memset(&pack, 0, sizeof(pack));
	if (size < 62) return false;
	if ((int)*(buf) != 1) return false;
	pack.was_samples = buf[1];
	pack.timestamp = DecodeWord(buf + 2);
	pack.last_command_id = DecodeWord(buf + 4);
	pack.temperature = DecodeShort(buf + 6);
	pack.recv_samples = (pack.was_samples > 2) ? 3 : pack.was_samples;
	if (pack.recv_samples != 1){ ++S; /*cout << "#"; */}
	for (byte i = 0; i < pack.recv_samples; i++) {
		UnpackSensor(buf + 8 + 16 * i, &pack.samples[i].acc_x, &pack.samples[i].acc_y, &pack.samples[i].acc_z);
		UnpackSensor(buf + 16 + 16 * i, &pack.samples[i].gyr_x, &pack.samples[i].gyr_y, &pack.samples[i].gyr_z);
	}
	pack.mag_x = DecodeShort(buf + 56);
	pack.mag_y = DecodeShort(buf + 58);
	pack.mag_z = DecodeShort(buf + 60);
	return true;
}

INS_worker::INS_worker() { last_time = 0; }
INS_worker::~INS_worker() {
	CloseDevice();
}
std::wstring INS_worker::GetSerialWString()const {
	string str = GetSerialString();
	return wstring(str.begin(), str.end());
}
std::string INS_worker::GetSerialString()const {
	return MakeString() << hex << uppercase << setfill('0') << setw(12) << serial_number;
}
bool INS_worker::OpenDevice(INT64 serial_number) {
	wchar_t wstr[50];
	if (serial_number != 0) {
		this->serial_number = serial_number;
		wstring str=GetSerialWString();
		handle = hid_open(vendor_id, device_id, str.c_str());
	} else
		handle = hid_open(vendor_id, device_id, NULL);
	if (!handle) return false;
	current_data.clear();
	int res = hid_get_serial_number_string(handle, wstr, 50);
	if (res < 0) { CloseDevice(); return false; }
	INT64 serial = _wcstoi64(wstr, NULL, 16);
	this->serial_number = serial;
	SendKeepAliveMsg();
	return true;
}
void INS_worker::CloseDevice() {
	if (handle != NULL) hid_close(handle);
	handle = NULL;
}
bool INS_worker::TakeData(INS_Data& data, int& S, bool reopen_device) {
	int Shift = 0;
	double now_time = HighPerformanceTimer();
	if (handle == NULL && reopen_device && fabs(now_time - last_time) > 200) { OpenDevice(serial_number); last_time = now_time; return false; }
	if (handle == NULL) return false;
	if (current_data.size() != 0) {
		data = current_data.back();
		current_data.pop_back();
		return true;
	}
	INS_Data tmp;
	TrackerPackage pack;
	while (true) {
		int res = hid_read(handle, hid_buf, sizeof(hid_buf));
		if (res < 0) { cerr << "\nLost device!!!\n" << endl; CloseDevice(); return false; }
		if (res == 0) return false;
		if (DecodeRawINSData(hid_buf, res, pack, Shift)) if (pack.was_samples > 0) break;
	}

	if (fabs(now_time - last_time) > KeepAliveTimeout)
		SendKeepAliveMsg();
	tmp.delta_step = 1;
	tmp.recv_timestamp = pack.timestamp;
	tmp.temp = pack.temperature * TempScale;
	tmp.mag[0] = pack.mag_x * MagScale; tmp.mag[1] = pack.mag_z * MagScale; tmp.mag[2] = pack.mag_y * MagScale; // error in Oculus: Y and Z swapped
	for (int i = pack.recv_samples - 1; i >= 0; --i) {
		tmp.recv_timestamp = pack.timestamp + i;
		tmp.acc[0] = pack.samples[i].acc_x * AccScale; tmp.acc[1] = pack.samples[i].acc_y * AccScale; tmp.acc[2] = pack.samples[i].acc_z * AccScale;
		tmp.gyr[0] = pack.samples[i].gyr_x * GyrScale; tmp.gyr[1] = pack.samples[i].gyr_y * GyrScale; tmp.gyr[2] = pack.samples[i].gyr_z * GyrScale;
		current_data.push_back(tmp);
	}
	current_data.back().delta_step = max(pack.was_samples - 2, 1);
	data = current_data.back();
	current_data.pop_back();
	S = Shift;
	return true;
}

bool INS_worker::SetFlagsRaw(int flag) {
	byte buffer[8];
	::memset(buffer, 0, sizeof(buffer));
	buffer[0] = 2; buffer[3] = flag;
	buffer[5] = byte(10000 & 0xFF); buffer[6] = byte(10000 >> 8);
	return hid_send_feature_report(handle, buffer, sizeof(buffer)) > 0;
}
bool INS_worker::SetParamsRaw(int acc, int gyr, int mag) {
	byte buffer[9];
	::memset(buffer, 0, sizeof(buffer));
	buffer[0] = 4;
	buffer[3] = acc;
	buffer[4] = byte(gyr & 0xFF);
	buffer[5] = byte(gyr >> 8);
	buffer[6] = byte(mag & 0xFF);
	buffer[7] = byte(mag >> 8);
	return hid_send_feature_report(handle, buffer, sizeof(buffer)) > 0;
}
bool INS_worker::SetFlags(INS_flags flag) {
	return SetFlagsRaw(flag);
}
bool INS_worker::SetParams(AccSens acc, GyrSens gyr, MagSens mag) {
	return SetParamsRaw(acc, gyr, mag);
}


vector<INT64> INS_worker::Enumerate(bool NeedLongNumber) {
	vector<INT64> res;
	hid_device_info *devs, *cur_dev;
	cur_dev = devs = hid_enumerate(vendor_id, device_id);
	while (cur_dev) {
		INT64 serial = _wcstoi64(cur_dev->serial_number, NULL, 16);
		res.push_back(serial);
		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
	return res;
}


