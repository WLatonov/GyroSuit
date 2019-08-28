#pragma once
#include "MyDefProject.h"
#include "hidapi.h"
#include "utils.h"
#include "Settings.h"

struct INS_Data {
	double temp;
	vector3 acc, mag, gyr;
	unsigned short recv_timestamp;
	unsigned char delta_step;
};


class INS_worker {
public:
	typedef unsigned char byte;
	typedef unsigned short word;
	enum INS_flags {
		Flag_RawMode = 0x01,
		Flag_CallibrationTest = 0x02, // Internal test mode
		Flag_UseCallibration = 0x04,
		Flag_AutoCallibration = 0x08,
		Flag_MotionKeepAlive = 0x10,
		Flag_CommandKeepAlive = 0x20,
		Flag_SensorCoordinates = 0x40
	};
	enum AccSens { acc_2_g = 2, acc_4_g = 4, acc_8_g = 8, acc_16_g = 16 };
	enum GyrSens { gyr_250_deg_per_sec = 250, gyr_500_deg_per_sec = 500, gyr_1000_deg_per_sec = 1000, gyr_2000_deg_per_sec = 2000 };
	enum MagSens { mag_880 = 880, mag_1300 = 1300, mag_1900 = 1900, mag_2500 = 2500 };
	static const int vendor_id = 0x2833;
	static const int device_id = 0x0001;
	static const int KeepAliveTimeout = 5000;
	static const word AccelRangeRamp[], GyroRangeRamp[], MagRangeRamp[];
	static const double TempScale, AccScale, GyrScale, MagScale;

	hid_device *handle = NULL;
	INT64 serial_number;
	double last_time;

	INS_worker();
	~INS_worker();
	static std::vector<INT64> Enumerate(bool NeedLongNumber);
	bool OpenDevice(INT64 serial_number = 0);
	void CloseDevice();
	bool SetFlags(INS_flags flag);
	bool SetParams(AccSens acc, GyrSens gyr, MagSens mag);
	bool SetFlagsRaw(int flag);
	bool SetParamsRaw(int acc, int gyr, int mag);
	bool TakeData(INS_Data& data, int& S, bool reopen_device = false);
	void SendKeepAliveMsg(word ms = 10000);
	std::wstring GetSerialWString()const;
	std::string GetSerialString()const;
private:
	byte hid_buf[1024];
	std::vector<INS_Data> current_data;
};

