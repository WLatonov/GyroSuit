#pragma once
#include "MyDefProject.h"
struct Settings {
	double  GravityEpsilon = 0.8, AngVelEpsilon = 0.05;
	double INS_gyr_coeff = 1;
	int INS_MODE = 8, INS_acc_max = 2, INS_gyr_max = 500, INS_mag_max = 1300;
	bool HelmIsNeeded = true;
	int PredictionPrev = 30, PredictionNext = 40;
	bool Read(string filename) {
		auto settings_map = SettingsReader::Read("settings.ini");
		if (settings_map.size() == 0) { return false; }
		SettingsReader::SetVar_int(settings_map, "INS_MODE", INS_MODE, 8);
		SettingsReader::SetVar_int(settings_map, "INS_acc_max", INS_acc_max, 2);
		SettingsReader::SetVar_int(settings_map, "INS_gyr_max", INS_gyr_max, 500);
		SettingsReader::SetVar_int(settings_map, "INS_mag_max", INS_mag_max, 1300);
		SettingsReader::SetVar_double(settings_map, "INS_gyr_coeff", INS_gyr_coeff, 1);
		SettingsReader::SetVar_double(settings_map, "GravityEpsilon", GravityEpsilon, 0.8);
		SettingsReader::SetVar_double(settings_map, "AngVelEpsilon", AngVelEpsilon, 0.05);
		SettingsReader::SetVar_bool(settings_map, "HelmIsNeeded", HelmIsNeeded, true);
		SettingsReader::SetVar_int(settings_map, "PredictionPrev", PredictionPrev, 30);
		SettingsReader::SetVar_int(settings_map, "PredictionNext", PredictionNext, 40);
		return true;
	}
};
