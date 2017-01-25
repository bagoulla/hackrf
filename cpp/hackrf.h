#ifndef HACKRF_I_IMPL_H
#define HACKRF_I_IMPL_H

#include "hackrf_base.h"
#include <libhackrf/hackrf.h>
#include <boost/thread/mutex.hpp>

#define DEFAULT_STREAMID "DEFAULT_STREAM_ID"

// Values taken from https://github.com/mossmann/hackrf/blob/master/host/hackrf-tools/src/hackrf_transfer.c
#define FREQ_ONE_MHZ (1000000ll)

#define DEFAULT_FREQ_HZ (900000000) /* 900MHz */
#define FREQ_MIN_HZ	(0ull) /* 0 Hz */
#define FREQ_MAX_HZ	(7250000000ll) /* 7250MHz */
#define IF_MIN_HZ (2150000000ll)
#define IF_MAX_HZ (2750000000ll)
#define LO_MIN_HZ (84375000ll)
#define LO_MAX_HZ (5400000000ll)
#define DEFAULT_LO_HZ (1000000000ll)

#define DEFAULT_SAMPLE_RATE_HZ (10000000) /* 10MHz default sample rate */
#define MIN_SAMPLE_RATE_HZ (2000000) /* 2MHz */
#define MAX_SAMPLE_RATE_HZ (20000000) /* 20MHz */

#define DEFAULT_BASEBAND_FILTER_BANDWIDTH (5000000) /* 5MHz default */

#define BASEBAND_FILTER_BW_MIN (1750000)  /* 1.75 MHz min value */
#define BASEBAND_FILTER_BW_MAX (28000000) /* 28 MHz max value */

const uint32_t AVAILABLE_BANDWIDTHS[] = { 1750000, 2500000, 3500000, 5000000, 5500000,
		6000000, 7000000, 8000000, 9000000, 10000000, 12000000, 14000000,
		15000000, 20000000, 24000000, 28000000, 0 };

struct rx_context {
	bulkio::OutOctetPort * pPort;
	BULKIO::StreamSRI *sri;
};

class hackrf_i : public hackrf_base
{
    ENABLE_LOGGING
    public:
        hackrf_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        hackrf_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        hackrf_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        hackrf_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        void start() throw (CF::Resource::StartError, CORBA::SystemException);
        void stop() throw (CF::Resource::StopError, CORBA::SystemException);
        ~hackrf_i();

        void constructor();

        int serviceFunction();

    protected:
        std::string getTunerType(const std::string& allocation_id);
        bool getTunerDeviceControl(const std::string& allocation_id);
        std::string getTunerGroupId(const std::string& allocation_id);
        std::string getTunerRfFlowId(const std::string& allocation_id);
        double getTunerCenterFrequency(const std::string& allocation_id);
        void setTunerCenterFrequency(const std::string& allocation_id, double freq);
        double getTunerBandwidth(const std::string& allocation_id);
        void setTunerBandwidth(const std::string& allocation_id, double bw);
        bool getTunerAgcEnable(const std::string& allocation_id);
        void setTunerAgcEnable(const std::string& allocation_id, bool enable);
        float getTunerGain(const std::string& allocation_id);
        void setTunerGain(const std::string& allocation_id, float gain);
        long getTunerReferenceSource(const std::string& allocation_id);
        void setTunerReferenceSource(const std::string& allocation_id, long source);
        bool getTunerEnable(const std::string& allocation_id);
        void setTunerEnable(const std::string& allocation_id, bool enable);
        double getTunerOutputSampleRate(const std::string& allocation_id);
        void setTunerOutputSampleRate(const std::string& allocation_id, double sr);
        std::string get_rf_flow_id(const std::string& port_name);
        void set_rf_flow_id(const std::string& port_name, const std::string& id);
        frontend::RFInfoPkt get_rfinfo_pkt(const std::string& port_name);
        void set_rfinfo_pkt(const std::string& port_name, const frontend::RFInfoPkt& pkt);
        void set_gain_struct(gain_settings_struct request);

    private:
        // these are pure virtual, must be implemented here
        void deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        void deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceSetTuning(const frontend::frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        static int hackrf_call_back(hackrf_transfer* transfer);
        hackrf_device * pHackRfDevice;
        hackrf_device_list_t * pHackRfDeviceList;
        boost::mutex mStartStopMutex;
        BULKIO::StreamSRI mSri;
        rx_context mRxContext;


};

#endif // HACKRF_I_IMPL_H
