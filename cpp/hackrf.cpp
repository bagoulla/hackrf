/**************************************************************************

    This is the device code. This file contains the child class where
    custom functionality can be added to the device. Custom
    functionality to the base class can be extended here. Access to
    the ports can also be done from this class

**************************************************************************/

#include "hackrf.h"


PREPARE_LOGGING(hackrf_i)

hackrf_i::hackrf_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl) :
    hackrf_base(devMgr_ior, id, lbl, sftwrPrfl),pHackRfDevice(NULL),pHackRfDeviceList(NULL)
{
}

hackrf_i::hackrf_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    hackrf_base(devMgr_ior, id, lbl, sftwrPrfl, compDev),pHackRfDevice(NULL),pHackRfDeviceList(NULL)
{
}

hackrf_i::hackrf_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    hackrf_base(devMgr_ior, id, lbl, sftwrPrfl, capacities),pHackRfDevice(NULL),pHackRfDeviceList(NULL)
{
}

hackrf_i::hackrf_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    hackrf_base(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),pHackRfDevice(NULL),pHackRfDeviceList(NULL)
{
}

hackrf_i::~hackrf_i()
{
}

void hackrf_i::constructor()
{
    this->setNumChannels(1, "RX_DIGITIZER");
    mSri = bulkio::sri::create(DEFAULT_STREAMID);
    mRxContext.pPort = dataOctet_out;
    mSri.streamID = DEFAULT_STREAMID;
    mRxContext.sri = &mSri;

    setPropertyConfigureImpl(gain_settings, this, &hackrf_i::set_gain_struct);

    if(!started()){
        start();
    }
}

void hackrf_i::set_gain_struct(gain_settings_struct request) {
	LOG_INFO(hackrf_i, "setting gain");

	uint32_t vga_gain = request.vga_gain;
	uint32_t lna_gain = request.lna_gain;
	int ln_distance;

	int VALID_LN_GAINS[] = {0, 8, 16, 24, 32, 40};

	vga_gain = (vga_gain % 2 == 0) ? (vga_gain) : (vga_gain + 1); // force vga_gain to be even

	if (vga_gain > 62) {
		vga_gain = 62;
	} else if (vga_gain < 0) {
		vga_gain = 0;
	}

	if (lna_gain < 0) {
		lna_gain = 0;
	} else if (lna_gain > 40) {
		lna_gain = 40;
	}

	ln_distance = 41;
	uint32_t tmp_lna_gain = lna_gain;
	for (size_t i = 0; i < sizeof(VALID_LN_GAINS)/sizeof(VALID_LN_GAINS[0]); ++i) {
		if(ln_distance > abs(lna_gain - VALID_LN_GAINS[i])) {
			tmp_lna_gain = VALID_LN_GAINS[i];
			ln_distance = abs(lna_gain - VALID_LN_GAINS[i]);
		}
	}

	lna_gain = tmp_lna_gain;

	if (hackrf_set_amp_enable(pHackRfDevice, request.rf_amp_enabled) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred attempting to set rf amp gain to: " << request.rf_amp_enabled);
	}
	gain_settings.rf_amp_enabled = request.rf_amp_enabled;

	if (hackrf_set_lna_gain(pHackRfDevice, lna_gain) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred attempting to set lna_gain to: " << lna_gain);
	}
	gain_settings.lna_gain = lna_gain;

	if (hackrf_set_vga_gain(pHackRfDevice, vga_gain) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred attempting to set vga_gain to: " << vga_gain);
	}
	gain_settings.vga_gain = vga_gain;
}

void hackrf_i::start() throw (CF::Resource::StartError, CORBA::SystemException) {
	boost::mutex::scoped_lock lock(mStartStopMutex);

	if (started()) {
		LOG_INFO(hackrf_i, "Told to start when already starting.");
		return;
	}

	if (hackrf_init() != HACKRF_SUCCESS) {
		LOG_FATAL(hackrf_i, "Error initializing hackrf library");
		throw CF::Resource::StartError(CF::CF_ENODEV, "Error initializing hackrf library");
	}

	pHackRfDeviceList = hackrf_device_list();

	if (pHackRfDeviceList->devicecount == 0) {
		serial_number = "No HackRF Found";
		LOG_ERROR(hackrf_i, "Could not find a hackrf, is it plugged in?");
		hackrf_exit();
		throw CF::Resource::StartError(CF::CF_ENODEV, "Error finding hackrf, ensure it is plugged in.");
	} else if (pHackRfDeviceList->devicecount > 1) {
		LOG_WARN(hackrf_i, "Multiple hackrfs found, will use first hackrf found");
	}

	if (hackrf_device_list_open(pHackRfDeviceList, 0, &pHackRfDevice) != HACKRF_SUCCESS) {
		LOG_FATAL(hackrf_i, "Error opening hackrf device");
		hackrf_exit();
		throw CF::Resource::StartError(CF::CF_ENODEV, "Error opening hackrf device");
	}

	hackrf_set_freq(pHackRfDevice, DEFAULT_FREQ_HZ);
	mSri.keywords.length(2);
	mSri.keywords[0].id = "CHAN_RF";
	mSri.keywords[0].value <<= DEFAULT_FREQ_HZ;

	mSri.keywords[1].id = "COL_RF";
	mSri.keywords[1].value <<= DEFAULT_FREQ_HZ;

	hackrf_set_amp_enable(pHackRfDevice, (uint8_t) gain_settings.rf_amp_enabled);
	hackrf_set_antenna_enable(pHackRfDevice, 0);
	hackrf_set_baseband_filter_bandwidth(pHackRfDevice, DEFAULT_BASEBAND_FILTER_BANDWIDTH);
	hackrf_set_lna_gain(pHackRfDevice, gain_settings.lna_gain); // TODO: checks on this to see if it's correct / worked.
	hackrf_set_sample_rate(pHackRfDevice, DEFAULT_SAMPLE_RATE_HZ);
	mSri.xdelta = 1.0 / DEFAULT_SAMPLE_RATE_HZ;
	hackrf_set_vga_gain(pHackRfDevice, gain_settings.vga_gain); // TODO:checks on this to see if it's correct / worked.
	mSri.mode = 1; // Complex

	hackrf_base::start();
}

void hackrf_i::stop() throw (CF::Resource::StopError, CORBA::SystemException) {
	boost::mutex::scoped_lock lock(mStartStopMutex);

	if (pHackRfDevice) {
		hackrf_close(pHackRfDevice);
		pHackRfDevice = NULL;
	}

	hackrf_exit();

	hackrf_base::stop();
}

int hackrf_i::hackrf_i::serviceFunction() { return FINISH; }


int hackrf_i::hackrf_call_back(hackrf_transfer* transfer) {
	LOG_TRACE(hackrf_i, "Hackrf call back called with: " << transfer->buffer_length << " bytes");
	BULKIO::PrecisionUTCTime tstamp = bulkio::time::utils::now();
	rx_context * pRxContext = (rx_context *) transfer->rx_ctx;
	std::string streamId = std::string(pRxContext->sri->streamID);

	if (pRxContext->pPort->currentSRIs.count(streamId) == 0) {
		pRxContext->pPort->pushSRI(*pRxContext->sri);
	}

	pRxContext->pPort->pushPacket(transfer->buffer, transfer->buffer_length, tstamp, false, streamId);
	return 0;
}

void hackrf_i::deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id){
	LOG_INFO(hackrf_i, "deviceEnable called");

	if(hackrf_start_rx(pHackRfDevice, &hackrf_i::hackrf_call_back, &mRxContext) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Problem occurred trying to start the RX chain.");
		return;
	}

    fts.enabled = true;
    return;
}
void hackrf_i::deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id){
	LOG_INFO(hackrf_i, "deviceDisable called");
	if (hackrf_stop_rx(pHackRfDevice) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Problem occured trying to stop the RX chain.");
		return;
	}

    fts.enabled = false;
    return;
}
bool hackrf_i::deviceSetTuning(const frontend::frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id){
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
      At a minimum, bandwidth, center frequency, and sample_rate have to be set
      If the device is tuned to exactly what the request was, the code should be:
        fts.bandwidth = request.bandwidth;
        fts.center_frequency = request.center_frequency;
        fts.sample_rate = request.sample_rate;

    return true if the tuning succeeded, and false if it failed
    ************************************************************/
	LOG_INFO(hackrf_i, "deviceSetTuning called with id: " << request.allocation_id);

	float minAcceptableSampleRate = request.sample_rate;
	float maxAcceptableSampleRate = (1 + request.sample_rate_tolerance/100.0) * request.sample_rate;
	double minAcceptableBandwidth = request.bandwidth;
	double maxAcceptableBandwidth = (1 + request.bandwidth_tolerance/100.0) * request.bandwidth;

	uint32_t hackrfBw = DEFAULT_BASEBAND_FILTER_BANDWIDTH;
	uint32_t hackrfSampleRate = DEFAULT_SAMPLE_RATE_HZ;
	unsigned long int hackrfFrequency = (unsigned long int) request.center_frequency;

	if (request.bandwidth > 0) {

		for (size_t i = 0; i < sizeof(AVAILABLE_BANDWIDTHS)/sizeof(AVAILABLE_BANDWIDTHS[0]); ++i) {
			hackrfBw = AVAILABLE_BANDWIDTHS[i];
			if (hackrfBw >= minAcceptableBandwidth and hackrfBw < maxAcceptableBandwidth) {
				break;
			}
		}

		if (hackrfBw == 0) {
			LOG_WARN(hackrf_i, "Could not find acceptable bandwidth given request of: " << request.bandwidth);
			return false;
		}
	}

	if (request.sample_rate > 0) {
		// if the request isn't in the sample rate range return false
		if (minAcceptableSampleRate > MAX_SAMPLE_RATE_HZ or maxAcceptableSampleRate < MIN_SAMPLE_RATE_HZ) {
			LOG_WARN(hackrf_i, "Requested sample rate is outside of range");
			return false;
		}

		if (minAcceptableSampleRate < MIN_SAMPLE_RATE_HZ) {
			hackrfSampleRate = MIN_SAMPLE_RATE_HZ;
		} else if (minAcceptableSampleRate > MIN_SAMPLE_RATE_HZ) { // that's all the cases
			hackrfSampleRate = minAcceptableSampleRate;
		}
	}


	if (hackrfFrequency > FREQ_MAX_HZ or hackrfFrequency < FREQ_MIN_HZ) {
		LOG_WARN(hackrf_i, "Requested center frequency is outside of range");
		return false;
	}

	// Looks like everything has passed, we can set the values now.
	if (hackrf_set_freq(pHackRfDevice, hackrfFrequency) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred setting frequency, device in unknown state.");
		return false;
	}

	fts.center_frequency = hackrfFrequency;

	if (hackrf_set_baseband_filter_bandwidth(pHackRfDevice, hackrfBw) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred setting filter bandwidth, device in unknown state.");
		return false;
	}

	fts.bandwidth = hackrfBw;

	if (hackrf_set_sample_rate(pHackRfDevice, hackrfSampleRate) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred setting sample rate, device in unknown state.");
		return false;
	}

	fts.sample_rate = hackrfSampleRate;

	mSri.keywords[0].value <<= hackrfFrequency;
	mSri.keywords[1].value <<= hackrfFrequency;
	mSri.xdelta = 1.0 / hackrfSampleRate;

	dataOctet_out->pushSRI(mSri);

    return true;
}
bool hackrf_i::deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id) {
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    return true if the tune deletion succeeded, and false if it failed
    ************************************************************/
	LOG_INFO(hackrf_i, "deviceDeleteTuning called");
    return true;
}

/*************************************************************
Functions servicing the tuner control port
*************************************************************/
std::string hackrf_i::getTunerType(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].tuner_type;
}

bool hackrf_i::getTunerDeviceControl(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if (getControlAllocationId(idx) == allocation_id)
        return true;
    return false;
}

std::string hackrf_i::getTunerGroupId(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].group_id;
}

std::string hackrf_i::getTunerRfFlowId(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].rf_flow_id;
}

void hackrf_i::setTunerCenterFrequency(const std::string& allocation_id, double freq) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (freq<0) throw FRONTEND::BadParameterException();
    // set hardware to new value. Raise an exception if it's not possible

    unsigned long int hackrfFrequency = (unsigned long int) freq;
	if (hackrfFrequency > FREQ_MAX_HZ or hackrfFrequency < FREQ_MIN_HZ) {
		LOG_WARN(hackrf_i, "Requested center frequency is outside of range");
		throw FRONTEND::BadParameterException();
	}

	if (hackrf_set_freq(pHackRfDevice, hackrfFrequency) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred setting frequency, device in unknown state.");
		throw FRONTEND::BadParameterException();
	}

	mSri.keywords[0].value <<= hackrfFrequency;
	mSri.keywords[1].value <<= hackrfFrequency;
	dataOctet_out->pushSRI(mSri);

    this->frontend_tuner_status[idx].center_frequency = hackrfFrequency;
}

double hackrf_i::getTunerCenterFrequency(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].center_frequency;
}

void hackrf_i::setTunerBandwidth(const std::string& allocation_id, double bw) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (bw<=0) throw FRONTEND::BadParameterException();
    // set hardware to new value. Raise an exception if it's not possible


    uint32_t hackrfBw = 0;

	for (size_t i = 0; i < sizeof(AVAILABLE_BANDWIDTHS)/sizeof(AVAILABLE_BANDWIDTHS[0]); ++i) {
		hackrfBw = AVAILABLE_BANDWIDTHS[i];
		if (hackrfBw == bw) {
			break;
		}
	}

	if (hackrfBw == 0) {
		LOG_WARN(hackrf_i, "Could not find acceptable bandwidth given request. See list of possible bandwidths");
		throw FRONTEND::BadParameterException();
	}

	if (hackrf_set_baseband_filter_bandwidth(pHackRfDevice, hackrfBw) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred setting filter bandwidth, device in unknown state.");
		throw FRONTEND::BadParameterException();
	}

    this->frontend_tuner_status[idx].bandwidth = hackrfBw;
}

double hackrf_i::getTunerBandwidth(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].bandwidth;
}

void hackrf_i::setTunerAgcEnable(const std::string& allocation_id, bool enable)
{
    throw FRONTEND::NotSupportedException("setTunerAgcEnable not supported");
}

bool hackrf_i::getTunerAgcEnable(const std::string& allocation_id)
{
    throw FRONTEND::NotSupportedException("getTunerAgcEnable not supported");
}

void hackrf_i::setTunerGain(const std::string& allocation_id, float gain)
{
    throw FRONTEND::NotSupportedException("setTunerGain not supported");
}

float hackrf_i::getTunerGain(const std::string& allocation_id)
{
    throw FRONTEND::NotSupportedException("getTunerGain not supported");
}

void hackrf_i::setTunerReferenceSource(const std::string& allocation_id, long source)
{
    throw FRONTEND::NotSupportedException("setTunerReferenceSource not supported");
}

long hackrf_i::getTunerReferenceSource(const std::string& allocation_id)
{
    throw FRONTEND::NotSupportedException("getTunerReferenceSource not supported");
}

void hackrf_i::setTunerEnable(const std::string& allocation_id, bool enable) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    // set hardware to new value. Raise an exception if it's not possible
    this->frontend_tuner_status[idx].enabled = enable;
}

bool hackrf_i::getTunerEnable(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].enabled;
}

void hackrf_i::setTunerOutputSampleRate(const std::string& allocation_id, double sr) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (sr<=0) throw FRONTEND::BadParameterException();

    unsigned long int hackrfSampleRate = (unsigned long int) sr;

	// if the request isn't in the sample rate range return false
	if (hackrfSampleRate > MAX_SAMPLE_RATE_HZ or hackrfSampleRate < MIN_SAMPLE_RATE_HZ) {
		LOG_WARN(hackrf_i, "Requested sample rate is outside of range");
		throw FRONTEND::BadParameterException();
	}

	if (hackrf_set_sample_rate(pHackRfDevice, hackrfSampleRate) != HACKRF_SUCCESS) {
		LOG_ERROR(hackrf_i, "Error occurred setting sample rate, device in unknown state.");
		throw FRONTEND::BadParameterException();
	}

	mSri.xdelta = 1.0 / hackrfSampleRate;

	dataOctet_out->pushSRI(mSri);
    this->frontend_tuner_status[idx].sample_rate = hackrfSampleRate;
}

double hackrf_i::getTunerOutputSampleRate(const std::string& allocation_id){
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].sample_rate;
}

/*************************************************************
Functions servicing the RFInfo port(s)
- port_name is the port over which the call was received
*************************************************************/
std::string hackrf_i::get_rf_flow_id(const std::string& port_name)
{
    return std::string("none");
}

void hackrf_i::set_rf_flow_id(const std::string& port_name, const std::string& id)
{
}

frontend::RFInfoPkt hackrf_i::get_rfinfo_pkt(const std::string& port_name)
{
    frontend::RFInfoPkt pkt;
    return pkt;
}

void hackrf_i::set_rfinfo_pkt(const std::string& port_name, const frontend::RFInfoPkt &pkt)
{
}

