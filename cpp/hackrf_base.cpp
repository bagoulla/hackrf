#include "hackrf_base.h"

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

    The following class functions are for the base class for the device class. To
    customize any of these functions, do not modify them here. Instead, overload them
    on the child class

******************************************************************************************/

hackrf_base::hackrf_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl) :
    frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>(devMgr_ior, id, lbl, sftwrPrfl),
    ThreadedComponent()
{
    construct();
}

hackrf_base::hackrf_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>(devMgr_ior, id, lbl, sftwrPrfl, compDev),
    ThreadedComponent()
{
    construct();
}

hackrf_base::hackrf_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>(devMgr_ior, id, lbl, sftwrPrfl, capacities),
    ThreadedComponent()
{
    construct();
}

hackrf_base::hackrf_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
    ThreadedComponent()
{
    construct();
}

hackrf_base::~hackrf_base()
{
    delete RFInfo_in;
    RFInfo_in = 0;
    delete DigitalTuner_in;
    DigitalTuner_in = 0;
    delete dataOctet_out;
    dataOctet_out = 0;
}

void hackrf_base::construct()
{
    loadProperties();

    RFInfo_in = new frontend::InRFInfoPort("RFInfo_in", this);
    addPort("RFInfo_in", RFInfo_in);
    DigitalTuner_in = new frontend::InDigitalTunerPort("DigitalTuner_in", this);
    addPort("DigitalTuner_in", DigitalTuner_in);
    dataOctet_out = new bulkio::OutOctetPort("dataOctet_out");
    addPort("dataOctet_out", dataOctet_out);

}

/*******************************************************************************************
    Framework-level functions
    These functions are generally called by the framework to perform housekeeping.
*******************************************************************************************/
void hackrf_base::start() throw (CORBA::SystemException, CF::Resource::StartError)
{
    frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>::start();
    ThreadedComponent::startThread();
}

void hackrf_base::stop() throw (CORBA::SystemException, CF::Resource::StopError)
{
    frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>::stop();
    if (!ThreadedComponent::stopThread()) {
        throw CF::Resource::StopError(CF::CF_NOTSET, "Processing thread did not die");
    }
}

void hackrf_base::releaseObject() throw (CORBA::SystemException, CF::LifeCycle::ReleaseError)
{
    // This function clears the device running condition so main shuts down everything
    try {
        stop();
    } catch (CF::Resource::StopError& ex) {
        // TODO - this should probably be logged instead of ignored
    }

    frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>::releaseObject();
}

void hackrf_base::loadProperties()
{
    device_kind = "FRONTEND::TUNER";
    addProperty(serial_number,
                "No HackRF Found",
                "serial_number",
                "serial_number",
                "readonly",
                "",
                "external",
                "property");

    frontend_listener_allocation = frontend::frontend_listener_allocation_struct();
    frontend_tuner_allocation = frontend::frontend_tuner_allocation_struct();
    addProperty(gain_settings,
                gain_settings_struct(),
                "gain_settings",
                "gain_settings",
                "readwrite",
                "",
                "external",
                "property");

}

/* This sets the number of entries in the frontend_tuner_status struct sequence property
 * as well as the tuner_allocation_ids vector. Call this function during initialization
 */
void hackrf_base::setNumChannels(size_t num)
{
    this->setNumChannels(num, "RX_DIGITIZER");
}
/* This sets the number of entries in the frontend_tuner_status struct sequence property
 * as well as the tuner_allocation_ids vector. Call this function during initialization
 */

void hackrf_base::setNumChannels(size_t num, std::string tuner_type)
{
    frontend_tuner_status.clear();
    frontend_tuner_status.resize(num);
    tuner_allocation_ids.clear();
    tuner_allocation_ids.resize(num);
    for (std::vector<frontend_tuner_status_struct_struct>::iterator iter=frontend_tuner_status.begin(); iter!=frontend_tuner_status.end(); iter++) {
        iter->enabled = false;
        iter->tuner_type = tuner_type;
    }
}

void hackrf_base::frontendTunerStatusChanged(const std::vector<frontend_tuner_status_struct_struct>* oldValue, const std::vector<frontend_tuner_status_struct_struct>* newValue)
{
    this->tuner_allocation_ids.resize(this->frontend_tuner_status.size());
}

CF::Properties* hackrf_base::getTunerStatus(const std::string& allocation_id)
{
    CF::Properties* tmpVal = new CF::Properties();
    long tuner_id = getTunerMapping(allocation_id);
    if (tuner_id < 0)
        throw FRONTEND::FrontendException(("ERROR: ID: " + std::string(allocation_id) + " IS NOT ASSOCIATED WITH ANY TUNER!").c_str());
    CORBA::Any prop;
    prop <<= *(static_cast<frontend_tuner_status_struct_struct*>(&this->frontend_tuner_status[tuner_id]));
    prop >>= tmpVal;

    CF::Properties_var tmp = new CF::Properties(*tmpVal);
    return tmp._retn();
}

void hackrf_base::assignListener(const std::string& listen_alloc_id, const std::string& allocation_id)
{
    // find control allocation_id
    std::string existing_alloc_id = allocation_id;
    std::map<std::string,std::string>::iterator existing_listener;
    while ((existing_listener=listeners.find(existing_alloc_id)) != listeners.end())
        existing_alloc_id = existing_listener->second;
    listeners[listen_alloc_id] = existing_alloc_id;

}

void hackrf_base::removeListener(const std::string& listen_alloc_id)
{
    if (listeners.find(listen_alloc_id) != listeners.end()) {
        listeners.erase(listen_alloc_id);
    }
}
void hackrf_base::removeAllocationIdRouting(const size_t tuner_id) {
}

